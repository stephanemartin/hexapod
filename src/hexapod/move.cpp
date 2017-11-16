
#include "move.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <Tone.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BeeperPin 4   // digital 4 used for beeper
#define BF_ERROR  100 // deep beep for error situations
#define BD_MED    50  // medium long beep duration

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!  If you hear buzzing or jittering, you went too far.
// These values are good for MG90S style small metal gear servos

#define SERVOMIN  190 // this is the 'minimum' pulse length count (out of 4096) [radj was 185/545]
#define SERVOMAX  540 // this is the 'maximum' pulse length count (out of 4096)

// Basic functions that move legs take a bit pattern
// indicating which legs to move. The legs are numbered
// clockwise starting with the right front leg being
// number zero, going around
// to the left legs, and finishing with the left front leg
// being number 5

#define NUM_LEGS 6

// Bit patterns for different combinations of legs
// bottom six bits. LSB is leg number 0

#define ALL_LEGS      0b111111
#define LEFT_LEGS     0b111000
#define RIGHT_LEGS    0b000111
#define TRIPOD1_LEGS  0b010101
#define TRIPOD2_LEGS  0b101010
#define FRONT_LEGS    0b100001
#define MIDDLE_LEGS   0b010010
#define BACK_LEGS     0b001100
#define NO_LEGS       0b0

// individual leg bitmasks
#define LEG0 0b1
#define LEG1 0b10
#define LEG2 0b100
#define LEG3 0b1000
#define LEG4 0b10000
#define LEG5 0b100000

#define LEG0BIT  0b1
#define LEG1BIT  0b10
#define LEG2BIT  0b100
#define LEG3BIT  0b1000
#define LEG4BIT  0b10000
#define LEG5BIT  0b100000

#define ISFRONTLEG(LEG) (LEG==0||LEG==5)
#define ISMIDLEG(LEG)   (LEG==1||LEG==4)
#define ISBACKLEG(LEG)  (LEG==2||LEG==3)
#define ISLEFTLEG(LEG)  (LEG==0||LEG==1||LEG==2)
#define ISRIGHTLEG(LEG) (LEG==3||LEG==4||LEG==5)

// default positions for knee and hip. Note that hip position is
// automatically reversed for the left side by the setHip function
// These are in degrees

#define KNEE_UP_MAX 180
#define KNEE_UP    150
#define KNEE_RELAX  120  // 80
#define KNEE_NEUTRAL 90 // 130
#define KNEE_CROUCH 110
#define KNEE_HALF_CROUCH 80
#define KNEE_STAND 30
#define KNEE_DOWN  30    // was 40
#define KNEE_TIPTOES 5
#define KNEE_FOLD 170

#define KNEE_SCAMPER (KNEE_NEUTRAL-20)

#define KNEE_TRIPOD_UP (KNEE_NEUTRAL-40)
#define KNEE_TRIPOD_ADJ 30

#define HIPSWING 25      // how far to swing hips on gaits like tripod or quadruped
#define HIPSMALLSWING 10  // when in fine adjust mode how far to move hips
#define HIPSWING_RIPPLE 20
#define HIP_FORWARD_MAX 175
#define HIP_FORWARD (HIP_NEUTRAL+HIPSWING)
#define HIP_FORWARD_SMALL (HIP_NEUTRAL+HIPSMALLSWING)
#define HIP_NEUTRAL 90
#define HIP_BACKWARD (HIP_NEUTRAL-HIPSWING)
#define HIP_BACKWARD_SMALL (HIP_NEUTRAL-HIPSMALLSWING)
#define HIP_BACKWARD_MAX 0
#define HIP_FORWARD_RIPPLE (HIP_NEUTRAL+HIPSWING_RIPPLE)
#define HIP_BACKWARD_RIPPLE (HIP_NEUTRAL-HIPSWING_RIPPLE)
#define HIP_FOLD 150

#define NOMOVE (-1)   // fake value meaning this aspect of the leg (knee or hip) shouldn't move

#define LEFT_START 3  // first leg that is on the left side
#define RIGHT_START 0 // first leg that is on the right side
#define KNEE_OFFSET 6 // add this to a leg number to get the knee servo number

// these modes are used to interpret incoming bluetooth commands

#define TRIPOD_CYCLE_TIME 750
#define RIPPLE_CYCLE_TIME 1800
#define FIGHT_CYCLE_TIME 660


#define BATTERYSAVER 5000   // milliseconds in stand mode before servos all detach to save power and heat buildup

short ServoPos[2*NUM_LEGS];
long startedStanding = 0;   // the last time we started standing, or reset to -1 if we didn't stand recently
long LastReceiveTime = 0;   // last time we got a bluetooth packet
long LastValidReceiveTime = 0;  // last time we got a completely valid packet including correct checksum
void flutter();

void beep(int f, int t) {
  if (f > 0 && t > 0) {
//    tone(BeeperPin, f, t);
  } else {
//    noTone(BeeperPin);
  }
}

void beep(int f) {
  beep(f, 250);
}

// This function sets the positions of both the knee and hip in 
// a single command.  For hip, the left side is reversed so
// forward direction is consistent.

// This function takes a bitmask to specify legs to move, note that
// the basic setHip and setKnee functions take leg numbers, not masks

// if a position is -1 then that means don't change that item

void setHip(int leg, int pos, int adj);
void setHipRaw(int leg, int pos);
void setKnee(int leg, int pos);
void setServo(int servonum, int position);
void boogie_woogie(int legs_flat, int submode, int timingfactor);


void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw) {
  for (int i = 0; i < NUM_LEGS; i++) {
    if (legmask & 0b1) {  // if the lowest bit is ON
      if (hip_pos != NOMOVE) {
        if (!raw) {
          setHip(i, hip_pos, adj);
        } else {
          setHipRaw(i, hip_pos);
        }
      }
      if (knee_pos != NOMOVE) {
        setKnee(i, knee_pos);
      }
    }
    legmask = (legmask>>1);  // shift down one bit position
  }
}


void setLeg(int legmask, int hip_pos, int knee_pos, int adj) {
  setLeg(legmask, hip_pos, knee_pos, adj, 0);  // use the non-raw version
}

// this version of setHip does no processing at all (for example
// to distinguish left from right sides)
void setHipRaw(int leg, int pos) {
  setServo(leg, pos);
}

// this version of setHip adjusts for left and right legs so
// that 0 degrees moves "forward" i.e. toward legs 5-0 which is
// nominally the front of the robot

void setHip(int leg, int pos) {
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }
  setHipRaw(leg, pos);
}

// this version of setHip adjusts not only for left and right,
// but also shifts the front legs a little back and the back legs
// forward to make a better balance for certain gaits like tripod or quadruped

void setHip(int leg, int pos, int adj) {
  if (ISFRONTLEG(leg)) {
    pos -= adj;
  } else if (ISBACKLEG(leg)) {
    pos += adj;
  }
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }

  setHipRaw(leg, pos);
}

void setKnee(int leg, int pos) {
  // find the knee associated with leg if this is not already a knee
  if (leg < KNEE_OFFSET) {
    leg += KNEE_OFFSET;
  }
  setServo(leg, pos);
}




void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod) {
  // use tripod groups to turn in place
  if (ccw) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_TURN_PHASES 6
#define FBSHIFT_TURN    40   // shift front legs back, back legs forward, this much
  
  long t = millis()%timeperiod;
  long phase = (NUM_TURN_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move clockwise
      // at the hips, while the rest of the legs move CCW at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0);
      break;
      
    case 4:
      // similar to phase 1, move raised legs CW and lowered legs CCW
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0);
      break;  
  }
  
}

void stand() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
}

void stand_90_degrees() {  // used to install servos, sets all servos to 90 degrees
  setLeg(ALL_LEGS, 90, 90, 0);
}

void laydown() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_UP, 0);
}

void tiptoes() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
}

void wave(int dpad) {
  
#define NUM_WAVE_PHASES 12
#define WAVE_CYCLE_TIME 900
#define KNEE_WAVE  60
  long t = millis()%WAVE_CYCLE_TIME;
  long phase = (NUM_WAVE_PHASES*t)/WAVE_CYCLE_TIME;

  if (dpad == 'b') {
    phase = 11-phase;  // go backwards
  }

  switch (dpad) {
    case 'f':
    case 'b':
      // swirl around
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0); // keep hips stable at 90 degrees
      if (phase < NUM_LEGS) {
        setKnee(phase, KNEE_WAVE);
      } else {
        setKnee(phase-NUM_LEGS, KNEE_STAND);
      }
      break;
    case 'l':
      // teeter totter around font/back legs
   
      if (phase < NUM_WAVE_PHASES/2) {
        setKnee(0, KNEE_TIPTOES);
        setKnee(5, KNEE_STAND);
        setHipRaw(0, HIP_FORWARD);
        setHipRaw(5, HIP_BACKWARD-40);
        setKnee(2, KNEE_TIPTOES);
        setKnee(3, KNEE_STAND);
        setHipRaw(2, HIP_BACKWARD);
        setHipRaw(3, HIP_FORWARD+40);
                
        setLeg(LEG1, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
      } else {
        setKnee(0, KNEE_STAND);
        setKnee(5, KNEE_TIPTOES);
        setHipRaw(0, HIP_FORWARD+40);
        setHipRaw(5, HIP_BACKWARD);
        setKnee(2, KNEE_STAND);
        setKnee(3, KNEE_TIPTOES);
        setHipRaw(2, HIP_BACKWARD-40);
        setHipRaw(3, HIP_FORWARD);
           
        setLeg(LEG1, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      }
      break;
    case 'r':
      // teeter totter around middle legs
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
      if (phase < NUM_LEGS) {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      } else {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);       
      }
      break;
    case 'w':
      // lay on ground and make legs go around in a wave
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0);
      int p = phase/2;
      for (int i = 0; i < NUM_LEGS; i++) {
        if (i == p) {
          setKnee(i, KNEE_UP_MAX);
        } else {
          setKnee(i, KNEE_NEUTRAL);
        }
      }
      return;
      if (phase < NUM_LEGS) {
        setKnee(phase/2, KNEE_UP);
      } else {
        int p = phase-NUM_LEGS;
        if (p < 0) p+=NUM_LEGS;
        setKnee(p/2, KNEE_NEUTRAL+10);
      }
      break;
  }
}

#if 0

void gait_sidestep(int left, long timeperiod) {

  // the gait consists of 6 phases and uses tripod definitions

#define NUM_SIDESTEP_PHASES 6
#define FBSHIFT    50   // shift front legs back, back legs forward, this much
  
  long t = millis()%timeperiod;
  long phase = (NUM_SIDESTEP_PHASES*t)/timeperiod;
  int side1 = LEFT_LEGS;
  int side2 = RIGHT_LEGS;

  if (left == 0) {
    side1 = RIGHT_LEGS;
    side2 = LEFT_LEGS;
  }

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  switch (phase) {
    case 0:
      // Lift up tripod group 1 while group 2 goes to neutral setting
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_UP, FBSHIFT);
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);
      break;

    case 1:
      // slide over by curling one side under the body while extending the other side
      setLeg(TRIPOD2_LEGS&side1, HIP_NEUTRAL, KNEE_DOWN, FBSHIFT);
      setLeg(TRIPOD2_LEGS&side2, HIP_NEUTRAL, KNEE_RELAX, FBSHIFT);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      // and at the sametime put the curled legs into neutral position
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);
      break;

    case 3:
      // Lift up tripod group 2 while group 2 goes to neutral setting
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_UP, FBSHIFT);
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);  
      break;
      
    case 4:
      // slide over by curling one side under the body while extending the other side
      setLeg(TRIPOD1_LEGS&side1, HIP_NEUTRAL, KNEE_DOWN, FBSHIFT);
      setLeg(TRIPOD1_LEGS&side2, HIP_NEUTRAL, KNEE_RELAX, FBSHIFT);
      break;

    case 5:
      // now put all the legs back down on the ground, then the cycle repeats
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFT);
      break;
  }
}

#endif

unsigned short KneeTarget[NUM_LEGS];
unsigned short HipTargetLeft, HipTargetRight;

#define MOVEINCREMENT 7  // degrees per transmission time delay
#define NUM_PUGIL_PHASES 8
#define HIP_FISTS_FORWARD 130


void fight_mode(char dpad, int mode, long timeperiod) {

 
  if (mode == SUBMODE_3) {
    // in this mode the robot leans forward, left, or right by adjusting hips only

    // this mode retains state and moves slowly, it's for getting somethign like the joust or 
    // capture the flag accessories in position

      switch (dpad) {
      case 's': 
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = ServoPos[i+NUM_LEGS];
        }
        HipTargetRight = ServoPos[0];  // all the hips have the same target so just pick one
        HipTargetLeft = ServoPos[LEFT_START];
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = KNEE_STAND;
        }
        HipTargetLeft = HipTargetRight = 90;
        break;
      case 'f': // swing hips forward, mirrored
        HipTargetRight = 55;
        HipTargetLeft = 180 - HipTargetRight;
        break;
      case 'b': // move the knees back up to standing position, leave hips alone
        HipTargetRight = 125;
        HipTargetLeft = 180 - HipTargetRight;
        break;
      case 'l':
        HipTargetLeft = HipTargetRight = 170;
        break;
      case 'r':
        HipTargetLeft = HipTargetRight = 10;
        break;
    }
    
  } else if (mode == SUBMODE_4) {
    // in this mode the entire robot leans in the direction of the pushbuttons
    // and the weapon button makes the robot return to standing position.

    // Only knees are altered by this, not hips (other than the reset action for
    // the special D-PAD button)

    // this mode does not immediately set servos to final positions, instead it
    // moves them toward targets slowly.
    
    switch (dpad) {
      case 's': 
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = ServoPos[i+NUM_LEGS];
        }
        HipTargetLeft = ServoPos[0];  // all the hips have the same target so just pick one
        HipTargetRight = ServoPos[LEFT_START];
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = KNEE_STAND;
        }
        HipTargetLeft = HipTargetRight = 90;
        break;
      case 'f': // move knees into forward crouch, leave hips alone

        if (ServoPos[8] == KNEE_STAND) { // the back legs are standing, so crouch the front legs
          KneeTarget[0]=KneeTarget[5]=KNEE_CROUCH;
          KneeTarget[1]=KneeTarget[4]=KNEE_HALF_CROUCH;
          KneeTarget[2]=KneeTarget[3]=KNEE_STAND;
        } else { // bring the back legs up first
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;
      case 'b': // move back legs down so robot tips backwards
        if (ServoPos[6] == KNEE_STAND) { // move the back legs down
          KneeTarget[0]=KneeTarget[5]=KNEE_STAND;
          KneeTarget[1]=KneeTarget[4]=KNEE_HALF_CROUCH;
          KneeTarget[2]=KneeTarget[3]=KNEE_CROUCH;
        } else { // front legs are down, return to stand first
            for (int i = 0; i < NUM_LEGS; i++) {
              KneeTarget[i] = KNEE_STAND;
            }
        }
        break;
     case 'l':
        if (ServoPos[9] == KNEE_STAND) {
          KneeTarget[0]=KneeTarget[2] = KNEE_HALF_CROUCH;
          KneeTarget[1]=KNEE_CROUCH;
          KneeTarget[3]=KneeTarget[4]=KneeTarget[5]=KNEE_STAND;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;
      case 'r':
        if (ServoPos[6] == KNEE_STAND) {
          KneeTarget[0]=KneeTarget[1]=KneeTarget[2] = KNEE_STAND;
          KneeTarget[3]=KneeTarget[5]=KNEE_HALF_CROUCH;
          KneeTarget[4]=KNEE_CROUCH;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;

    }
  }

  if (mode == SUBMODE_4 || mode == SUBMODE_3) { // incremental moves

    // move servos toward their targets


    for (int i = 0; i < NUM_LEGS; i++) {
      int h, k;
      h = ServoPos[i];
      k = ServoPos[i+KNEE_OFFSET];
      int diff = KneeTarget[i] - k;
      
      if (diff <= -MOVEINCREMENT) {
        // the knee has a greater value than the target
        k -= MOVEINCREMENT;
      } else if (diff >= MOVEINCREMENT) {
        // the knee has a smaller value than the target
        k += MOVEINCREMENT;
      } else {
        // the knee is within MOVEINCREMENT of the target so just go to target
        k = KneeTarget[i];
      }

      setKnee(i, k);

      if (i < LEFT_START) {
        diff = HipTargetRight - h;
      } else {
        diff = HipTargetLeft - h;
      }
      if (diff <= -MOVEINCREMENT) {
        // the hip has a greater value than the target
        h -= MOVEINCREMENT;
      } else if (diff >= MOVEINCREMENT) {
        // the hip has a smaller value than the target
        h += MOVEINCREMENT;
      } else {
        // the knee is within MOVEINCREMENT of the target so just go to target
        if (i < LEFT_START) {
          h = HipTargetRight;
        } else {
          h = HipTargetLeft;
        }
      }
        
      setHipRaw(i, h);
      //Serial.print("RAW "); Serial.print(i); Serial.print(" "); Serial.println(h);

    }
    return;  // /this mode does not execute the rest of the actions
  }

  // If we get here, we are in either submode A or B
  //
  // submode A: fight with two front legs, individual movement
  // submode B: fight with two front legs, in unison
  
  setLeg(MIDDLE_LEGS, HIP_FORWARD+10, KNEE_STAND, 0);
  setLeg(BACK_LEGS, HIP_BACKWARD, KNEE_STAND, 0);
  
  switch (dpad) {
    case 's':  // stop mode: both legs straight out forward
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_NEUTRAL, 0);

      break;
      
    case 'f':  // both front legs move up in unison
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
      break;
    
    case 'b':  // both front legs move down in unison
      setLeg(FRONT_LEGS, HIP_FORWARD, KNEE_STAND, 0);
      break;
    
    case 'l':  // left front leg moves left, right stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else {
        // both legs move in unison in submode B
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;
    
    case 'r':  // right front leg moves right, left stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else { // submode B
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;
    
    case 'w':  // automatic ninja motion mode with both legs swinging left/right/up/down furiously!

        {  // we need a new scope for this because there are local variables
        
        long t = millis()%timeperiod;
        long phase = (NUM_PUGIL_PHASES*t)/timeperiod;
      
        //Serial.print("PHASE: ");
        //Serial.println(phase);
    
        switch (phase) {
          case 0:
            // Knees down, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_RELAX, 0);
            break;
      
          case 1:
            // Knees up, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
            break;
      
          case 2:
            // Knees neutral, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_NEUTRAL, 0);
            break;
      
          case 3:
            // Knees up, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_UP, 0);
            break;
      
          case 4:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             break;
      
          case 5:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;
      
          case 6:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             break;
      
          case 7:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;
        }
      }
  }

}
#define NUM_TRIPOD_PHASES 6
#define FBSHIFT    15   // 40 shift front legs back, back legs forward, this much
 
void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod) {

  // the gait consists of 6 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the 
  // desired time period that all six  phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

 
  long t = millis()%timeperiod;
  long phase = (NUM_TRIPOD_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0);
      break;
      
    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0);
      break;  
  }
}

int ScamperPhase = 0;
long NextScamperPhaseTime = 0;

#define FBSHIFT    15   // 40 shift front legs back, back legs forward, this much
#define SCAMPERPHASES 6

#define KNEEDELAY 30
#define HIPDELAY 90

void gait_tripod_scamper(int reverse, int turn) {

  // this is a tripod gait that tries to go as fast as possible by not waiting
  // for knee motions to complete before beginning the next hip motion

  // this was experimentally determined and assumes the battery is maintaining
  // +5v to the servos and they are MG90S or equivalent speed. There is very
  // little room left for slower servo motion. If the battery voltage drops below
  // 6.5V then the BEC may not be able to maintain 5.0V to the servos and they may
  // not complete motions fast enough for this to work.

  int hipforward, hipbackward;
  
  if (reverse) {
    hipforward = HIP_BACKWARD;
    hipbackward = HIP_FORWARD;
  } else {
    hipforward = HIP_FORWARD;
    hipbackward = HIP_BACKWARD;
  }



  if (millis() >= NextScamperPhaseTime) {
    ScamperPhase++;
    if (ScamperPhase >= SCAMPERPHASES) {
      ScamperPhase = 0;
    }
    switch (ScamperPhase) {
      case 0: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 1: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 2: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 3: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 4: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 5: NextScamperPhaseTime = millis()+KNEEDELAY; break;
    }

  }

  //Serial.print("ScamperPhase: "); Serial.println(ScamperPhase);

  switch (ScamperPhase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_SCAMPER, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_SCAMPER, 0, turn);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0, turn);
      break;
      
    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;  
  }
}

void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod) {
  // the gait consists of 10 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the 
  // desired time period that all phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_RIPPLE_PHASES 19
  
  long t = millis()%timeperiod;
  long phase = (NUM_RIPPLE_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  if (phase == 18) {
    setLeg(ALL_LEGS, hipbackward, NOMOVE, FBSHIFT);
  } else {
    int leg = phase/3;  // this will be a number between 0 and 2
    leg = 1<<leg;
    int subphase = phase%3;

    switch (subphase) {
      case 0:
        setLeg(leg, NOMOVE, kneeup, 0);
        break;
      case 1:
        setLeg(leg, hipforward, NOMOVE, FBSHIFT);
        break;
      case 2:
        setLeg(leg, NOMOVE, kneedown, 0);
        break;     
    }
  }
}


#define G_STAND 0
#define G_TURN  1
#define G_TRIPOD 2
#define G_SCAMPER 3
#define G_DANCE 4
#define G_BOOGIE 5
#define G_FIGHT 6
#define G_TEETER 7
#define G_BALLET 8

#define G_NUMGATES 9


int curGait = G_STAND;
int curReverse = 0;
long nextGaitTime = 0;

void random_gait(int timingfactor) {

#define GATETIME 3500  // number of milliseconds for each demo

  if (millis() > nextGaitTime) {
    curGait++;
    if (curGait >= G_NUMGATES) {
      curGait = 0;
    }
    nextGaitTime = millis() + GATETIME;

    // when switching demo modes, briefly go into a standing position so 
    // we're starting at the same position every time.
    setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
    delay(600);
  }

  switch (curGait) {
    case G_STAND:
      stand();
      break;
    case G_TURN:
      turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 700
      break;
    case G_TRIPOD:
      gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 900
      break;
    case G_SCAMPER:
      gait_tripod_scamper((nextGaitTime-(millis())<GATETIME/2),0);  // reverse direction halfway through
      break;
    case G_DANCE:
      stand();
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 150);
      delay(350);
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 30);
      delay(350);
      break;
    case G_BOOGIE:
       boogie_woogie(NO_LEGS, SUBMODE_1, 2);
       break;
    case G_FIGHT:
      fight_mode('w', SUBMODE_1, FIGHT_CYCLE_TIME);
      break;

    case G_TEETER:
      wave('r');
      break;

    case G_BALLET:
      flutter();
      break;
      
  }
  
}

void foldup() {
  setLeg(ALL_LEGS, NOMOVE, KNEE_FOLD, 0);
  for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, HIP_FOLD);
}


#define NUM_DAB_PHASES 3
void dance_dab(int timingfactor) {
  long t = millis()%(1100*timingfactor);
  long phase = (NUM_DAB_PHASES*t)/(1100*timingfactor);

  switch (phase) {
    case 0: 
      stand(); break;

    case 1: 
      setKnee(6, KNEE_UP); break;

    case 2: 
      for (int i = 0; i < NUM_LEGS; i++)
         if (i != 0) setHipRaw(i, 40);
      setHipRaw(0, 140);
      break;
  }
}


#define NUM_FLUTTER_PHASES 4
#define FLUTTER_TIME 200
#define KNEE_FLUTTER (KNEE_TIPTOES+20)

void flutter() {   // ballet flutter legs on pointe

  long t = millis()%(FLUTTER_TIME);
  long phase = (NUM_FLUTTER_PHASES*t)/(FLUTTER_TIME);

  setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
  
  switch (phase) {
    case 0:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 1:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 2:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 3:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
  }

}


#define BALLET_TIME 250

void dance_ballet(int dpad) {   // ballet flutter legs on pointe
  switch (dpad) {

    default:
    case 's': tiptoes(); return;

    case 'w': flutter(); return;

    case 'l':
      turn(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'r':
      turn(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME); 
      break;

    case 'f':
      gait_tripod(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'b':
      gait_tripod(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;
  }
}

#define NUM_HANDS_PHASES 2
#define HANDS_TIME_PERIOD 400
void dance_hands(int dpad) {

  setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0); 
  setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0); 

  switch (dpad) {
    case 's':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_UP, 0, 0);
      break;
    case 'f':
      setLeg(MIDDLE_LEGS, HIP_FORWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'b':
      setLeg(MIDDLE_LEGS, HIP_BACKWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'l':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
      break;
    case 'r':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
      break;
    case 'w':
      // AUTOMATIC MODE

        {  // we need a new scope for this because there are local variables
        
        long t = millis()%HANDS_TIME_PERIOD;
        long phase = (NUM_HANDS_PHASES*t)/HANDS_TIME_PERIOD;
     
    
        switch (phase) {
          case 0:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
            break;
      
          case 1:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
            break;

        }
      }
      break;
  }
}


#define NUM_DANCE_PHASES 2
void dance(int legs_up, int submode, int timingfactor) {
   setLeg(legs_up, NOMOVE, KNEE_UP, 0, 0);
   setLeg((legs_up^0b111111), NOMOVE, ((submode==SUBMODE_1)?KNEE_STAND:KNEE_TIPTOES), 0, 0);

  
  long t = millis()%(600*timingfactor);
  long phase = (NUM_DANCE_PHASES*t)/(600*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 140);
      break;
    case 1:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 40);
      break;
  }
}

#define NUM_BOOGIE_PHASES 2
void boogie_woogie(int legs_flat, int submode, int timingfactor) {
  
      setLeg(ALL_LEGS, NOMOVE, KNEE_UP, 0);
      //setLeg(legs_flat, NOMOVE, KNEE_RELAX, 0, 0);

  
  long t = millis()%(400*timingfactor);
  long phase = (NUM_BOOGIE_PHASES*t)/(400*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 150);
      break;
      
    case 1: 
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 30);
      break;
  }
}


int ServosDetached = 0;

void attach_all_servos() {
  Serial.print("A");
  for (int i = 0; i < 12; i++) {
    setServo(i, ServoPos[i]);
  }
  ServosDetached = 0;
  return;
}
void detach_all_servos() {
  Serial.print("D");
  for (int i = 0; i < 16; i++) {
    pwm.setPin(i,0,false); // stop pulses which will quickly detach the servo
  }
  ServosDetached = 1;
}

void initMove() {

  pinMode(BeeperPin, OUTPUT);
  beep(200);
  /*pinMode(13, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);*/
  
  //digitalWrite(13, LOW);
  
  // A1 and A2 provide power to the potentiometer
  //digitalWrite(A1, HIGH);
  //digitalWrite(A2, LOW);

 
  /*BlueTooth.begin(38400);

  BlueTooth.println("");
  delay(250);
  BlueTooth.println("Vorpal H12 starting!");*/

  delay(250);

  pwm.begin();  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(250);
  
  //stand();
  laydown();
  delay(600);
  beep(400);

  //CmuCam5.init();

  yield();
}



void setServo(int servonum, int position) {
  int p = map(position,0,180,SERVOMIN,SERVOMAX);
  pwm.setPWM(servonum, 0, p);
  ServoPos[servonum] = position;  // keep data on where the servo was last commanded to go
  // DEBUG: Uncomment the next line to debug setservo problems. It causes some lagginess due to all the printing
  //Serial.print("SS:");Serial.print(servonum);Serial.print(":");Serial.println(position);
  if (0) {
    ServosDetached = 0;
    Serial.print("D");
    for (int i = 0; i < 12; i++) {
      if (i != servonum && ServoPos[i]>=0 && ServoPos[i]<=180) {
        pwm.setPWM(i, 0, map(ServoPos[i],0,180,SERVOMIN,SERVOMAX));
      }
    }
  }
}




int pulselen = SERVOMIN;


int lastCmd = 's';
int priorCmd = 0;
int mode = MODE_WALK; // default
int submode = SUBMODE_1;     // standard submode.
int timingfactor = 1;   // default is full speed. If this is greater than 1 it multiplies the cycle time making the robot slower


void doAction(char mode1,char submode1,char lastCmd1){
  mode=mode1;
  submode=submode1;
  lastCmd=lastCmd1;
}





int servoNo = 0;
void loopMode(int selector) {
  if(ServosDetached && selector!=OFF_MODE){
    attach_all_servos();
  }else if(!ServosDetached && selector==OFF_MODE){
    detach_all_servos();
  }
  int factor = 1;
switch (selector){
  case OFF_MODE:
  break;
   //Serial.print("Analog0="); Serial.println(p);
  case STAND_MODE:
    static long ReportTime = 0;
    stand();
    // in Stand mode we will also dump out all sensor values once per second to aid in debugging hardware issues
    if (millis() > ReportTime) {
          ReportTime = millis() + 1000;
          Serial.println("Stand, Sensors:");
          Serial.print(" A3="); Serial.print(analogRead(A3));
          Serial.print(" A6="); Serial.print(analogRead(A6));
          Serial.print(" A7="); Serial.print(analogRead(A7));
          //Serial.print(" Dist="); Serial.print(readUltrasonic());
          Serial.println("");
    }

 case ADJUST_MODE:
    stand_90_degrees();
    Serial.println("AdjustMode");
    break;
    case TEST_MODE:
    // test each motor one by one mode
    
      setServo(servoNo, 140);
      delay(500);
    
      setServo(servoNo, 40);
      delay(500);
      setServo(servoNo, 90);
      delay(100);
      Serial.print("SERVO: "); Serial.println(servoNo);
    
  
  servoNo=(servoNo+1)%12;
  break;
  case DEMO_MODE:// demo mode
  
    random_gait(timingfactor);
    Serial.println("Rand");
    return;

 break;
 case COMMAND_MODE:// bluetooth mode

    int gotnewdata = 0;//receiveDataHandler();  // handle any new incoming data first
    //Serial.print(gotnewdata); Serial.print(" ");

      // if its been more than 1 second since we got a valid bluetooth command
      // then for safety just stand still.

      if (millis() > LastValidReceiveTime + 1000) {
        if (millis() > LastValidReceiveTime + 50000) {
          // this is just test code for now to study the loss of connection problem
          // after 5 full seconds of not receiving a valid command, reset the bluetooth connection
          Serial.println("Loss of Signal: resetting bluetooth");
          beep(200,40); // loss of connection test
          delay(100);
          beep(400, 40);
          delay(100);
          beep(600, 40);
          //BlueTooth.begin(38400);
          LastReceiveTime = LastValidReceiveTime = millis();
          lastCmd = -1;  // for safety put it in stop mode
        }
        long losstime = millis() - LastValidReceiveTime;
        Serial.print("LOS"); Serial.println(losstime);
        return;
      }

    if (gotnewdata == 0) {
      // we didn't receive any new instructions so repeat the last command unless it was binary
      // or unless we're in fight adjust mode
      if (lastCmd == -1) {
        //Serial.print("-");
        return;
      }


      // fight submodes C and E should not be repeated without receiving
      // a packet because otherwise they'll zoom right to the end state instead
      // of giving the user a chance to make fine adjustments to position
      if (mode == MODE_FIGHT && (submode == SUBMODE_3 || submode == SUBMODE_4)) {
        //Serial.print("f");
        return;
      }

    } else {
      LastReceiveTime = millis();
    }
    // Leg set mode should also not be repeated
    if (mode == MODE_LEG) {
      //Serial.print("l");
      return;
    }
    //
    // Now we're either repeating the last command, or reading the new bluetooth command
    //
    
    switch(lastCmd) {
      case '?': //BlueTooth.println("Vorpal H12"); 
        break;
      case 'W': 
        mode = MODE_WALK; 
        break;
      case 'F': 
        mode = MODE_FIGHT; startedStanding = -1;
        break;
      case 'D': 
        mode = MODE_DANCE; startedStanding = -1;
        break;
      case '1': 
      case '2': 
      case '3': 
      case '4': 
        submode = lastCmd;
        break;
      case 'w':  // weapon mode, special depending on mode
        startedStanding = -1;
        switch (mode) {
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, 660*timingfactor);
            break;
          case MODE_DANCE:
            if (submode == SUBMODE_1) {
              dance_dab(timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_WALK: {
              beep(400);
              // stomp in place while beeping horn
              if (submode == SUBMODE_2) { // high step
                factor = 2;
              }
              int cyc = TRIPOD_CYCLE_TIME*factor;
              if (submode == SUBMODE_4) {
                cyc = TRIPOD_CYCLE_TIME/2;  // faster stomp in scamper mode
              }
              gait_tripod(1, 90, 90, 
                      KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, 
                      cyc);
            }  
            break;
          default:     // for any other mode implement a "horn"
            beep(400);
            break;
        }
        break;
        
      case 'f':  // forward
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
              if (submode == SUBMODE_4) {
                gait_tripod_scamper(0,0);
              } else {
                if (submode == SUBMODE_2) { // high step
                  factor = 2;
                }
                gait_tripod(1, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, 
                  TRIPOD_CYCLE_TIME*factor);
              }
              break;
          case MODE_DANCE:
              if (submode == SUBMODE_1) {
                dance(NO_LEGS, submode, timingfactor);
              } else if (submode == SUBMODE_2) {
                dance_ballet(lastCmd);
              } else if (submode == SUBMODE_3) {
                wave(lastCmd);
              } else if (submode == SUBMODE_4) {
                dance_hands(lastCmd);
              }
              break;
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'b':  // backward
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
          if (submode == SUBMODE_4) {
              gait_tripod_scamper(1,0);
          } else {
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            gait_tripod(0, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
          }
          break;
          case MODE_DANCE:
              if (submode == SUBMODE_1) {
                boogie_woogie(NO_LEGS, submode, timingfactor);
              } else if (submode == SUBMODE_2) {
                dance_ballet(lastCmd);
              } else if (submode == SUBMODE_3) {
                wave(lastCmd);   
              } else if (submode == SUBMODE_4) {
                dance_hands(lastCmd);
              }         
              break;
          case MODE_FIGHT:
              fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'l': // left
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            if (submode == SUBMODE_4) {
              gait_tripod_scamper(1,1);
            } else {
              turn(0, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
            }
            break;
          case MODE_DANCE:      
            if (submode == SUBMODE_1) {
              dance(TRIPOD1_LEGS, submode, timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'r':  // right
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            if (submode == SUBMODE_4) {
              gait_tripod_scamper(0,1);
            } else {
              turn(1, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, 
                  KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
            }
            break;
          case MODE_DANCE:
            if (submode == SUBMODE_1) {
              dance(TRIPOD2_LEGS, submode, timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_FIGHT:
              fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 's':  // stop and just stand there
        if (startedStanding == -1) {
          startedStanding = millis();
        }
        if (mode == MODE_FIGHT) {
          startedStanding = millis();  // reset in fight mode, never sleep the legs
          fight_mode(lastCmd, submode, 660*timingfactor);
        } else if (mode == MODE_DANCE && submode == SUBMODE_2) { // ballet
          tiptoes();
        } else if (mode == MODE_DANCE && submode == SUBMODE_4) {
          dance_hands(lastCmd);
        } else {
            if (millis() - startedStanding > BATTERYSAVER) {
              //Serial.print("DET LC=");Serial.write(lastCmd); Serial.println("");
              detach_all_servos();
              return;
            }
          stand();
        }


        break;

      case 'a': // adjust mode
        stand_90_degrees();
        break;
       
       default:
        Serial.print("BAD CHAR:"); Serial.write(lastCmd); Serial.println("");
        beep(100,20);
    }  // end of switch
    

  }  // end of main if statement
  


}
