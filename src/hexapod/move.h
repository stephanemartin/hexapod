
#ifndef MOVE_H
#define MOVE_H

#define STAND_MODE 1
#define ADJUST_MODE 2
#define DEMO_MODE 3
#define COMMAND_MODE 4
#define TEST_MODE 5

#define MODE_WALK 'W'
#define MODE_DANCE 'D'
#define MODE_FIGHT 'F'
#define MODE_RECORD 'R'
#define MODE_LEG   'L'

#define SUBMODE_1 '1'
#define SUBMODE_2 '2'
#define SUBMODE_3 '3'
#define SUBMODE_4 '4'

#define FORWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'

void loopMode(int selector);
void doAction(char mode1,char submode1,char lastCmd1);
#endif
