
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "move.h"


#define SERVICE_UUID        "d126db57-3b24-4077-bf38-759927bacc54"
#define CHARACTERISTIC_UUID "03cd0bcf-5c00-49eb-9d2a-d7ed2791d762"


const char* modeName[] = {"Off","Stand","Adjust","Demo","Command","Test Mode"};

int metaMod=0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);
        Serial.println();

        if(value[0] == 'M'){
          metaMod=value[1]-'0';
        }else if(value.length()==3){
          metaMod=4;
          doAction(value[0],value[1],value[2]);
        }else{
          Serial.println("Oops");
          metaMod=0;
        }
      }
    }
};


void setup(){
      pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("MyHexapod");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("MS");
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();    
      initMove();
//      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  //    digitalWrite(LED_BUILTIN, HIGH);    //
}
void loop(){
  loopMode(metaMod);
}

