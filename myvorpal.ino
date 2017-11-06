
#include <WiFi.h>

const char* ssid     = "yourssid";
const char* password = "yourpasswd";

WiFiServer server(80);


void setup(){
      pinMode(LED_BUILTIN, OUTPUT);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
      delay(50);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    //
      delay(500);
      Serial.print(".");
    }
       digitalWrite(LED_BUILTIN, LOW);
}

