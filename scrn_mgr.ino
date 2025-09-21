#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include "Adafruit_VL53L0X.h"



#define MQTT_PUB_DISTANCE "sensor/screen/distance"
#define MQTT_PUB_TOUCH "sensor/screen/touch"

#define LED_PIN D5

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned int distance_threshold;
unsigned int near_counter; // count how many times an object was near enough
unsigned long near_applied;

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("HELLO. setup()...");
    pinMode(LED_PIN, OUTPUT);

    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    digitalWrite(LED_PIN, LOW);
    distance_threshold = 300;
    near_counter = 0;
    near_applied = 0;
}

void loop() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        if(measure.RangeMilliMeter < distance_threshold)
            ++near_counter;
    }
    if (near_counter > 40){
        near_counter = 0;
        near_applied = millis();
        digitalWrite(LED_PIN, HIGH);
    } else {
        if(millis() > near_applied + (1000*10)){
            digitalWrite(LED_PIN, LOW);
        }
    }
    Serial.print(".");
}
