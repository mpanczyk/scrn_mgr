/*
    Touch screen power manager based on proximity sensor.

    When the proximity sensor detects someone near to the screen,
    it sends message through MQTT service.

    In response, it may receive a command to turn power of the screen on/off.

    arduino-cli compile --fqbn  esp8266:esp8266:nodemcuv2 scrn_mgr
    arduino-cli upload -p /dev/ttyUSB0 --fqbn  esp8266:esp8266:nodemcuv2 scrn_mgr
*/
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include "Adafruit_VL53L0X.h"

#define LED_PIN D5
#define SECOND 1000 // 1000 milliseconds

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned long proximity_reported = 0;
unsigned long proximity_started = 0;
unsigned int INF_DISTANCE = INT_MAX;
bool state_on = false;

unsigned int distance_threshold = 300; // mm
unsigned int on_time_threshold = SECOND;
unsigned int off_time_threshold = 10*SECOND;

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    if (!lox.begin()) {
        Serial.println("Failed to boot VL53L0X");
        while(1);
    }
}

unsigned int get_distance(){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) // phase failures have incorrect data
        return measure.RangeMilliMeter;
    else
        return INF_DISTANCE;
}

void report_change(bool new_state){
    if(new_state != state_on){
        state_on = new_state;
        digitalWrite(LED_PIN, state_on ? HIGH : LOW);
        Serial.println(state_on ? "CLOSE" : "FAR");
    }
}

void loop() {
    unsigned int distance = get_distance();
    unsigned long current_millis = millis();
    if(distance < distance_threshold){
        // Wait for `on_time_threshold` seconds before reporting proximity.
        if (current_millis > proximity_started + on_time_threshold){
            proximity_reported = current_millis;
            report_change(true);
        }
    } else {
        // `proximity_started` records the last time an object was far away.
        // When this no longer holds, it means an object starts to be close.
        proximity_started = current_millis;

        // If there was no proximity for `off_time_threshold` seconds since `proximity_reported`,
        // we report the end of proximity.
        if(current_millis > proximity_reported + off_time_threshold){
            report_change(false);
        }
    }
}
