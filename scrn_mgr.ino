#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include "Adafruit_VL53L0X.h"



#define MQTT_PUB_DISTANCE "sensor/screen/distance"
#define MQTT_PUB_TOUCH "sensor/screen/touch"

#define LED_PIN D5
#define SECOND 1000

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned int distance_threshold;
unsigned int near_counter; // count how many times an object was near enough
unsigned long near_applied;
unsigned long last_up;
unsigned int INF_DISTANCE = INT_MAX;
bool state_on = false;

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("HELLO. setup()...");
    pinMode(LED_PIN, OUTPUT);

    if (!lox.begin()) {
        Serial.println("Failed to boot VL53L0X");
        while(1);
    }
    digitalWrite(LED_PIN, LOW);
    distance_threshold = 300;
    near_counter = 0;
    near_applied = 0;
}

unsigned int get_distance(){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) // phase failures have incorrect data
        return measure.RangeMilliMeter;
    else
        return INF_DISTANCE;
}

void change(bool new_state){
    if(new_state != state_on){
        state_on = new_state;
        digitalWrite(LED_PIN, state_on?HIGH:LOW);
        Serial.println(state_on? "CLOSE" : "FAR");
    }
}

void loop() {
    unsigned int distance = get_distance();
    unsigned long current_millis = millis();
    if(distance < distance_threshold){
        if (current_millis > last_up + SECOND){ // stay for one second to turn on
            near_applied = current_millis;
            change(true);
        }
    } else {
        last_up = current_millis;
        if(current_millis > near_applied + 10*SECOND){
            change(false);
        }
    }
}
