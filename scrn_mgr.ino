/*
    Touch screen power manager based on proximity sensor.

    When the proximity sensor detects someone near to the screen,
    it sends message through MQTT service.

    In response, it may receive a command to turn power of the screen on/off.

    arduino-cli compile --fqbn  esp8266:esp8266:nodemcuv2 scrn_mgr
    arduino-cli upload -p /dev/ttyUSB0 --fqbn  esp8266:esp8266:nodemcuv2 scrn_mgr
*/

#include "credentials.h"
#include <string>

#include "Adafruit_VL53L0X.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

using namespace std;

#define SCREEN_ON_PIN D5
#define LED_GREEN_PIN D3
#define LED_RED_PIN D6
#define SECOND 1000 // 1000 milliseconds

#define DEVICE_NAME "esp_screen"
#define DEVICE_ID DEVICE_NAME "_0001"

#define DISTANCE "distance_threshold"
#define DISTANCE_DISCOVERY "homeassistant/number/" DEVICE_NAME "/" DISTANCE "/config"

#define ON_TIME "on_time"
#define ON_TIME_DISCOVERY "homeassistant/number/" DEVICE_NAME "/" ON_TIME "/config"

#define OFF_TIME "off_time"
#define OFF_TIME_DISCOVERY "homeassistant/number/" DEVICE_NAME "/" OFF_TIME "/config"

#define PROXIMITY "proximity"
#define PROXIMITY_DISCOVERY "homeassistant/binary_sensor/" DEVICE_NAME "/motion/config"
#define PROXIMITY_STATE_TOPIC DEVICE_ID "/" PROXIMITY "/state"
#define PROXIMITY_DISCOVERY_PAYLOAD "{\"name\": \"Proximity sensor\",\"state_topic\": \"" PROXIMITY_STATE_TOPIC "\",\"payload_on\": \"on\",\"payload_off\": \"off\",\"unique_id\": \"" DEVICE_ID "\",\"device\": {\"identifiers\": [\"esp8266_sensor1\"],\"name\": \"" DEVICE_NAME"_screen_controller\",\"model\": \"Custom Screen Controller\",\"manufacturer\": \"Michal Panczyk\"}}"

#define LED_TOGGLE "led_toggle"
#define LED_TOGGLE_DISCOVERY "homeassistant/switch/" DEVICE_NAME "/" LED_TOGGLE "/config"

#define SCREEN_TOGGLE "screen_toggle"
#define SCREEN_TOGGLE_DISCOVERY "homeassistant/switch/" DEVICE_NAME "/" SCREEN_TOGGLE "/config"
#define SCREEN_TOGGLE_COMMAND_TOPIC DEVICE_ID "/screen_switch/set"
#define SCREEN_TOGGLE_STATE_TOPIC DEVICE_ID "/screen_switch/state"
#define SCREEN_TOGGLE_DISCOVERY_PAYLOAD "{\"name\": \"Screen State\",\"command_topic\": \"" SCREEN_TOGGLE_COMMAND_TOPIC "\",\"state_topic\": \"" SCREEN_TOGGLE_STATE_TOPIC "\",\"payload_on\": \"on\",\"payload_off\": \"off\",\"unique_id\": \"" DEVICE_ID "\",\"device\": {\"identifiers\": [\"esp8266_sensor1\"],\"name\": \"" DEVICE_NAME"_screen_controller\",\"model\": \"Custom Screen Controller\",\"manufacturer\": \"Michal Panczyk\"}}"

#define MQTT_TOPIC_SCREEN_TOGGLE "screen/set/led"
#define MQTT_TOPIC_LED_TOGGLE "screen/set/display"
#define MQTT_TOPIC_DISTANCE "screen/set/distance"
#define MQTT_TOPIC_ON_TIME "screen/set/on_time"
#define MQTT_TOPIC_OFF_TIME "screen/set/off_time"

#define MQTT_TOPIC_PROXIMITY_STATE "screen/get/proximity"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned long proximity_reported = 0;
unsigned long proximity_started = 0;
unsigned int INF_DISTANCE = INT_MAX;
bool screen_on = true;
bool proximity = false;

unsigned int distance_threshold = 800; // mm
unsigned int on_time_threshold = SECOND;
unsigned int off_time_threshold = 10*SECOND;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  mqttClient.publish(SCREEN_TOGGLE_DISCOVERY, 1, true, SCREEN_TOGGLE_DISCOVERY_PAYLOAD);
  mqttClient.publish(PROXIMITY_DISCOVERY, 1, true, PROXIMITY_DISCOVERY_PAYLOAD);
  mqttClient.subscribe(SCREEN_TOGGLE_COMMAND_TOPIC, 1);
  //mqttClient.subscribe(MQTT_TOPIC_LED_TOGGLE, 1);
  //mqttClient.subscribe(MQTT_TOPIC_DISTANCE, 1);
  //mqttClient.subscribe(MQTT_TOPIC_ON_TIME, 1);
  //mqttClient.subscribe(MQTT_TOPIC_OFF_TIME, 1);
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void toggle_screen(bool new_state){
    if(new_state != screen_on){
        screen_on = new_state;
        digitalWrite(SCREEN_ON_PIN, screen_on ? HIGH : LOW);
        mqttClient.publish(SCREEN_TOGGLE_STATE_TOPIC, 1, true, screen_on?"on":"off");
        Serial.println(screen_on ? "CLOSE" : "FAR");
    }
}

void onMqttMessage(
    char* topic,
    char* payload,
    AsyncMqttClientMessageProperties properties,
    size_t len,
    size_t index,
    size_t total
) {
    Serial.printf("message - topic: %s, payload: %s\n", topic, payload);
    string t = topic;
    string p = payload;
    if(t == SCREEN_TOGGLE_COMMAND_TOPIC){
        bool new_state = p[1] == 'n';
        Serial.printf("new state: %s\n", new_state?"on":"off");
        toggle_screen(new_state);
    }
    return;
    if(!strcmp(topic, MQTT_TOPIC_SCREEN_TOGGLE)){
        toggle_screen(strcmp(payload, "on") == 0);
    } else if(!strcmp(topic, MQTT_TOPIC_DISTANCE)){
        sscanf(payload, "%d", &distance_threshold);
    } else if(!strcmp(topic, MQTT_TOPIC_ON_TIME)){
        sscanf(payload, "%d", &on_time_threshold);
    } else if(!strcmp(topic, MQTT_TOPIC_OFF_TIME)){
        sscanf(payload, "%d", &off_time_threshold);
    }
}

void setup() {
    Serial.begin(115200);
    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    //mqttClient.onSubscribe(onMqttSubscribe);
    //mqttClient.onUnsubscribe(onMqttUnsubscribe);
    //mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
    connectToWifi();

    pinMode(SCREEN_ON_PIN, OUTPUT);
    digitalWrite(SCREEN_ON_PIN, LOW);

    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);

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

void report_proximity(bool new_state){
    if(new_state != proximity){
        proximity = new_state;
        mqttClient.publish(PROXIMITY_STATE_TOPIC, 1, true, proximity?"on":"off");
        Serial.println(proximity ? "proximity: on" : "proximity: off");
    }
}

void loop() {
    unsigned int distance = get_distance();
    unsigned long current_millis = millis();
    if(distance < distance_threshold){
        // Wait for `on_time_threshold` seconds before reporting proximity.
        if (current_millis > proximity_started + on_time_threshold){
            proximity_reported = current_millis;
            report_proximity(true);
        }
    } else {
        // `proximity_started` records the last time an object was far away.
        // When this no longer holds, it means an object starts to be close.
        proximity_started = current_millis;

        // If there was no proximity for `off_time_threshold` seconds since `proximity_reported`,
        // we report the end of proximity.
        if(current_millis > proximity_reported + off_time_threshold){
            report_proximity(false);
        }
    }
}
