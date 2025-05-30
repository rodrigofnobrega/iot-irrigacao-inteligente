#include <Arduino.h>
#include <WiFi.h>
#include "DHTesp.h"
#include <ESP32Servo.h>

#define PUBLISH_PERIOD    10000  // em msec
#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASSWORD   ""

#define BROKER_ADDR     IPAddress(10, 0, 1, 207)
#define BROKER_USERNAME ""
#define BROKER_PASSWORD ""

#define DHT_PIN         13
#define SERVO_PIN       18
#define LED_RED_PIN       25
#define LED_GREEN_PIN       26
#define LED_BLUE_PIN       27

DHTesp dht;
WiFiClient client;
HADevice device("SamuelFontes_Device");
HAMqtt mqtt(client, device);

HASensor tempSensor("temperature");
HASensor humiSensor("humidity");

HASwitch ledRed("led_red");
HASwitch ledGreen("led_green");
HASwitch ledBlue("led_blue");
HAServo haServo("servo");

Servo servo;

unsigned long lastTime = 0;

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}