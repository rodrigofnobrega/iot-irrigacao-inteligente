#include <WiFi.h>
#include <ArduinoHA.h>
#include "DHTesp.h"
#include <ESP32Servo.h>

#define PUBLISH_PERIOD     10000  // em milissegundos
#define DHT_PIN            13

#define LED_RED_PIN        25
#define LED_GREEN_PIN      26
#define LED_BLUE_PIN       27

#define BTN_R_PIN          32
#define BTN_G_PIN          33
#define BTN_B_PIN          34
#define SERVO_PIN          18

#define BROKER_ADDR        IPAddress(10,0,1,207)
#define BROKER_USERNAME    "samuel00711"
#define BROKER_PASSWORD    "@"
#define WIFI_SSID          "Wokwi-GUEST"
#define WIFI_PASSWORD      ""

unsigned long lastTime = 0;

DHTesp dhtSensor;
WiFiClient client;
Servo servo;

HADevice device("SamuelFontes_feeddevice");
HAMqtt mqtt(client, device);

HASwitch led_red("SamuelF_led_red");
HASwitch led_green("SamuelF_led_green");
HASwitch led_blue("SamuelF_led_blue");
HASensor dhtSensorTemp("SamuelF_temperature");
HASensor dhtSensorHumi("SamuelF_humidity");
HASwitch servo_switch("SamuelF_servo");

void onRedSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(LED_RED_PIN, state ? HIGH : LOW);
  sender->setState(state);
}

void onGreenSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(LED_GREEN_PIN, state ? HIGH : LOW);
  sender->setState(state);
}

void onBlueSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(LED_BLUE_PIN, state ? HIGH : LOW);
  sender->setState(state);
}

void onServoSwitchCommand(bool state, HASwitch* sender) {
  servo.write(state ? 90 : 0);  // 90° = aberto, 0° = fechado
  sender->setState(state);
  Serial.print("Servo ");
  Serial.println(state ? "ABERTO " : "FECHADO ");
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  servo.attach(SERVO_PIN);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  pinMode(BTN_R_PIN, INPUT_PULLUP);
  pinMode(BTN_G_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);

  // Configuração do dispositivo
  device.setName("SamuelFontes");
  device.setModel("ESP32-Wokwi");
  device.setSoftwareVersion("1.0.0");
  device.enableSharedAvailability();
  device.setAvailability(true);
  device.enableLastWill();

  // LEDs
  led_red.setName("LED Vermelho");
  led_red.onCommand(onRedSwitchCommand);

  led_green.setName("LED Verde");
  led_green.onCommand(onGreenSwitchCommand);

  led_blue.setName("LED Azul");
  led_blue.onCommand(onBlueSwitchCommand);

  // Sensor de temperatura
  dhtSensorTemp.setName("Temperatura");
  dhtSensorTemp.setDeviceClass("temperature");
  dhtSensorTemp.setUnitOfMeasurement("°C");

  // Sensor de umidade
  dhtSensorHumi.setName("Umidade");
  dhtSensorHumi.setDeviceClass("humidity");
  dhtSensorHumi.setUnitOfMeasurement("%");

  // Servo
  servo_switch.setName("Controle do Servo");
  servo_switch.onCommand(onServoSwitchCommand);

  // Inicialmente, servo fechado
  servo.write(0);
  servo_switch.setState(false);

  mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
}

void loop() {
  mqtt.loop();

  // Leitura de temperatura e umidade
  if (millis() - lastTime > PUBLISH_PERIOD) {
    lastTime = millis();
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    String tempStr = String(data.temperature, 1);  // Uma casa decimal
    String humiStr = String(data.humidity, 1);

    dhtSensorTemp.setValue(tempStr.c_str());
    dhtSensorHumi.setValue(humiStr.c_str());


    Serial.printf("Temp: %.1f°C, Umidade: %.1f%%\n", data.temperature, data.humidity);
  }

  // Leitura dos botões (borda de descida)
  static bool lastR = HIGH, lastG = HIGH, lastB = HIGH;

  bool btnR = digitalRead(BTN_R_PIN);
  bool btnG = digitalRead(BTN_G_PIN);
  bool btnB = digitalRead(BTN_B_PIN);

  if (lastR == HIGH && btnR == LOW) {
    bool newState = !digitalRead(LED_RED_PIN);
    digitalWrite(LED_RED_PIN, newState);
    led_red.setState(newState);
    Serial.println("Botão vermelho: toggle");
  }
  if (lastG == HIGH && btnG == LOW) {
    bool newState = !digitalRead(LED_GREEN_PIN);
    digitalWrite(LED_GREEN_PIN, newState);
    led_green.setState(newState);
    Serial.println("Botão verde: toggle");
  }
  if (lastB == HIGH && btnB == LOW) {
    bool newState = !digitalRead(LED_BLUE_PIN);
    digitalWrite(LED_BLUE_PIN, newState);
    led_blue.setState(newState);
    Serial.println("Botão azul: toggle");
  }

  lastR = btnR;
  lastG = btnG;
  lastB = btnB;
}