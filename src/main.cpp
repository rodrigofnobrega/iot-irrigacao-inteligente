#include <WiFi.h>
#include <ArduinoHA.h>
#include "DHTesp.h"
#include <ESP32Servo.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define PUBLISH_PERIOD     5000  // em milissegundos
#define DHT_PIN            13

#define LED_RED_PIN        25
#define LED_GREEN_PIN      26
#define LED_BLUE_PIN       27

#define BTN_R_PIN          32
#define BTN_G_PIN          33
#define BTN_B_PIN          34
#define SERVO_PIN          18

#define BROKER_ADDR        IPAddress(192,168,1,184)
#define BROKER_USERNAME   "aluno"
#define BROKER_PASSWORD   "4luno#imd"
#define WIFI_SSID          "Wokwi-GUEST"
#define WIFI_PASSWORD      ""

// Configurações para se conectar ao Adafruit
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "rodrigofnobrega"

unsigned long lastTime = 0;

WiFiClient client;
DHTesp dhtSensor;
Servo servo;

HADevice device("irrigacao_solo_device");
HAMqtt mqtt(client, device);

HASwitch led_red("irrigacao_led_red");
HASwitch led_green("irrigacao_led_green");
HASwitch led_yellow("irrigacao_led_yellow");
HASensor dhtSensorHumi("irrigacao_humidity");
HASwitch servo_switch("irrigacao_servo");

WiFiClient clientAdafruit; // Wifi para o adafruit para evitar conflito com o Wifi do HA

// Conectar ao adafruit e mapear rotas MQTT
Adafruit_MQTT_Client mqttAdafruit(&clientAdafruit, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, );
Adafruit_MQTT_Publish mqttUmidadePublish = Adafruit_MQTT_Publish(&mqttAdafruit, AIO_USERNAME "/feeds/irrigacao_umidade");
Adafruit_MQTT_Publish mqttServoPublish = Adafruit_MQTT_Publish(&mqttAdafruit, AIO_USERNAME "/feeds/irrigacao_servo");
Adafruit_MQTT_Publish mqttIntensidadeBombaPublish = Adafruit_MQTT_Publish(&mqttAdafruit, AIO_USERNAME "/feeds/irrigacao_intensidade_bomba");


// Conectar ao MQTT do Adafruit
void MQTT_connect() {
  int8_t ret;
  if (mqttAdafruit.connected()) return;

  Serial.print("Conectando ao MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqttAdafruit.connect()) != 0) {
    Serial.println(mqttAdafruit.connectErrorString(ret));
    Serial.println("Tentando novamente em 5 segundos...");
    mqttAdafruit.disconnect();
    delay(5000);
    if (--retries == 0) {
      while (1); // Trava aqui se não conectar
    }
  }
  Serial.println("Conectado ao MQTT!");
}

void desligarTodosLeds() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void onRedSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(LED_RED_PIN, state ? HIGH : LOW);
  sender->setState(state);
}

void onGreenSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(LED_GREEN_PIN, state ? HIGH : LOW);
  sender->setState(state);
}
void onYellowSwitchCommand(bool state, HASwitch* sender) {
  if (state) {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_BLUE_PIN, LOW); 
  } else {
    desligarTodosLeds();
  }
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
  device.setName("IrrigacaoSolo");
  device.setModel("ESP32-Wokwi");
  device.setManufacturer("Rodrigo Nobrega e Samuel Fontes");
  device.setSoftwareVersion("1.0.0");
  device.enableSharedAvailability();
  device.setAvailability(true);
  device.enableLastWill();

  // LEDs
  led_red.setName("LED Vermelho");
  led_red.onCommand(onRedSwitchCommand);

  led_green.setName("LED Verde");
  led_green.onCommand(onGreenSwitchCommand);

  led_yellow.setName("LED Amarelo");
  led_yellow.onCommand(onYellowSwitchCommand);

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
  MQTT_connect();
  mqttAdafruit.processPackets(10);
  if (!mqttAdafruit.ping()) mqttAdafruit.disconnect();

  if (millis() - lastTime > PUBLISH_PERIOD) {
    lastTime = millis();
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    float humidity = data.humidity;
    String humiStr = String(humidity, 1);

    dhtSensorHumi.setValue(humiStr.c_str());

    if (humidity < 40) {
      desligarTodosLeds();
      digitalWrite(LED_GREEN_PIN, HIGH);
      led_red.setState(LOW);
      led_green.setState(HIGH);
      led_yellow.setState(LOW);
      servo.write(90);
      servo_switch.setState(HIGH);
      // Enviar dados para o adafruit
      mqttServoPublish.publish("ON"); 
      mqttIntensidadeBombaPublish.publish("Vazão Alta");
      
      Serial.println("Servo ABERTO 90°");
      Serial.println("Led verde ligado");
    } else if (humidity < 60) {
      desligarTodosLeds();
      digitalWrite(LED_RED_PIN, HIGH);
      digitalWrite(LED_GREEN_PIN, HIGH);
      led_red.setState(LOW);
      led_green.setState(LOW);
      led_yellow.setState(HIGH);
      servo.write(45);
      servo_switch.setState(HIGH);
      // Enviar dados para o adafruit
      mqttServoPublish.publish("ON");
      mqttIntensidadeBombaPublish.publish("Vazão Média");

      Serial.println("Servo ABERTO 45°");
      Serial.println("Led amarelo ligado");
    } else {
      desligarTodosLeds();
      digitalWrite(LED_RED_PIN, HIGH);
      led_red.setState(HIGH);
      led_green.setState(LOW);
      led_yellow.setState(LOW);
      servo.write(0);
      servo_switch.setState(LOW);
      // Enviar dados para o adafruit
      mqttServoPublish.publish("OFF");
      mqttIntensidadeBombaPublish.publish("Desligado");

      Serial.println("Servo Fechado");
      Serial.println("Led vermelho ligado");
    }

    // Enviar umidade para o adafruit
    mqttUmidadePublish.publish(humidity);
    Serial.printf("Umidade: %.1f%%\n", data.humidity);
  }
}