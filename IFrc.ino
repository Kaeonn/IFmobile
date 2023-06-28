#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid = "Raphael-2G";
const char* password = "Badaro11";
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;

//Pinout
#define PIN_SG90 18  // Output pin servo

#define ENA 5  // Output pin motor PWM

#define INA1 13  // Output pin motor frente
#define INA2 14  // Output pin motor trás

#define ENB 2  // Output pin motor PWM

#define INB1 16  // Output pin motor frente
#define INB2 17  // Output pin motor trás

#define V_GPIO3 22  // Input pin voltímetro

#define LED1_GPIO3 33  //Output pin leds frontais

#define LED3_GPIO4 19  //Output pin led indicador PWM
#define LED3_GPIO5 21  //Output pin led indicador PWM

#define BUZZER_GPIO4 15  //Output pin buzzer

#define LIGHT_SENSOR_PIN 35  // Input pin LDR

#define TEMP_SENSOR_PIN 4  // Input pin D18

unsigned long timer = millis();

//Semáforos e filas
SemaphoreHandle_t semaforoMedicao;
QueueHandle_t filaResultados;

SemaphoreHandle_t semaforoBuzzer;
QueueHandle_t filaComandos;

SemaphoreHandle_t semaforoLDR;

SemaphoreHandle_t semaforoTemp;

SemaphoreHandle_t semaforoMotor;

SemaphoreHandle_t semaforoServo;


Servo sg90;

//Sensor de temperatura
const int oneWireBus = TEMP_SENSOR_PIN;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

WiFiClient wifiClient;
PubSubClient mqttClient;

//variáveis globais
String comando;
int potencia = 0;
char tensao[8];
char temperatura[8];
float V = 0;

void callback(char* topic, byte* payload, unsigned int length) {

  String msg;

  for (int i = 0; i < length; i++) {

    msg = msg + (char)payload[i];
  }
  if (String(topic) == "/swa/comando") {
    comando = msg;
  } else if (String(topic) == "/swa/potencia") {
    potencia = msg.toInt();
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);

  pinMode(INA1, OUTPUT);

  pinMode(INA2, OUTPUT);

  pinMode(ENB, OUTPUT);

  pinMode(INB1, OUTPUT);

  pinMode(INB2, OUTPUT);

  pinMode(LED1_GPIO3, OUTPUT);

  pinMode(LED3_GPIO4, OUTPUT);

  pinMode(LED3_GPIO5, OUTPUT);

  pinMode(BUZZER_GPIO4, OUTPUT);
  pinMode(V_GPIO3, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  WiFi.begin(ssid, password);

  //Acender um botão caso não tiver conexão ao invés de ficar imprimindo pontinhos com delay?
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }

  //Acender um botão caso tiver conexão
  Serial.println("\nWiFi connected");

  //Não sei se tem necessidade
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setClient(wifiClient);

  mqttClient.setServer(mqttServer, mqttPort);

  mqttClient.setCallback(callback);

  mqttClient.subscribe("/swa/comando");
  mqttClient.subscribe("/swa/potencia");

  sg90.setPeriodHertz(50);           // PWM frequency for SG90
  sg90.attach(PIN_SG90, 500, 2400);  // Minimum and maximum pulse width (in µs) to go from 0° to 180

  // Criação do semáforos e da filas
  semaforoMedicao = xSemaphoreCreateMutex();
  filaResultados = xQueueCreate(10, sizeof(float));

  semaforoBuzzer = xSemaphoreCreateMutex();
  filaComandos = xQueueCreate(10, sizeof(char));

  semaforoLDR = xSemaphoreCreateMutex();

  semaforoTemp = xSemaphoreCreateMutex();

  semaforoMotor = xSemaphoreCreateMutex();

  semaforoServo = xSemaphoreCreateMutex();

  // Criação da tarefa
  xTaskCreate(tarefaVoltage, "Tarefa Voltage", 1024, NULL, 3, NULL);
  xTaskCreate(tarefaBuzzer, "Tarefa Buzzer", 1024, NULL, 1, NULL);
  xTaskCreate(tarefaLeds, "Tarefa LEDs", 1024, NULL, 2, NULL);
  xTaskCreate(tarefaTemp, "Tarefa Temp", 1024, NULL, 4, NULL);
  xTaskCreate(tarefaMotor, "Tarefa Motor", 1024, NULL, 6, NULL);
  xTaskCreate(tarefaServo, "Tarefa Servo", 1024, NULL, 5, NULL);
}

void tarefaServo(void *parameter) {
  int pos = 0;  // Declare a variável "pos" fora do loop while

  while (true) {
    if (comando == "a") {  // Use strcmp() para comparar strings
      // Move o servo para a esquerda
      for (int p = pos; p <= 45; p += 5) {
        sg90.write(p);
        delay(15);
      }
      pos = 180;
    } else if (comando == "d") {  // Use strcmp() para comparar strings
      // Move o servo para a direita
      for (int p = pos; p >= 0; p -= 5) {
        sg90.write(p);
        delay(15);
      }
      pos = 0;
    } else if (comando == "c") {  // Use strcmp() para comparar strings
      // Centraliza o servo
      sg90.write(0);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void tarefaBuzzer(void* parameter) {
  float sinVal;
  int toneVal;
  while (true) {
    if (V < 6) {
    for (byte t = 0; t < 10; t++) {
      for (byte x = 0; x < 180; x++) {
        //converte graus em radianos
        sinVal = (sin(x * (3.14 / 180)));
        //agora gera uma frequencia
        toneVal = 666 + (int(sinVal * 100));
        //toca o valor no buzzer
        analogWrite(BUZZER_GPIO4, toneVal);
        //atraso de 2ms e gera novo tom
        delay(2);
      }
    }
  }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void tarefaTemp(void* parameter) {
  while (true) {
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    //  Serial.println("temp");

    if (temperatureC <= 70) {
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, LOW);
    }

    dtostrf(temperatureC, 1, 2, temperatura);
    mqttClient.publish("temperatura", temperatura);
    vTaskDelay(pdMS_TO_TICKS(5000));  // Atraso entre as medições (5000ms = 5 segundo)
  }
}

void tarefaLeds(void* parameter) {
  int LDR;
  while (true) {
    LDR = analogRead(LIGHT_SENSOR_PIN);
    if (LDR < 2000) {
      digitalWrite(LED1_GPIO3, HIGH);  // turn on LED
    } else {
      digitalWrite(LED1_GPIO3, LOW);  // turn off LED
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void tarefaVoltage(void* parameter) {
  while (true) {
    float V_cc[50];
    int ResV = 0;
    float razaoResistor = 0;
    float Vmedido = 0;
    float R1 = 15000.0;
    float R2 = 5000.0;
    float Vref = 3.543;
    //Serial.println("voltage");

    razaoResistor = (R2 / (R1 + R2));

    ResV = analogRead(V_GPIO3);
    Vmedido = ((ResV * Vref) / 1024);

    for (int i = 0; i < 50; i++) {
      V_cc[i] = Vmedido / razaoResistor;
    }

    for (int i = 0; i < 50; i++) {
      V += V_cc[i];
    }

    V /= 50;

    // Envio do resultado para a fila
    xQueueSend(filaResultados, &V, portMAX_DELAY);

    dtostrf(V, 1, 2, tensao);
    mqttClient.publish("tensao", tensao);

    vTaskDelay(pdMS_TO_TICKS(1000));  // Atraso entre as medições (1000ms = 1 segundo)
  }
}

void tarefaMotor(void *parameter) {
  while (true) {

    if (potencia > 0 && comando == "w") {
      analogWrite(ENA, potencia);
      digitalWrite(INA1, HIGH);
      digitalWrite(INA2, LOW);

      analogWrite(ENB, potencia);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, HIGH);
    } else if (potencia > 0 && comando == "s") {
      analogWrite(ENA, potencia);
      digitalWrite(INA1, LOW);
      digitalWrite(INA2, HIGH);

      analogWrite(ENB, potencia);
      digitalWrite(INB1, HIGH);
      digitalWrite(INB2, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
 
void loop() {

}