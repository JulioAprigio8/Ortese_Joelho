#include <WiFi.h>
#include<PubSubClient.h>
#include <Wire.h>
//============Bibliotecas BNO055===================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//=================================================

#define WIFISSID "Paulo Aprigio 2.4GHz"
#define PASSWORD "Voepaulo08"
#define TOKEN ""
#define MQTT_CLIENT_NAME ""

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
void checkBNO();

float eixo_x, eixo_y, eixo_z;
int pot;
void setup() {
  checkBNO();
  delay(1000);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  eixo_x = euler.x();
  eixo_y = euler.y();
  eixo_z = euler.z();

}
void checkBNO() {
  Serial.print("Detectando sensor BNO055...");
  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("[ERRO] verificar fiacao ou o integridade do modulo BNO055");
    delay(2000);
  }
  Serial.println("BNO055 DETECTADO");
  bno.setExtCrystalUse(true);
}
