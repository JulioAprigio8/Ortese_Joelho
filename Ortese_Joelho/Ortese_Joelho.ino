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

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
void checkBNO();

float eixo_x, eixo_y, eixo_z;
int pot;
void setup() {
  checkBNO();
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  eixo_x = euler.x();
  eixo_y = euler.y();
  eixo_z = euler.z();

}
