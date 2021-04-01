#include <Wire.h>
//============Bibliotecas BNO055===================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//=================================================
float eixo_x, eixo_y, eixo_z;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void checkBNO();
void setup() {
  Serial.begin(9600);
  checkBNO();
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  eixo_x = euler.x();
  eixo_y = euler.y();
  eixo_z = euler.z();

  Serial.print(eixo_x);
  Serial.print(" ");
  Serial.print(eixo_y);
  Serial.print(" ");
  Serial.println(eixo_z);

}
void checkBNO() {
  Serial.print("Detectando sensor BNO055...");
  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("[ERRO] verificar fiacao ou o integridade do modulo BNO055");
    delay(2000);
  }
  Serial.println("BNO055 DETECTADO");
  bno.setExtCrystalUse(true);
}
