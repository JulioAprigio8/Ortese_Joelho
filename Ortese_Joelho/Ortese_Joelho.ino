#include <Wire.h>
//============Bibliotecas BNO055===================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//=================================================
//============Dados configuração===================
#define pin_pot 13 //GPIO13 para o pino do potenciometro
float eixo_x, eixo_y, eixo_z;
int potenciometro, map_pot;
//=================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void checkBNO();
void setup() {
  Serial.begin(9600);
  checkBNO();
}

void loop() {
  potenciometro = analogRead(pin_pot);
  map_pot = map(potenciometro, 0, 4095, 0, 100);
  Serial.print("Valor bruto: ");
  Serial.print(potenciometro);
  Serial.print(" Valor Ajustado: ");
  Serial.print(map_pot);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  eixo_x = euler.x();
  eixo_y = euler.y();
  eixo_z = euler.z();

  Serial.print(" X:");
  Serial.print(eixo_x);
  Serial.print(" Y:");
  Serial.print(eixo_y);
  Serial.print(" Z:");
  Serial.println(eixo_z);
  delay(100);
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
