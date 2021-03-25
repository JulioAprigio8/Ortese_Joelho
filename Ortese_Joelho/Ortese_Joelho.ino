#include <WiFi.h>
#include<PubSubClient.h>
#include <Wire.h>
//============Bibliotecas BNO055===================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//=================================================
//============Dados configuração===================
#define WIFISSID "Paulo Aprigio 2.4GHz"
#define PASSWORD "Voepaulo08"
#define TOKEN "BBFF-Xsta1AUJkVTqMOBkD6xTI8uAQuH1pj"
#define MQTT_CLIENT_NAME "ortesejoelho"
//=================================================
//============Variáveis===========================
#define VARIABLE_LABEL_POT "potenciometro"
#define VARIABLE_LABEL_X "eixo-x"
#define VARIABLE_LABEL_Y "eixo-y"
#define VARIABLE_LABEL_Z "eixo-z"
#define DEVICE_LABEL "demo"

#define pin_pot 13 //GPIO13 para o pino do potenciometro

float eixo_x, eixo_y, eixo_z;
int potenciometro, map_pot;

//=================================================
//=============MQTT Broker=========================
char mqttBroker[] = "things.ubidots.com";
char payload[100];
char topic[150];
char topicSubscribe[100];
char str_sensor[10];

WiFiClient ubidots;
PubSubClient client(ubidots);

void reconnect() {
  while (!client.connected()) {
    Serial.println("Tentando conectar MQTT...");
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Conectado");
      client.subscribe(topicSubscribe);
    } else {
      Serial.print("Failed,rc=");
      Serial.print(client.state());
      Serial.println("Tentando conectar novamente em 2 segundos");
      delay(2000);
    }
  }
}

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
void checkBNO();


void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFISSID, PASSWORD);
  Serial.println();
  Serial.print("Wait for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);


//  sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE);

  client.subscribe(topicSubscribe);
  pinMode(pin_pot, INPUT);
  //checkBNO();
  delay(1000);
}

void loop() {
  if (!client.connected()) {
    client.subscribe(topicSubscribe);
    reconnect();
  }
  sprintf(topic, "%s%s", "/v1.6/devices/potenciometro", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL_POT);

  potenciometro = analogRead(pin_pot);
  map_pot = map(potenciometro, 0, 4096, 0, 100);
  Serial.print("Valor bruto: ");
  Serial.print(potenciometro);
  Serial.print(" Valor Ajustado: ");
  Serial.print(map_pot);

  dtostrf(map_pot, 4, 2, str_sensor);

  sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor);
  Serial.println("Publishing data to Ubidots Cloud");
  client.loop();
  delay(1000);

  //  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  //  potenciometro = analogRead(pin_pot);
  //
  //  eixo_x = euler.x();
  //  eixo_y = euler.y();
  //  eixo_z = euler.z();

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
