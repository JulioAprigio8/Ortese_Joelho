#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//============Bibliotecas BNO055===================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//=================================================
//============Dados configuração===================
#define pin_pot 35 //GPIO13 para o pino do potenciometro
float eixo_x, eixo_y, eixo_z;
int potenciometro, map_pot;
//=================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void checkBNO();
// Replace the next variables with your SSID/Password combination
const char* ssid = "Paulo Aprigio 2.4GHz";
const char* password = "";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.0.149";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Serial.begin(9600);
  checkBNO();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  //  if (String(topic) == "esp32/output") {
  //    Serial.print("Changing output to ");
  //    if(messageTemp == "on"){
  //      Serial.println("on");
  //      digitalWrite(ledPin, HIGH);
  //    }
  //    else if(messageTemp == "off"){
  //      Serial.println("off");
  //      digitalWrite(ledPin, LOW);
  //    }
  //  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  potenciometro = analogRead(pin_pot);
  map_pot = map(potenciometro, 0, 4095, 0, 100);
  client.loop();

  long now = millis();
  if (now - lastMsg > 50) {

    lastMsg = now;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    eixo_x = euler.x();
    eixo_y = euler.y();
    eixo_z = euler.z();

    // Convert the value to a char array
    char xString[8];
    dtostrf(eixo_x, 1, 2, xString);
    Serial.print("Eixo X: ");
    Serial.println(eixo_x);
    client.publish("esp32/eixo_x", xString);
    // Convert the value to a char array
    char yString[8];
    dtostrf(eixo_y, 1, 2, yString);
    Serial.print("Eixo Y: ");
    Serial.println(eixo_y);
    client.publish("esp32/eixo_y", yString);
    // Convert the value to a char array
    char zString[8];
    dtostrf(eixo_z, 1, 2, zString);
    Serial.print("Eixo Z: ");
    Serial.println(eixo_z);
    client.publish("esp32/eixo_z", zString);


    // Convert the value to a char array
    char potString[8];
    Serial.print("Pot: ");
    Serial.println(map_pot);
    dtostrf(map_pot, 1, 2, potString);
    client.publish("esp32/pot", potString);
  }

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
