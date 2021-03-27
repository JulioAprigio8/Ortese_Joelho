const int pin_pot = 13;//pino do ESP32
int potenciometro = 0;//valor do ADC
float voltagem = 0;//valor da tensão 
void setup() 
{
Serial.begin(9600);
}
void loop() 
{
potenciometro = analogRead(pin_pot);
Serial.print("Valor ADC= ");
Serial.print(potenciometro);
voltagem = (potenciometro * 3.3 ) / (4095);
Serial.print(" Tensao= ");
Serial.print(voltagem);
Serial.println(" V");
delay(100);
}
