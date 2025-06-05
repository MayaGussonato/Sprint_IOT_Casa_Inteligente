
//Biblioteca
#include <DHT11.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPINO A1 //Definimos qual será o pino para o sensor
#define DHTTYPE DHT11 //Variável do tipo DHT  

DHT dht(DHTPINO, DHT11); //Inicializa o objeto DHT com o pino e o tipo


//Variaveis globais sçao da função acender  LEDAodetectarPresenca
const int PIR = 2;  // Pno Digital que o PIR está plugado
const int ledVermelho = 13;

const int MQ135 = A0; // Pino Analogico  do mq-135
const int buzzer = 13;


void acenderLEDAoDetectarPresenca() {
  int estadoPIR = digitalRead(PIR);  //Le o Pino Digital 2

  if (estadoPIR == HIGH) {
    digitalWrite(ledVermelho, HIGH);
  delay(1000);
    Serial.println("LED ligado");
  } else {
    digitalWrite(ledVermelho, LOW);
  delay(1000);
    Serial.println("LED apagado");
  }
}

void verificarVazamentoDeGas(){}

void verificarVasamentoDeGas(){
  int estadoMQ135 = analogRead(MQ135);

  if (estadoMQ135 >= 600) 					
  {				
    						
    alarme_dois_tons();
  }
  else 	
  {
     noTone (buzzer);	
  }
  



  Serial.println(estadoMQ135);

}

void alarme_dois_tons() {
  int freqAlta = 2000;
  int freqBaixa = 800;
  int duracaoTom = 250;

  tone(buzzer, freqAlta, duracaoTom);
  delay(duracaoTom);
  tone(buzzer, freqBaixa, duracaoTom);
  delay(duracaoTom);
}

 void VerificarTemperaturaEUmidade(){
float umidade = dht.readHumidity();
float temperatura = dht.readTemperature(); //Lê a temperatura

Serial.println("Umidade" + String (umidade) + "%");
Serial.println("Temperatura" + String (temperatura) + "C");
delay(5000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Fala ao programa que o pino 13 será uma saída/output
  pinMode(ledVermelho, OUTPUT);
  pinMode(MQ135, INPUT);
  pinMode(buzzer, OUTPUT);

  //Inicializa o sensor dht
  dht.begin();

  Serial.println("Calibrando os sensores");
  delay(10000);
  Serial.println("Censores calibrados!!!!;) Pode testar haha");
}

void loop() {

  // put your main code here, to run repeatedly:
  //As instruções no loop será somente chamada de funções
  //acenderLEDAoDetectarPresenca();
//verificarVasamentoDeGas();
 VerificarTemperaturaEUmidade();
}