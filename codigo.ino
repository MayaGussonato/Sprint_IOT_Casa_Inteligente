#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP32Servo.h>

// --- WiFi & MQTT ---
const char* ssid = "ProjetosIoT_Esp32";//sua rede wifi
// const char* ssid = "SALA 16";//sua rede wifi
const char* password = "senai@134";//senha da sua rede wifi
const char* mqtt_server = "broker.hivemq.com";//endereço do broker público
const int mqtt_port = 1883;//porta do broker público, geralmente 1883

//Tópicos
const char* topic_led = "techeagles/lab19/luzsala";
const char* topic_temp = "escolainteligente/lab19/temperatura";
const char* topic_umid = "escolainteligente/lab19/umidade";
const char* topic_porta = "techeagles/lab19/porta";

// --- Pinos ---
const int servoMotor = 19; 
const int PIR = 5;  // GPIO5 que o pir está plucado
const int LedSala = 13;   // LED que vai acender com presença
const int MQ135 = 34;   // GPIO34  do mq-135
const int buzzer = 12;  //
const int rele = 15;

#define DHTPIN 33
#define DHTTYPE DHT11

// --- Objetos ---
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
Servo motor;

// --- Variáveis ---
int contadorGas = 0;
unsigned long ultimaLeitura = 0;

// --- Funções WiFi e MQTT ---
void conectarWiFi() {
  Serial.println("Conectando ao WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

void reconectarMQTT() {
  while (!client.connected()) {
    Serial.print("Reconectando MQTT...");
    if (client.connect("ESP32ClientTest")) {
      Serial.println("Conectado!");
      client.subscribe(topic_led);
      client.subscribe(topic_porta);
    } else {
      Serial.print("Falha: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void tratarMensagem(char* topic, byte* payload, unsigned int length) {
  String mensagem = "";
  for (int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }

  Serial.printf("Mensagem recebida [%s]: %s\n", topic, mensagem.c_str());

  if (strcmp(topic, topic_led) == 0) {
    if (mensagem == "ligar") {
      digitalWrite(LedSala, HIGH); // Use ledAmarelo, pois luzSala não está definido
    } else if (mensagem == "desligar") {
      digitalWrite(LedSala, LOW);
    }
  }

  if (strcmp(topic, topic_porta) == 0) {      
    if (mensagem == "abrir") {
      destrancarPorta();
      delay(500);
      abrirPortaAutomatico();
    } else if (mensagem == "fechar") {
      fecharPortaAutomatico();
      delay(500);
      trancarPorta();
    }
  }
}

// --- Sensores e atuadores ---
void lerSensorEDisponibilizar() {
  float temperatura = dht.readTemperature();
  float umidade = dht.readHumidity();

  if (isnan(temperatura) || isnan(umidade)) {
    Serial.println("Erro ao ler DHT!");
    return;
  }

  Serial.printf("Temp: %.1f °C | Umid: %.1f %%\n", temperatura, umidade);

  char tempStr[10], umidStr[10];
  dtostrf(temperatura, 4, 1, tempStr);
  dtostrf(umidade, 4, 1, umidStr);
  client.publish(topic_temp, tempStr);
  client.publish(topic_umid, umidStr);
}

void acenderLEDAoDetectarPresenca() {
  digitalWrite(LedSala, digitalRead(PIR));
}

void verificarVazamentoDeGas() {
  int leituraMQ = analogRead(MQ135);
  Serial.print("Gás: ");
  Serial.println(leituraMQ);

  if (leituraMQ >= 800) {
    if (contadorGas == 0) {
      Serial.println("Gás detectado!");
      delay(3000);
      contadorGas = 1;
    }
    alarme_dois_tons();
  } else {
    if (contadorGas == 1) {
      contadorGas = 0;
      noTone(buzzer);
    }
  }
}

void alarme_dois_tons() {
  tone(buzzer, 2000, 250);
  delay(250);
  tone(buzzer, 800, 250);
  delay(250);
}

void destrancarPorta() {
  digitalWrite(rele, HIGH);
  Serial.println("Porta destrancada");
}

void trancarPorta() {
  digitalWrite(rele, LOW);
  Serial.println("Porta trancada");
}

void abrirPortaAutomatico() {
  motor.write(0);  // 0 graus = porta aberta
  Serial.println("Porta aberta");
}

void fecharPortaAutomatico() {
  motor.write(180);  // 180 graus = porta fechada
  Serial.println("Porta fechada");
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(LedSala, OUTPUT);
  pinMode(PIR, INPUT);
 // pinMode(MQ135, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(rele, INPUT);
  digitalWrite(rele, LOW); // Porta fechada por padrão

  motor.attach(servoMotor);
  motor.write(180); // Porta fechada no início

  dht.begin();

  conectarWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(tratarMensagem);

  Serial.println("Sistema iniciado!");
}

// --- Loop ---
void loop() {
 ;
  if (!client.connected()) reconectarMQTT();
  client.loop();

  //acenderLEDAoDetectarPresenca();
  //verificarVazamentoDeGas();

  if (millis() - ultimaLeitura > 5000) {
    ultimaLeitura = millis();
    //lerSensorEDisponibilizar();
  }
}
