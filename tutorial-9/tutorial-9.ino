#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <analogWrite.h>

const char* ssid = "DANIL-WIFI"; // Enter your WiFi name
const char* password = "Hendra0711!"; // Enter WiFi password

#define mqttServer "broker.hivemq.com"
#define mqttPort 1883

WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

String Topic;
String Payload;

#define T1       34
#define T2       35
#define LED      26
#define Q1       32
#define Q2       33

float cel, cel1, degC, degC1;
const float batas_suhu_atas = 58;

void Q1on() {
  analogWrite(Q1, 100);
  // analogWrite(Q1, 100); // analogwrite(pin,period,frequency,resolusi,phase)
}

void Q1off() {
  analogWrite(Q1, 0, 255);
}

void Q2on() {
  analogWrite(Q2, 100);
  // analogWrite(Q2, 341, 5000, 10, 341); // analogwrite(pin,period,frequency,resolusi,phase)
}

void Q2off() {
  analogWrite(Q2, 0);
}

void ledon() {
  analogWrite(LED, 255);
}

void ledoff() {
  analogWrite(LED, 0);
}

void cektemp() {
  degC = analogRead(T1) * 0.322265625;    // use for 3.3v AREF
  cel = degC / 10;
  degC1 = analogRead(T2) * 0.322265625;    // use for 3.3v AREF
  cel1 = degC1 / 10;

  Serial.print("Temperature: ");
  Serial.print(cel);   // print the temperature T1 in Celsius
  Serial.print("Â°C");
  Serial.print("  ~  "); // separator between Celsius and Fahrenheit
  Serial.print(cel1);   // print the temperature T2 in Celsius
  Serial.println("Â°C");
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {

  /* we got '1' -> Q1_on */
  if ((char)payload[0] == '1') {
    Q1on();
    Serial.println("Q1 On");
  }

  /* we got '2' -> Q1_off */
  if ((char)payload[0] == '2') {
    Q1off();
    Serial.println("Q1 Off");
  }

  /* we got '3' -> Q2_on */
  if ((char)payload[0] == '3') {
    Q2on();
    Serial.println("Q2 On");
  }

  /* we got '4' -> Q2_off */
  if ((char)payload[0] == '4') {
    Q2off();
    Serial.println("Q2 Off");
  }
}


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  analogWriteFrequency(5000); // set frequency to 10 KHz for all pins
  analogWriteResolution(LED, 10);
  analogWriteResolution(Q1, 10);
  analogWriteResolution(Q2, 10);

  // Connect to WiFi network
  Serial.println();
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

  // Connect to Server IoT (CloudMQTT)
  client.setServer(mqttServer, mqttPort);
  client.setCallback(receivedCallback);

  while (!client.connected()) {
    Serial.println("Connecting to CLoud IoT ...");

    //    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
    if (client.connect("iTCLab Suhu On/Off")) {

      Serial.println("connected");
      Serial.print("Message received: ");


    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
    client.subscribe("heater1");
    client.subscribe("heater2");
  }
}

void loop() {
  char suhu1[4];
  char suhu2[4];
  client.loop();

  // put your main code here, to run repeatedly:
  cektemp();
  if (cel > batas_suhu_atas) {
    Q1off();
    ledon();
  } else {
    Q1on();
    ledoff();
  }
  if (cel1 > batas_suhu_atas) {
    Q2off();
    ledon();
  } else {
    Q2on();
    ledoff();
  }
  delay(100);


  Serial.print("Temperature T1: ");
  Serial.print(cel);
  Serial.print(" Celcius ");
  Serial.println(" send to i-ot.net");

  dtostrf(cel, 1, 0, suhu1);
  client.publish("Suhu1", suhu1);

  delay(200);

  Serial.print("Temperature T2: ");
  Serial.print(cel1);
  Serial.print(" Celcius ");
  Serial.println(" send to i-ot.net");

  dtostrf(cel1, 1, 0, suhu2);
  client.publish("Suhu2", suhu2);
  delay(200);
}