#include <WiFiNINA.h>
#include "secret.h" 
#include <ArduinoMqttClient.h>
#include <Ultrasonic.h>

#define ledPin 17           // LED Output
#define dist_threshold 15   // Detect hand movements within this many CM
#define pat_time 1000       // A pat is a detection that lasts this many ms, anything less than this is a wave.
#define loop_delay 50       // time between checking ultrasonic sensor

// Setting up WiFi
WiFiClient client;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS; 

// Setting up MQTT
MqttClient mqttClient(client);
const char broker[] = "broker.emqx.io";
int        port     = 1883;
const char wave_topic[]  = "SIT210/wave";
const char pat_topic[]  = "SIT210/pat";
const char message[]  = "JPKarnilowicz";

// Setting up Ultrasonic Hand Detection
Ultrasonic ultrasonic(14, 15);  // create ultrasonic object
int distance;                   // variable to store distance from ultrasonic
unsigned long timer;            // timer to differentiate between "wave" and "pat"
bool detectflag = false;        // flag to determine if a hand has been detected


void setup() { 
  pinMode(ledPin, OUTPUT);      // enables LED output
  Serial.begin(9600);           // enables Serial Comms
  while (!Serial) {}            // wait for serial to connect

  // Connect to Wifi Network
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println("You're connected to the network!");
  Serial.println();

  // Connect to MQTT Broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // subscribe to topics
  mqttClient.subscribe(wave_topic);
  mqttClient.subscribe(pat_topic);

  // Allocate function for when MQTT message is received
  mqttClient.onMessage(onMessage);
}


void loop() {
  mqttClient.poll();
  distance = ultrasonic.read();

  if (!detectflag and distance <= dist_threshold) { // If no hand detected currently, but something is within distance,  
    
    timer = millis();                               // Then start timer, flag hand detected
    detectflag = true;

  } else if (detectflag and distance > dist_threshold) { // Otherwise is hand is currently detected, but nothing is within distance,
      detectflag = false;                                // unflag as detected, and finish timer. 
      timer = millis() - timer;

      Serial.print("Hand detected for ");   // Comm
      Serial.print(timer);
      Serial.print("ms , ");

      if (timer < pat_time) { // Now decide next steps on how long the hand was detected.               
        Serial.println("this was a WAVE.");
        sendWaveMessage();   
      } else {
        Serial.println("this was a PAT.");
        sendPatMessage();
      }
  }

  delay(loop_delay);
}

void onMessage(int messageSize) {

  // Decide on action depending on topic of message received.
  if(mqttClient.messageTopic() == wave_topic) {
    ledWaveAction();
  } else if (mqttClient.messageTopic() == pat_topic) {
    ledPatAction();
  }

  // Print out message
  Serial.print("Received a message on topic ");
  Serial.print(mqttClient.messageTopic());
  Serial.print(": ");

  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }

  Serial.println();
}

void sendWaveMessage() {
  Serial.print("Sending ");
  Serial.print(message);
  Serial.print(" to topic ");
  Serial.println(wave_topic);

  mqttClient.beginMessage(wave_topic);
  mqttClient.println(message);
  mqttClient.endMessage();
}

void sendPatMessage() {
  Serial.print("Sending ");
  Serial.print(message);
  Serial.print(" to topic ");
  Serial.println(pat_topic);

  mqttClient.beginMessage(pat_topic);
  mqttClient.println(message);
  mqttClient.endMessage();
}


void ledWaveAction() {
  digitalWrite(ledPin, HIGH);
  delay(50);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(50);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(50);
  digitalWrite(ledPin, LOW);
}


void ledPatAction() {
  digitalWrite(ledPin, HIGH);
  delay(250);
  digitalWrite(ledPin, LOW);
}


