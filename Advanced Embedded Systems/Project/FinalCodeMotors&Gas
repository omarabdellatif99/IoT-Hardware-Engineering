#include <WiFiNINA.h>
#include <PubSubClient.h>

// ——— Wi-Fi & MQTT Configuration ———————————————————————
char ssid[] = "Vodafone-374899";
char pass[] = "3Kn66LnxGpCe9ed9";
const char* mqtt_server   = "broker.hivemq.com";
const char* control_topic = "myroom/message";

// ——— Hardware Pins ————————————————————————————————————————
const int gasPin    = A0;   // MQ-2 analog output
const int in1Pin    = 2;    // L298N IN1
const int in2Pin    = 3;    // L298N IN2
const int in3Pin    = 4;    // L298N IN1
const int in4Pin    = 5;  
const int enbpin    = 10; 
const int enaPin    = 9;    // L298N ENA (PWM)
const int threshold = 100;  // Gas trigger threshold

// ——— Globals ——————————————————————————————————————————————
WiFiClient wifiClient;
PubSubClient client(wifiClient);

String lastCommand = "";
int motorSpeed     = 0;

// ——— Reconnect to MQTT ————————————————————————————————————
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    if (client.connect("arduinoClient_1")) {
      Serial.println(" connected");
      client.subscribe(control_topic);
    } else {
      Serial.print(" failed, rc="); Serial.print(client.state());
      Serial.println("; retrying in 1s");
      delay(1000);
    }
  }
}

// ——— Handle MQTT Messages ——————————————————————————————
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("← MQTT Command received: "); Serial.println(msg);
  lastCommand = msg;

  // If command is "speed:xxx", extract the number
  if (msg.startsWith("speed:")) {
    int val = msg.substring(6).toInt();  // Get the number after "speed:"
    if (val >= 0 && val <= 255) {
      motorSpeed = val;
      Serial.print("→ Motor speed set to: "); Serial.println(motorSpeed);
    } else {
      Serial.println("⚠️ Speed out of range (0–255)");
    }
  } else if (msg == "Reset") {
    Serial.println("→ Entering 'Reset' mode (gas-sensitive)");
  } else {
    Serial.println("⚠️ Unknown command");
  }
}


// ——— Setup ———————————————————————————————————————————————
void setup() {
  Serial.begin(9600);

  pinMode(gasPin, INPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enaPin, OUTPUT);

  // Stop motor initially
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enaPin, 0);

  Serial.println("Setup complete. Connecting to Wi-Fi...");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(" connected to Wi-Fi");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();
}

// ——— Main Loop —————————————————————————————————————————————
void loop() {
    int reading = analogRead(gasPin);
  if (!client.connected()) reconnect();
  client.loop();

  int gasLevel = analogRead(gasPin);
  Serial.print("Gas level: "); Serial.println(gasLevel);

  bool shouldRun = false;

  if (lastCommand == "Reset") {
    if (gasLevel > threshold) {
      shouldRun = true;
    } else {
      shouldRun = false;
    }
  } else {
    shouldRun = true;
  }

  if (shouldRun && motorSpeed > 0) {
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
    analogWrite(enbpin, motorSpeed);
    Serial.print("→ Motor running at speed: "); Serial.println(motorSpeed);
  } else {
    analogWrite(enbpin, 0);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
    Serial.println("→ Motor stopped");
  }
   if (reading > threshold) {
    // Spin Motor A forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enaPin, 255);  // full speed
    Serial.println("→ Motor A ON");
  } else {
    // Stop Motor A
    analogWrite(enaPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    Serial.println("→ Motor A OFF");
  }

  delay(1000);
}
