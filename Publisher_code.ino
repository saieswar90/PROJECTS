#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// WiFi credentials
const char* ssid = "System1";
const char* password = "1234567890";

// MQTT Broker
const char* mqtt_server = "172.16.56.247"; // MQTT broker address
const int mqtt_port = 1883; // MQTT port
const char* mqtt_username = ""; // MQTT username
const char* mqtt_password = ""; // MQTT password

// MQTT topics
const char* mqtt_dht_topic = "sensor/dht11";
const char* mqtt_hall_topic = "sensor/hall";
const char* mqtt_sound_topic = "sensor/sound";
const char* mqtt_ultrasonic_topic = "sensor/dist";

// Pin Definitions
#define DHTPIN 15        // Pin connected to the DHT sensor
#define DHTTYPE DHT11    // DHT 11
#define HALL_PIN 4       // Pin connected to the output of the Hall sensor
#define SOUND_PIN 34     // Analog pin connected to the output of the sound sensor
#define TRIG_PIN 5       // Pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN 18      // Pin connected to the echo pin of the ultrasonic sensor

// Constants
#define SOUND_SPEED 340
#define TRIG_PULSE_DURATION_US 10

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT Broker...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  setup_mqtt();

  dht.begin();

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(SOUND_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temperature)) {
    String dht_payload = String(temperature) + "," + String(humidity);
    client.publish(mqtt_dht_topic, dht_payload.c_str());
    Serial.println("DHT11 Sensor Data published to MQTT topic: " + String(dht_payload));
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  int hallState = digitalRead(HALL_PIN);
  client.publish(mqtt_hall_topic, String(hallState).c_str());
  Serial.println("Hall Sensor State published to MQTT topic: " + String(hallState));

  int soundValue = analogRead(SOUND_PIN);
  client.publish(mqtt_sound_topic, String(soundValue).c_str());
  Serial.println("Sound Sensor Value published to MQTT topic: " + String(soundValue));

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_PULSE_DURATION_US);
  digitalWrite(TRIG_PIN, LOW);
  long ultrason_duration = pulseIn(ECHO_PIN, HIGH);
  float distance_cm = ultrason_duration * SOUND_SPEED / 2 * 0.0001;

  Serial.print("Distance (cm): ");
  Serial.println(distance_cm);

  if (distance_cm <= 3) {
     client.publish(mqtt_ultrasonic_topic, String(distance_cm).c_str());
  }

  delay(2000);
}
