#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#define MQTT_MAX_PACKET_SIZE 512
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Cấu hình WiFi
const char* ssid = "PCCMR";
const char* password = "aqswdefr";

// Cấu hình MQTT
const char* mqttBroker = "192.168.125.196";
const int mqttPort = 50001;
const char* mqttTopicFromWeb = "traffic/light/timings";
const char* mqttTopicToServer = "traffic/light/esp8266_data";
const char* mqttClientId = "ESP8266_TrafficLight";

// Biến lưu dữ liệu
int red_time = 21;
int yellow_time = 2;
int green_time = 20;
String command_type = "auto";
float avg_traffic_flow = 0.0;
float avg_speed = 0.0;
int motorcycle_count = 0;
int car_count = 0;
int bus_count = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#define NSS 15
#define RST 16
#define DIO0 5

const unsigned long ackTimeout = 10000; // Chờ ACK 10 giây

// Hàm kết nối WiFi
bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int wifiTimeout = 10;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout > 0) {
    delay(1000);
    Serial.print(".");
    wifiTimeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi RSSI: ");
    Serial.println(WiFi.RSSI());
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi!");
    return false;
  }
}

// Hàm gửi dữ liệu lên server qua MQTT
void publishDataToServer() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected, reconnecting...");
    connectMQTT();
  }
  StaticJsonDocument<300> doc;
  doc["red_time"] = red_time;
  doc["yellow_time"] = yellow_time;
  doc["green_time"] = green_time;
  doc["avg_traffic_flow"] = avg_traffic_flow;
  doc["avg_speed"] = avg_speed;
  doc["motorcycle_count"] = motorcycle_count;
  doc["car_count"] = car_count;
  doc["bus_count"] = bus_count;
  doc["mode"] = command_type;

  char payload[300];
  serializeJson(doc, payload);
  Serial.print("Publishing to ");
  Serial.print(mqttTopicToServer);
  Serial.print(": ");
  Serial.println(payload);

  if (mqttClient.publish(mqttTopicToServer, payload)) {
    Serial.println("Publish successful");
  } else {
    Serial.println("Publish failed");
  }
}

// Hàm gửi lệnh đến Arduino qua LoRa và chờ ACK một lần
void sendCommandToArduino() {
  char message[100];
  snprintf(message, sizeof(message), 
           "COMMAND,red_time=%d,yellow_time=%d,green_time=%d,command_type=%s",
           red_time, yellow_time, green_time, command_type.c_str());
  Serial.print("Sending to Arduino: ");
  Serial.println(message);
  
  // Gửi lệnh qua LoRa
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  Serial.println("LoRa packet sent, waiting for ACK...");

  // Chờ ACK một lần
  bool ackReceived = false;
  unsigned long startTime = millis();
  while (millis() - startTime < ackTimeout) {
    mqttClient.loop(); // Duy trì kết nối MQTT
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      char received[150] = "";
      int i = 0;
      while (LoRa.available() && i < 149) {
        received[i++] = (char)LoRa.read();
      }
      received[i] = '\0';
      if (strncmp(received, "ACK_MEGA", 8) == 0) {
        Serial.println("ACK_MEGA received: ");
        Serial.println(received);

        // Tách các giá trị từ ACK
        int ack_red_time = 0, ack_yellow_time = 0, ack_green_time = 0;
        float ack_avg_traffic_flow = 0.0, ack_avg_speed = 0.0;
        int ack_motorcycle_count = 0, ack_car_count = 0, ack_bus_count = 0;

        char* redStr = strstr(received, "red_time=");
        if (redStr) {
          ack_red_time = atoi(redStr + 9);
          Serial.print("Received red_time: ");
          Serial.println(ack_red_time);
        } else {
          Serial.println("Error: red_time not found");
        }

        char* yellowStr = strstr(received, "yellow_time=");
        if (yellowStr) {
          ack_yellow_time = atoi(yellowStr + 12);
          Serial.print("Received yellow_time: ");
          Serial.println(ack_yellow_time);
        } else {
          Serial.println("Error: yellow_time not found");
        }

        char* greenStr = strstr(received, "green_time=");
        if (greenStr) {
          ack_green_time = atoi(greenStr + 11);
          Serial.print("Received green_time: ");
          Serial.println(ack_green_time);
        } else {
          Serial.println("Error: green_time not found");
        }

        char* trafficStr = strstr(received, "avg_traffic_flow=");
        if (trafficStr) {
          ack_avg_traffic_flow = atof(trafficStr + 17);
          Serial.print("Received avg_traffic_flow: ");
          Serial.println(ack_avg_traffic_flow);
        } else {
          Serial.println("Error: avg_traffic_flow not found");
        }

        char* speedStr = strstr(received, "avg_speed=");
        if (speedStr) {
          ack_avg_speed = atof(speedStr + 10);
          Serial.print("Received avg_speed: ");
          Serial.println(ack_avg_speed);
        } else {
          Serial.println("Error: avg_speed not found");
        }

        char* motoStr = strstr(received, "motorcycle_count=");
        if (motoStr) {
          ack_motorcycle_count = atoi(motoStr + 17);
          Serial.print("Received motorcycle_count: ");
          Serial.println(ack_motorcycle_count);
        } else {
          Serial.println("Error: motorcycle_count not found");
        }

        char* carStr = strstr(received, "car_count=");
        if (carStr) {
          ack_car_count = atoi(carStr + 10);
          Serial.print("Received car_count: ");
          Serial.println(ack_car_count);
        } else {
          Serial.println("Error: car_count not found");
        }

        char* busStr = strstr(received, "bus_count=");
        if (busStr) {
          ack_bus_count = atoi(busStr + 10);
          Serial.print("Received bus_count: ");
          Serial.println(ack_bus_count);
        } else {
          Serial.println("Error: bus_count not found");
        }

        char* modeStr = strstr(received, "mode=");
        if (modeStr) {
          char modeValue[20];
          sscanf(modeStr, "mode=%19s", modeValue);
          command_type = String(modeValue);
          Serial.print("Received mode: ");
          Serial.println(command_type);
        } else {
          Serial.println("Error: mode not found");
        }

        if (ack_red_time > 0) red_time = ack_red_time;
        if (ack_yellow_time > 0) yellow_time = ack_yellow_time;
        if (ack_green_time > 0) green_time = ack_green_time;
        if (ack_avg_traffic_flow > 0) avg_traffic_flow = ack_avg_traffic_flow;
        if (ack_avg_speed > 0) avg_speed = ack_avg_speed;
        if (ack_motorcycle_count >= 0) motorcycle_count = ack_motorcycle_count;
        if (ack_car_count >= 0) car_count = ack_car_count;
        if (ack_bus_count >= 0) bus_count = ack_bus_count;

        ackReceived = true;
        publishDataToServer();
        break;
      } else {
        Serial.print("Received unknown LoRa packet: ");
        Serial.println(received);
      }
    }
    delay(5);
  }

  if (!ackReceived) {
    Serial.println("No ACK_MEGA received, publishing current data to server");
    publishDataToServer();
  }
}

// Hàm callback khi nhận tin nhắn MQTT từ web
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  char message[100];
  strncpy(message, (char*)payload, length);
  message[length] = '\0';
  Serial.println(message);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  red_time = doc["red_time"] | 21;
  yellow_time = doc["yellow_time"] | 2;
  green_time = doc["green_time"] | 20;
  command_type = doc["command_type"] | "auto";

  Serial.println("Updated command:");
  Serial.print("Red Time: "); Serial.println(red_time);
  Serial.print("Yellow Time: "); Serial.println(yellow_time);
  Serial.print("Green Time: "); Serial.println(green_time);
  Serial.print("Command Type: "); Serial.println(command_type);

  sendCommandToArduino();
}

// Hàm kết nối MQTT
void connectMQTT() {
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(mqttBroker);
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("MQTT connected!");
      mqttClient.subscribe(mqttTopicFromWeb);
      Serial.print("Subscribed to topic: ");
      Serial.println(mqttTopicFromWeb);
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 10 seconds...");
      delay(10000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender with MQTT - ESP8266");

  if (!connectWiFi()) {
    Serial.println("Initial WiFi connection failed! Retrying in loop...");
  }

  connectMQTT();

  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);
  LoRa.setTxPower(20);

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Attempting to reconnect...");
    if (!connectWiFi()) {
      Serial.println("Reconnect failed! Retrying in 10 seconds...");
      delay(10000);
      return;
    }
  }

  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();
}