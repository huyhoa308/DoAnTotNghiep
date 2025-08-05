#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <LoRa.h>
#include <semphr.h>
#include <avr/wdt.h>
#include <ArduinoJson.h>

// Hàm kiểm tra SRAM
int freeMemory() {
  char top;
  return &top - __malloc_heap_start;
}

// --- Chân LoRa ---
#define NSS 48
#define RST 49
#define DIO0 2

// --- LED 7 đoạn và đèn 4 hướng ---
const byte north_a = 23, north_b = 25, north_c = 27, north_d = 29, north_e = 31, north_f = 33, north_g = 35;
const byte north_com_tens = 4, north_com_units = 3;
const byte north_red = 41, north_yellow = 39, north_green = 37;
const byte south_a = A0, south_b = A1, south_c = A2, south_d = A3, south_e = A4, south_f = A5, south_g = A6;
const byte south_com_tens = A8, south_com_units = A7;
const byte south_red = 47, south_yellow = 45, south_green = 43;
const byte east_a = 22, east_b = 24, east_c = 26, east_d = 28, east_e = 30, east_f = 32, east_g = 34;
const byte east_com_tens = 8, east_com_units = 7;
const byte east_red = 40, east_yellow = 38, east_green = 36;
const byte west_a = A9, west_b = A10, west_c = A11, west_d = A12, west_e = A13, west_f = A14, west_g = A15;
const byte west_com_tens = 6, west_com_units = 5;
const byte west_red = 46, west_yellow = 44, west_green = 42;

// Mảng chân segment
const byte north_segments[7] = {north_a, north_b, north_c, north_d, north_e, north_f, north_g};
const byte south_segments[7] = {south_a, south_b, south_c, south_d, south_e, south_f, south_g};
const byte east_segments[7] = {east_a, east_b, east_c, east_d, east_e, east_f, east_g};
const byte west_segments[7] = {west_a, west_b, west_c, west_d, west_e, west_f, west_g};

// Mảng chân common
const byte common_pins[8] = {north_com_tens, north_com_units, south_com_tens, south_com_units,
                             east_com_tens, east_com_units, west_com_tens, west_com_units};

// Mảng hiển thị số 0-9
const byte numbers[11] = {B1111110, B0110000, B1101101, B1111001, B0110011,
                          B1011011, B1011111, B1110000, B1111111, B1111011, B0000000};

// Cấu trúc dữ liệu cho Queue
typedef struct {
  int red_time;
  int yellow_time;
  int green_time;
  float avg_traffic_flow;
  float avg_speed;
  int motorcycle_count;
  int car_count;
  int bus_count;
  String command_type;
} TrafficData;

// Biến toàn cục
const int all_red_time = 1; // Thời gian tất cả đỏ (giây)
const int scan_delay = 1000; // Giữ 1000µs
int primary_red_time = 21; // Đỏ hướng chính (Đông-Tây, biến động)
int primary_green_time = 10; // Xanh hướng chính (biến động)
int primary_yellow_time = 2; // Vàng hướng chính (biến động)
const int secondary_green_time = 18; // Xanh hướng phụ (Bắc-Nam)
const int secondary_yellow_time = 2; // Vàng hướng phụ
int secondary_red_time = primary_green_time + primary_yellow_time + all_red_time; // Đỏ hướng phụ (biến động)
String mode = "auto"; // Chế độ hoạt động: auto, manual, yellow_all

// Semaphores
SemaphoreHandle_t serialSemaphore;
SemaphoreHandle_t serial1Semaphore;
SemaphoreHandle_t spiSemaphore;
SemaphoreHandle_t timeSemaphore;

// Queue
QueueHandle_t trafficDataQueue;
QueueHandle_t greenTimeQueue;

// Task prototypes
void LoRaTask(void *pvParameters);
void TrafficLightTask(void *pvParameters);
void UartTask(void *pvParameters);
void multiplexAll(int north, int south, int east, int west);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial1.begin(115200);

  pinMode(13, INPUT);
  

  serialSemaphore = xSemaphoreCreateMutex();
  serial1Semaphore = xSemaphoreCreateMutex(); // Tạo semaphore cho Serial1
  spiSemaphore = xSemaphoreCreateMutex();

  if (serialSemaphore == NULL || serial1Semaphore == NULL || spiSemaphore == NULL) {
    Serial.println("Failed to create semaphores");
    while (1);
  }
  timeSemaphore = xSemaphoreCreateMutex();
  if (timeSemaphore == NULL) {
    Serial.println("Failed to create timeSemaphore");
    while (1);
  }

  // Tạo Queue
  trafficDataQueue = xQueueCreate(1, sizeof(TrafficData)); // Chỉ chứa 1 phần tử
  if (trafficDataQueue == NULL) {
    Serial.println("Failed to create trafficDataQueue");
    while (1);
  }
  greenTimeQueue = xQueueCreate(1, sizeof(int));
  if (greenTimeQueue == NULL) {
    Serial.println("Failed to create greenTimeQueue");
    while (1);
  }
  TrafficData defaultData = {21, 2, 25, 0.0, 0.0, 0, 0, 0, "auto"};
  xQueueOverwrite(trafficDataQueue, &defaultData);

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("Setup started");
  Serial.println("Running updated code v1.15 - 2025-06-22");
  Serial.print("Free SRAM before tasks: ");
  Serial.println(freeMemory());
  xSemaphoreGive(serialSemaphore);

  LoRa.setPins(NSS, RST, DIO0);
  xSemaphoreTake(spiSemaphore, portMAX_DELAY);
  if (!LoRa.begin(433E6)) {
    xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    Serial.println("LoRa init failed");
    xSemaphoreGive(serialSemaphore);
    xSemaphoreGive(spiSemaphore);
    while (1);
  }
  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);
  LoRa.setTxPower(20);
  xSemaphoreGive(spiSemaphore);
  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("LoRa init OK");
  xSemaphoreGive(serialSemaphore);

  for (int i = 0; i < 7; i++) {
    pinMode(north_segments[i], OUTPUT);
    pinMode(south_segments[i], OUTPUT);
    pinMode(east_segments[i], OUTPUT);
    pinMode(west_segments[i], OUTPUT);
  }
  for (int i = 0; i < 8; i++) {
    pinMode(common_pins[i], OUTPUT);
    digitalWrite(common_pins[i], HIGH);
  }
  byte leds[] = {north_red, north_yellow, north_green, south_red, south_yellow, south_green,
                 east_red, east_yellow, east_green, west_red, west_yellow, west_green};
  for (byte p : leds) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("Creating tasks");
  Serial.print("Free SRAM after init: ");
  Serial.println(freeMemory());
  xSemaphoreGive(serialSemaphore);

  TaskHandle_t loRaTaskHandle, trafficTaskHandle, uartTaskHandle;
  wdt_enable(WDTO_8S); // Reset sau 8 giây nếu treo
  if (xTaskCreate(LoRaTask, "LoRa", 1536, NULL, 1, &loRaTaskHandle) != pdPASS) {
    xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    Serial.println("Failed to create LoRaTask");
    xSemaphoreGive(serialSemaphore);
    while (1);
  }
  if (xTaskCreate(TrafficLightTask, "Traffic", 2048, NULL, 1, &trafficTaskHandle) != pdPASS) {
    xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    Serial.println("Failed to create TrafficLightTask");
    xSemaphoreGive(serialSemaphore);
    while (1);
  }
  if (xTaskCreate(UartTask, "Uart", 1536, NULL, 1, &uartTaskHandle) != pdPASS) { // Ưu tiên 2
    xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    Serial.println("Failed to create UartTask");
    xSemaphoreGive(serialSemaphore);
    while (1);
  }

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("Starting scheduler");
  xSemaphoreGive(serialSemaphore);

  vTaskStartScheduler();
}

void loop() {}

void clearDirectionLeds(int red, int yellow, int green) {
  digitalWrite(red, HIGH);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, HIGH);
}

void UartTask(void *pvParameters) {
  (void) pvParameters;
  char buffer[128];
  int index = 0;
  static int uart_operation_count = 0; // Đếm số lần xử lý gói tin

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("UartTask started");
  Serial.print("Current mode: ");
  Serial.println(mode);
  xSemaphoreGive(serialSemaphore);

  for (;;) {
    index = 0;
    memset(buffer, 0, sizeof(buffer));
    Serial1.flush();

    unsigned long startTime = millis();
    bool packetComplete = false;
    while (millis() - startTime < 500 && !packetComplete) {
      if (Serial1.available()) {
        char c = Serial1.read();
        if (index < 127) {
          buffer[index++] = c;
          if (c == '\n') {
            buffer[index] = '\0';
            packetComplete = true;
            break;
          }
        }
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    if (index > 0 && packetComplete) {
      xSemaphoreTake(serialSemaphore, portMAX_DELAY);
      Serial.print("Raw data received: ");
      Serial.println(buffer);
      xSemaphoreGive(serialSemaphore);

      TrafficData data;
      char *token = strtok(buffer, ",");
      int fieldCount = 0;
      while (token != NULL && fieldCount < 6) {
        switch (fieldCount) {
          case 0: data.green_time = atoi(token); break;
          case 1: data.avg_traffic_flow = atof(token); break;
          case 2: data.avg_speed = atof(token); break;
          case 3: data.motorcycle_count = atoi(token); break;
          case 4: data.car_count = atoi(token); break;
          case 5: data.bus_count = atoi(token); break;
        }
        token = strtok(NULL, ",");
        fieldCount++;
      }
      if (fieldCount == 6) {
        // Đọc dữ liệu hiện tại từ trafficDataQueue
        TrafficData currentData;
        bool queueHasData = false;
        if (xQueuePeek(trafficDataQueue, &currentData, 0) == pdPASS) {
          queueHasData = true;
        }

        // Cập nhật các trường cần thiết
        currentData.avg_traffic_flow = data.avg_traffic_flow;
        currentData.avg_speed = data.avg_speed;
        currentData.motorcycle_count = data.motorcycle_count;
        currentData.car_count = data.car_count;
        currentData.bus_count = data.bus_count;

        // Ghi đè dữ liệu đã cập nhật vào trafficDataQueue
        xQueueOverwrite(trafficDataQueue, &currentData);
      }
      if (fieldCount == 6 && mode == "auto") {
        // Nếu mode là auto, ghi đè green_time vào greenTimeQueue
        if (data.green_time > 0) {
          xQueueOverwrite(greenTimeQueue, &data.green_time);
        }

        xSemaphoreTake(serialSemaphore, portMAX_DELAY);
        Serial.print("Parsed data: green_time=");
        Serial.print(data.green_time);
        Serial.print(", avg_traffic_flow=");
        Serial.print(data.avg_traffic_flow);
        Serial.print(", avg_speed=");
        Serial.print(data.avg_speed);
        Serial.print(", motorcycle_count=");
        Serial.print(data.motorcycle_count);
        Serial.print(", car_count=");
        Serial.print(data.car_count);
        Serial.print(", bus_count=");
        Serial.println(data.bus_count);
        Serial.println("Sent ACK after 50ms delay");
        xSemaphoreGive(serialSemaphore);
        // Gửi ACK ngay lập tức, chỉ delay 50ms
        xSemaphoreTake(serial1Semaphore, portMAX_DELAY);
        Serial1.println("ACK");
        xSemaphoreGive(serial1Semaphore);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Tăng biến đếm và kiểm tra reset UartTask
        uart_operation_count++;
        if (uart_operation_count >= 2) {
          xSemaphoreTake(serial1Semaphore, portMAX_DELAY);
          Serial1.end(); // Ngắt UART
          Serial1.begin(115200); // Khởi động lại UART
          Serial1.flush(); // Xóa bộ đệm UART
          xSemaphoreGive(serial1Semaphore);
          memset(buffer, 0, sizeof(buffer)); // Xóa bộ đệm
          index = 0; // Đặt lại index
          uart_operation_count = 0; // Đặt lại biến đếm
          xSemaphoreTake(serialSemaphore, portMAX_DELAY);
          Serial.println("UartTask reset after 2 operations");
          xSemaphoreGive(serialSemaphore);
        }
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
    wdt_reset();
  }
}

void LoRaTask(void *pvParameters) {
  (void) pvParameters;

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("LoRaTask started");
  xSemaphoreGive(serialSemaphore);

  for (;;) {
    if (xSemaphoreTake(spiSemaphore, portMAX_DELAY) == pdTRUE) {
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        char received[150] = "";
        int rssi = LoRa.packetRssi();
        int i = 0;
        while (LoRa.available() && i < 99) {
          received[i++] = (char)LoRa.read();
        }
        received[i] = '\0';

        if (strncmp(received, "COMMAND", 7) == 0) {
          // Lấy dữ liệu hiện tại từ trafficDataQueue
          TrafficData currentData;
          if (xQueuePeek(trafficDataQueue, &currentData, 0) != pdPASS) {
            // Nếu không lấy được dữ liệu, khởi tạo mặc định
            currentData = {21, 2, 25, 0, 0, 0, 0, 0, "auto"};
          }

          // Phân tích dữ liệu nhận được
          char* redStr = strstr(received, "red_time=");
          if (redStr) currentData.red_time = atoi(redStr + 9);
          char* yellowStr = strstr(received, "yellow_time=");
          if (yellowStr) currentData.yellow_time = atoi(yellowStr + 12);
          char* greenStr = strstr(received, "green_time=");
          if (greenStr) currentData.green_time = atoi(greenStr + 11);
          char* cmdStr = strstr(received, "command_type=");
          String newCommandType = currentData.command_type;
          if (cmdStr) {
            char cmd[20] = {0}; // Khởi tạo bộ đệm sạch
            // Trích xuất command_type, bỏ qua ký tự không mong muốn
            if (sscanf(cmdStr + 13, "%19[^,]", cmd) == 1) {
              newCommandType = String(cmd);
              // Kiểm tra giá trị hợp lệ
              if (newCommandType != "auto" && newCommandType != "manual" && newCommandType != "yellow_all") {
                xSemaphoreTake(serialSemaphore, portMAX_DELAY);
                Serial.print("Invalid command_type received: ");
                Serial.println(newCommandType);
                xSemaphoreGive(serialSemaphore);
                newCommandType = currentData.command_type; // Giữ nguyên giá trị hiện tại
              }
            } else {
              xSemaphoreTake(serialSemaphore, portMAX_DELAY);
              Serial.println("Failed to parse command_type");
              xSemaphoreGive(serialSemaphore);
            }
          }

          xSemaphoreTake(serialSemaphore, portMAX_DELAY);
          Serial.print("Parsed command_type: ");
          Serial.println(newCommandType);
          xSemaphoreGive(serialSemaphore);

          // Hàm gửi ACK
          auto sendAck = [&]() {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Chờ 200ms trước khi gửi ACK
            char flowStr[10], speedStr[10];
            dtostrf(currentData.avg_traffic_flow, 5, 2, flowStr);
            dtostrf(currentData.avg_speed, 5, 2, speedStr);

            char ackMessage[150];
            snprintf(ackMessage, sizeof(ackMessage),
                     "ACK_MEGA,red_time=%d,yellow_time=%d,green_time=%d,avg_traffic_flow=%s,avg_speed=%s,motorcycle_count=%d,car_count=%d,bus_count=%d,mode=%s",
                     currentData.red_time, currentData.yellow_time, currentData.green_time, flowStr, speedStr,
                     currentData.motorcycle_count, currentData.car_count, currentData.bus_count, currentData.command_type.c_str());

            LoRa.beginPacket();
            LoRa.print(ackMessage);
            LoRa.endPacket();

            xSemaphoreTake(serialSemaphore, portMAX_DELAY);
            Serial.print("Sent LoRa ACK: ");
            Serial.println(ackMessage);
            xSemaphoreGive(serialSemaphore);
          };

          // Xử lý theo command_type
          if (newCommandType == "auto") {
            // Chỉ cập nhật command_type
            currentData.command_type = newCommandType;
            mode = newCommandType;
            xQueueOverwrite(trafficDataQueue, &currentData);

            xSemaphoreTake(serialSemaphore, portMAX_DELAY);
            Serial.println("Received LoRa COMMAND with command_type=auto, updated command_type only");
            Serial.print("Updated mode: ");
            Serial.println(mode);
            xSemaphoreGive(serialSemaphore);
            sendAck();
          } else if (newCommandType == "manual") {
            // Cập nhật các biến thời gian và toàn bộ dữ liệu
            // primary_red_time = currentData.red_time;
            // primary_yellow_time = currentData.yellow_time;
            // primary_green_time = currentData.green_time;
            // secondary_red_time = primary_green_time + primary_yellow_time + all_red_time;
            currentData.command_type = newCommandType;
            mode = newCommandType;
            xQueueOverwrite(trafficDataQueue, &currentData);
            xQueueOverwrite(greenTimeQueue, &currentData.green_time);

            xSemaphoreTake(serialSemaphore, portMAX_DELAY);
            Serial.print("Received LoRa COMMAND: ");
            Serial.println(received);
            Serial.println("Updated to manual mode");
            Serial.print("Updated primary_red_time: ");
            Serial.println(primary_red_time);
            Serial.print("Updated primary_yellow_time: ");
            Serial.println(primary_yellow_time);
            Serial.print("Updated primary_green_time: ");
            Serial.println(primary_green_time);
            Serial.print("Updated secondary_red_time: ");
            Serial.println(secondary_red_time);
            Serial.print("Updated mode: ");
            Serial.println(mode);
            xSemaphoreGive(serialSemaphore);
            sendAck();
          } else if (newCommandType == "yellow_all") {
            // Chỉ cập nhật command_type và đặt thời gian về 0
            currentData.command_type = newCommandType;
            currentData.red_time = 0;
            currentData.yellow_time = 0;
            currentData.green_time = 0;
            mode = newCommandType;
            xQueueOverwrite(trafficDataQueue, &currentData);

            xSemaphoreTake(serialSemaphore, portMAX_DELAY);
            Serial.print("Received LoRa COMMAND: ");
            Serial.println(received);
            Serial.println("Updated to yellow_all mode");
            Serial.print("Updated mode: ");
            Serial.println(mode);
            xSemaphoreGive(serialSemaphore);
            sendAck();
          }
        }
      }
      xSemaphoreGive(spiSemaphore);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    wdt_reset();
  }
}

void TrafficLightTask(void *pvParameters) {
  (void) pvParameters;

  xSemaphoreTake(serialSemaphore, portMAX_DELAY);
  Serial.println("TrafficLightTask started");
  xSemaphoreGive(serialSemaphore);

  int cycle_time = secondary_red_time + secondary_green_time + secondary_yellow_time + all_red_time;
  int global_timer = 0;
  int last_green_time = primary_green_time; // Lưu giá trị green_time trước đó

  for (;;) {
    wdt_reset();

    // Cập nhật mode từ trafficDataQueue ở đầu mỗi chu kỳ
    TrafficData data;
    // if (xQueuePeek(trafficDataQueue, &data, 0) == pdPASS) {
    //   if (data.command_type != mode) {
    //     // Kiểm tra giá trị command_type hợp lệ
    //     if (data.command_type == "auto" || data.command_type == "manual" || data.command_type == "yellow_all") {
    //       xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    //       Serial.print("Mode updated from ");
    //       Serial.print(mode);
    //       Serial.print(" to ");
    //       Serial.println(data.command_type);
    //       xSemaphoreGive(serialSemaphore);
    //       mode = data.command_type;
    //     } else {
    //       xSemaphoreTake(serialSemaphore, portMAX_DELAY);
    //       Serial.print("Invalid command_type in queue: ");
    //       Serial.println(data.command_type);
    //       xSemaphoreGive(serialSemaphore);
    //     }
    //   }
    // }

    if (mode == "yellow_all") {
      // Tắt các đoạn LED 7 thanh
      for (int c = 0; c < 8; c++) digitalWrite(common_pins[c], HIGH);
      for (int i = 0; i < 7; i++) {
        digitalWrite(north_segments[i], LOW);
        digitalWrite(south_segments[i], LOW);
        digitalWrite(east_segments[i], LOW);
        digitalWrite(west_segments[i], LOW);
      }

      static bool yellow_on = false; // Trạng thái đèn vàng (bật/tắt)
      static unsigned long last_toggle_time = 0; // Thời điểm chuyển đổi trạng thái gần nhất
      unsigned long current_time = millis();

      // Kiểm tra thời gian để chuyển đổi trạng thái (mỗi 2 giây)
      if (current_time - last_toggle_time >= 2000) {
        if (yellow_on) {
          // Tắt đèn vàng
          clearDirectionLeds(north_red, north_yellow, north_green);
          clearDirectionLeds(south_red, south_yellow, south_green);
          clearDirectionLeds(east_red, east_yellow, east_green);
          clearDirectionLeds(west_red, west_yellow, west_green);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        } else {
          // Bật đèn vàng
          clearDirectionLeds(north_red, north_yellow, north_green);
          clearDirectionLeds(south_red, south_yellow, south_green);
          clearDirectionLeds(east_red, east_yellow, east_green);
          clearDirectionLeds(west_red, west_yellow, west_green);
          digitalWrite(north_yellow, LOW);
          digitalWrite(south_yellow, LOW);
          digitalWrite(east_yellow, LOW);
          digitalWrite(west_yellow, LOW);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        yellow_on = !yellow_on; // Chuyển đổi trạng thái
        last_toggle_time = current_time; // Cập nhật thời điểm chuyển đổi
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay ngắn để nhường CPU cho các task khác
      continue;
    }

    // Cập nhật cycle_time từ trafficDataQueue
    if (xQueueReceive(trafficDataQueue, &data, 0) == pdPASS) {
      xSemaphoreTake(timeSemaphore, portMAX_DELAY);
      cycle_time = secondary_red_time + secondary_green_time + secondary_yellow_time + all_red_time;
      xSemaphoreGive(timeSemaphore);
    }

    global_timer = 0;
    while (global_timer < cycle_time) {
      unsigned long st = millis();
      
      int north_time = 0, south_time = 0, east_time = 0, west_time = 0;
      
      xSemaphoreTake(timeSemaphore, portMAX_DELAY);
      if (global_timer < primary_red_time) {
        digitalWrite(east_red, LOW);
        digitalWrite(west_red, LOW);
        east_time = primary_red_time - global_timer;
        west_time = east_time;
      } else if (global_timer < primary_red_time + primary_green_time) {
        digitalWrite(east_green, LOW);
        digitalWrite(west_green, LOW);
        east_time = primary_red_time + primary_green_time - global_timer;
        west_time = east_time;
      } else if (global_timer < primary_red_time + primary_green_time + primary_yellow_time) {
        digitalWrite(east_yellow, LOW);
        digitalWrite(west_yellow, LOW);
        east_time = primary_red_time + primary_green_time + primary_yellow_time - global_timer;
        west_time = east_time;
      } else if (global_timer < primary_red_time + primary_green_time + primary_yellow_time + all_red_time) {
        digitalWrite(east_red, LOW);
        digitalWrite(west_red, LOW);
        east_time = primary_red_time + primary_green_time + primary_yellow_time + all_red_time - global_timer;
        west_time = east_time;
      } else {
        digitalWrite(east_red, LOW);
        digitalWrite(west_red, LOW);
        east_time = primary_red_time + primary_green_time + primary_yellow_time + all_red_time + primary_red_time - global_timer;
        west_time = east_time;
      }

      if (global_timer < secondary_green_time) {
        digitalWrite(north_green, LOW);
        digitalWrite(south_green, LOW);
        north_time = secondary_green_time - global_timer;
        south_time = north_time;
      } else if (global_timer < secondary_green_time + secondary_yellow_time) {
        digitalWrite(north_yellow, LOW);
        digitalWrite(south_yellow, LOW);
        north_time = secondary_green_time + secondary_yellow_time - global_timer;
        south_time = north_time;
      } else if (global_timer < secondary_green_time + secondary_yellow_time + all_red_time) {
        digitalWrite(north_red, LOW);
        digitalWrite(south_red, LOW);
        north_time = secondary_green_time + secondary_yellow_time + all_red_time - global_timer;
        south_time = north_time;
      } else {
        digitalWrite(north_red, LOW);
        digitalWrite(south_red, LOW);
        north_time = secondary_green_time + secondary_yellow_time + all_red_time + secondary_red_time - global_timer;
        south_time = north_time;
      }
      xSemaphoreGive(timeSemaphore);

      while (millis() - st < 1000) {
        multiplexAll(north_time, south_time, east_time, west_time);
        vTaskDelay(5 / portTICK_PERIOD_MS);
      }

      clearDirectionLeds(north_red, north_yellow, north_green);
      clearDirectionLeds(south_red, south_yellow, south_green);
      clearDirectionLeds(east_red, east_yellow, east_green);
      clearDirectionLeds(west_red, west_yellow, west_green);

      global_timer++;
    }

    // Cập nhật green_time từ greenTimeQueue sau mỗi chu kỳ
    int new_green_time = last_green_time;
    while (xQueueReceive(greenTimeQueue, &new_green_time, 0) == pdPASS) {
        // Lấy giá trị mới nhất
    }
    if (new_green_time > 0) {
        xSemaphoreTake(timeSemaphore, portMAX_DELAY);
        primary_green_time = new_green_time;
        secondary_red_time = primary_green_time + primary_yellow_time + all_red_time;
        cycle_time = secondary_red_time + secondary_green_time + secondary_yellow_time + all_red_time;
        // xQueueReset(greenTimeQueue); // Reset greenTimeQueue
        xSemaphoreGive(timeSemaphore);

        xSemaphoreTake(serialSemaphore, portMAX_DELAY);
        Serial.print("Updated green_time from queue: ");
        Serial.println(primary_green_time);
        Serial.println("greenTimeQueue reset");
        xSemaphoreGive(serialSemaphore);

        last_green_time = new_green_time; // Cập nhật giá trị trước đó
    }
  }
}

void multiplexAll(int north, int south, int east, int west) {
  int nums[4] = {north, south, east, west};
  unsigned long startTime = micros();
  const unsigned long maxScanTime = 5000;
  for (int scan = 0; scan < 5 && (micros() - startTime) < maxScanTime; scan++) {
    for (int dir = 0; dir < 4; dir++) {
      int tens = nums[dir] / 10, units = nums[dir] % 10;
      bool show_tens = (nums[dir] >= 10);
      for (int part = 0; part < 2; part++) {
        for (int c = 0; c < 8; c++) digitalWrite(common_pins[c], HIGH);
        const byte* segs = (dir == 0 ? north_segments : dir == 1 ? south_segments :
                           dir == 2 ? east_segments : west_segments);
        for (int i = 0; i < 7; i++) digitalWrite(segs[i], LOW);
        if (part == 0 && !show_tens) continue;

        int value = (part == 0 ? tens : units);
        int common_idx = dir * 2 + part;
        for (int i = 0; i < 7; i++) {
          digitalWrite(segs[i], bitRead(numbers[value], i));
        }
        digitalWrite(common_pins[common_idx], LOW);
        delayMicroseconds(500); // Giảm từ 1000µs xuống 500µs
      }
    }
  }
}