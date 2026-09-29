#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

#define SD_SCK  7
#define SD_MISO 8
#define SD_MOSI 9
#define SD_CS   21

WiFiServer server(80);
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
File dataFile;

// --- QUEUE ARCHITECTURE ---
// Structure to hold exactly one row of CSV data
struct IMUData {
  float elapsedTime;
  float d[12]; // Array holding Ax1 through Gz2
};

// FreeRTOS Handles
QueueHandle_t dataQueue;
TaskHandle_t ServerTask;
TaskHandle_t SDTask;

// --- STATE VARIABLES ---
volatile bool isLogging = false;
volatile bool firstReading = false;
unsigned long lastLogTimeMicros = 0;
unsigned long loggingStartTimeMicros = 0;
unsigned long logIntervalMicros = 10000; // Default 10ms (100 Hz)

// --- CORE 0: SD WRITE TASK ---
// Dedicated task that waits for data in the Queue and writes to the SD card
void sdWriteTask(void * pvParameters) {
  IMUData item;
  for(;;) {
    if (isLogging) {
      // xQueueReceive pauses the task until data arrives, consuming 0% CPU while waiting
      if (xQueueReceive(dataQueue, &item, portMAX_DELAY) == pdPASS) {
        if (dataFile) {
          dataFile.print(item.elapsedTime, 3);
          for (int i = 0; i < 12; i++) {
            dataFile.print(",");
            dataFile.print(item.d[i], 3);
          }
          dataFile.println();
        }
      }
    } else {
      vTaskDelay(50 / portTICK_PERIOD_MS); // Sleep if not logging
    }
  }
}

// --- CORE 0: WEB SERVER TASK ---
void serverTask(void * pvParameters) {
  for(;;) {
    WiFiClient client = server.available();  
    
    if (client) {                     
      Serial.println("New Client.");  
      String currentLine = "";
      String requestHeader = ""; 
      
      while (client.connected()) {    
        if (client.available()) {     
          char c = client.read();     
          
          if (c == '\n') {            
            if (currentLine.length() == 0) {
              
              if (requestHeader.indexOf("GET /start") >= 0) {
                // Parse variable sampling rate from the URL
                int rateIndex = requestHeader.indexOf("rate=");
                if (rateIndex > 0) {
                  int hz = requestHeader.substring(rateIndex + 5).toInt();
                  if (hz > 0 && hz <= 500) {
                    // NOTE ON MILLISECOND ROUNDING: 
                    // The millis() timer uses whole integers. If you request a frequency 
                    // that does not divide cleanly into 1000, it truncates.
                    // Example: Requesting 120 Hz yields 1000 / 120 = 8.33 ms. 
                    // This truncates to 8 ms, resulting in an actual sampling rate of 125 Hz.
                    logIntervalMicros = 1000000 / hz;
                  }
                } else {
                  logIntervalMicros = 10000;   
                }

                // Prepare file and queue
                dataFile = SD.open("/data.csv", FILE_WRITE);
                if (dataFile) {
                  dataFile.println("T_s,Ax1,Ay1,Az1,Gx1,Gy1,Gz1,Ax2,Ay2,Az2,Gx2,Gy2,Gz2");
                }
                
                xQueueReset(dataQueue); // Wipe any old data from memory
                firstReading = true;
                isLogging = true; 
                
                Serial.printf("Logging initiated via Web at %lu Hz\n", 1000 / logIntervalMicros);
                client.println("HTTP/1.1 303 See Other");
                client.println("Location: /");
                client.println();
                break;
                
              } 
              else if (requestHeader.indexOf("GET /stop") >= 0) {
                isLogging = false;
                
                // Allow the SD task a brief moment to finish writing the remaining queue
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                if (dataFile) {
                  dataFile.close();
                }
                Serial.println("Logging stopped via Web");
                client.println("HTTP/1.1 303 See Other");
                client.println("Location: /");
                client.println();
                break;
                
              } 
              else if (requestHeader.indexOf("GET /download") >= 0) {
                File downloadFile = SD.open("/data.csv", FILE_READ);
                if (downloadFile) {
                  client.println("HTTP/1.1 200 OK");
                  client.println("Content-Type: text/csv");
                  client.println("Content-Disposition: attachment; filename=\"data.csv\"");
                  client.println("Connection: close");
                  client.println();
                  
                  uint8_t buffer[512];
                  size_t bytesRead;
                  while ((bytesRead = downloadFile.read(buffer, sizeof(buffer))) > 0) {
                    client.write(buffer, bytesRead);
                  }
                  downloadFile.close();
                  Serial.println("File downloaded successfully.");
                } else {
                  client.println("HTTP/1.1 404 Not Found");
                  client.println("Content-Type: text/html");
                  client.println();
                  client.println("<h1>File not found!</h1><p>Start logging first.</p>");
                  client.println("<br><a href=\"/\">Return Home</a>");
                }
                break;
                
              } 
              else {
                // Interactive Web Interface
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();
                client.print("<h1>Dual MPU-6050 Data Logger</h1>");
                
                client.print("<p>Current Status: <strong>");
                if (isLogging) {
                  client.print("<span style=\"color:red;\">LOGGING at ");
                  client.print(1000 / logIntervalMicros);
                  client.print(" Hz</span>");
                } else {
                  client.print("<span style=\"color:green;\">STOPPED</span>");
                }
                client.print("</strong></p>");
                
                client.print("<h3>Start Logging</h3>");
                client.print("<form action=\"/start\" method=\"GET\">");
                client.print("<label for=\"rate\">Sampling Rate (1 - 500 Hz): </label>");
                client.print("<input type=\"number\" id=\"rate\" name=\"rate\" min=\"1\" max=\"500\" value=\"100\"> ");
                client.print("<input type=\"submit\" value=\"START\">");
                client.print("</form><br><br>");
                
                client.print("<a href=\"/stop\"><button style=\"background-color:red;color:white;padding:10px;\">STOP LOGGING</button></a><br><br>");
                client.print("<hr>");
                client.print("<h3>Data Management</h3>");
                client.print("<a href=\"/download\"><button style=\"padding:10px;\">DOWNLOAD data.csv</button></a><br>");
                client.println();
                break; 
              }
              
            } else {  
              if (requestHeader.length() == 0 && currentLine.startsWith("GET ")) {
                  requestHeader = currentLine;
              }
              currentLine = "";
            }
          } else if (c != '\r') {  
            currentLine += c;      
          }
        }
      }
      client.stop();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
    delay(10);

  Wire.begin(5, 6);
  Wire.setClock(400000); // 100 kHz I2C to maintain physical signal integrity
  
  if (!mpu1.begin(0x68)) Serial.println("Failed to find MPU6050 #1 (0x68)");
  if (!mpu2.begin(0x69)) Serial.println("Failed to find MPU6050 #2 (0x69)");
  
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu2.setFilterBandwidth(MPU6050_BAND_260_HZ);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI)) Serial.println("SD Card Mount Failed");

  Serial.println("\nCreating Access Point...");
  WiFi.softAP("ESP32_Logger", "12345678"); 
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Connect to 'ESP32_Logger', IP: ");
  Serial.println(IP); 

  server.begin();

  // Create a queue capable of holding 200 data rows (protects against up to 2 full seconds of SD delays at 100Hz)
  dataQueue = xQueueCreate(200, sizeof(IMUData));

  // Launch tasks on Core 0 (Higher priority for SD card writing to prevent dropped packets)
  xTaskCreatePinnedToCore(serverTask, "ServerTask", 10000, NULL, 1, &ServerTask, 0);
  xTaskCreatePinnedToCore(sdWriteTask, "SDWriteTask", 10000, NULL, 2, &SDTask, 0);
}

// --- CORE 1: DEDICATED IMU TIMER ---
void loop() {
  if (isLogging) {
    unsigned long currentMicros = micros();
    
    // Fixes the rapid-fire bug by instantly syncing the timer on the very first loop
    if (firstReading) {
      loggingStartTimeMicros = currentMicros; 
      lastLogTimeMicros = currentMicros - logIntervalMicros; // Forces immediate first read
      firstReading = false;             
    }
    
    // Strict frequency check
    if (currentMicros - lastLogTimeMicros >= logIntervalMicros) {
      lastLogTimeMicros += logIntervalMicros; 
      
      sensors_event_t a1, g1, temp1, a2, g2, temp2;
      mpu1.getEvent(&a1, &g1, &temp1);
      mpu2.getEvent(&a2, &g2, &temp2);
      
      IMUData newData;
      newData.elapsedTime = (currentMicros - loggingStartTimeMicros) / 1000000.0;
      
      newData.d[0] = a1.acceleration.x;
      newData.d[1] = a1.acceleration.y;
      newData.d[2] = a1.acceleration.z;
      newData.d[3] = g1.gyro.x;
      newData.d[4] = g1.gyro.y;
      newData.d[5] = g1.gyro.z;
      
      newData.d[6] = a2.acceleration.x;
      newData.d[7] = a2.acceleration.y;
      newData.d[8] = a2.acceleration.z;
      newData.d[9] = g2.gyro.x;
      newData.d[10] = g2.gyro.y;
      newData.d[11] = g2.gyro.z;
      
      // Push data into the queue (The '0' means never pause Core 1, even if queue is full)
      xQueueSend(dataQueue, &newData, 0);
    }
  }
}

