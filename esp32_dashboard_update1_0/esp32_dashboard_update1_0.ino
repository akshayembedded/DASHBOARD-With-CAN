#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <BME280I2C.h>
#include <Wire.h>
//#include <FreeRTOS.h>  // For mutex

BME280I2C bme;
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;  // Queue size for CAN reception
int state = 1;  // Indicator state (for blinking)
unsigned long previousMillis = 0;
const int interval = 500;  // CAN message interval

// Pin Definitions
const int leftButtonPin = 12;
const int rightButtonPin = 14;
const int hazardButtonPin = 25;
const int leftLED = 26;
const int rightLED = 27;
const int indicatorBuzzer = 23;

bool leftIndicatorState = false;   // Left indicator state (on/off)
bool rightIndicatorState = false;  // Right indicator state (on/off)
bool hazardIndicatorState = false;  
unsigned long leftLastBlinkTime = 0;  // Last blink time for left indicator
unsigned long rightLastBlinkTime = 0; // Last blink time for right indicator
int leftBlinkDelay = 500;     // Delay for left indicator blink
int rightBlinkDelay = 500;    // Delay for right indicator blink

// Mutex for CAN access
SemaphoreHandle_t canMutex;

// Button debounce variables
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 1500;  // Debounce delay in milliseconds

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  // CAN configuration
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  Wire.begin();
  while (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  Serial.println("BME280 sensor initialized.");

  // Pin modes for buttons and LEDs
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
   pinMode(hazardButtonPin, INPUT_PULLUP);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(indicatorBuzzer, OUTPUT);

  // Create a mutex for CAN communication
  canMutex = xSemaphoreCreateMutex();
  if (canMutex == NULL) {
    Serial.println("Failed to create mutex.");
    while (1);
  }

  // Create tasks
 xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(indicatorTask, "Indicator Task", 4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(canCommunicationTask, "CAN Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ledBuzzerTask, "LED and Buzzer Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS will handle the tasks, so the main loop is empty
}

// Sensor Task: Reads data from BME280 sensor and sends it over CAN
void sensorTask(void *parameter) {
  CAN_frame_t tx_frame;
  float temp, hum, pres;
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  while (true) {
    bme.read(pres, temp, hum, tempUnit, presUnit);

    // Prepare temperature and humidity data
    int16_t tempInteger = (int16_t)temp;
    int16_t tempFraction = (int16_t)((temp - tempInteger) * 1000);
    int16_t humInteger = (int16_t)hum;
    int16_t humFraction = (int16_t)((hum - humInteger) * 1000);
    int16_t presInteger = (int16_t)pres;
    int16_t presFraction = (int16_t)((pres - presInteger) * 1000);

    // Lock the mutex before sending CAN message
    if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
      // CAN Message for temperature and humidity
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = 0x010;
      tx_frame.FIR.B.DLC = 8;
      tx_frame.data.u8[0] = (tempInteger >> 8) & 0xFF;
      tx_frame.data.u8[1] = tempInteger & 0xFF;
      tx_frame.data.u8[2] = (tempFraction >> 8) & 0xFF;
      tx_frame.data.u8[3] = tempFraction & 0xFF;
      tx_frame.data.u8[4] = (humInteger >> 8) & 0xFF;
      tx_frame.data.u8[5] = humInteger & 0xFF;
      tx_frame.data.u8[6] = (humFraction >> 8) & 0xFF;
      tx_frame.data.u8[7] = humFraction & 0xFF;
      ESP32Can.CANWriteFrame(&tx_frame);

      // CAN Message for pressure
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = 0x011;
      tx_frame.FIR.B.DLC = 4;
      tx_frame.data.u8[0] = (presInteger >> 8) & 0xFF;
      tx_frame.data.u8[1] = presInteger & 0xFF;
      tx_frame.data.u8[2] = (presFraction >> 8) & 0xFF;
      tx_frame.data.u8[3] = presFraction & 0xFF;
      ESP32Can.CANWriteFrame(&tx_frame);

      // Unlock the mutex after CAN communication
      xSemaphoreGive(canMutex);
    }

    // Delay for 1 second before the next reading
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Indicator Task: Handles the left and right indicator button presses
void indicatorTask(void *parameter) {
  CAN_frame_t tx_frame;
  unsigned long currentMillis;
  
  while (true) {
    currentMillis = millis();
    
    // Check if left button is pressed
    if (!digitalRead(leftButtonPin) && (currentMillis - lastButtonPressTime > debounceDelay)) {
      leftIndicatorState = !leftIndicatorState;  // Toggle the left indicator state
      rightIndicatorState=hazardIndicatorState=0;
      lastButtonPressTime = currentMillis;  // Update the last button press time
      Serial.print(leftIndicatorState);
      Serial.println("Button Pressed");
      // Lock the mutex before sending CAN message
      if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
        // Send CAN message for left indicator
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x03;
        tx_frame.FIR.B.DLC = 1;
        tx_frame.data.u8[0] = leftIndicatorState ? 1 : 0;  // 1 for left on, 0 for left off
        ESP32Can.CANWriteFrame(&tx_frame);

        // Unlock the mutex after CAN communication
        xSemaphoreGive(canMutex);
      }
    }

    // Check if right button is pressed
    if (!digitalRead(rightButtonPin) && (currentMillis - lastButtonPressTime > debounceDelay)) {
      rightIndicatorState = !rightIndicatorState;  // Toggle the right indicator state
      leftIndicatorState=hazardIndicatorState=0;
      lastButtonPressTime = currentMillis;  // Update the last button press time

      // Lock the mutex before sending CAN message
      if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
        // Send CAN message for right indicator
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x03;
        tx_frame.FIR.B.DLC = 1;
        tx_frame.data.u8[0] = rightIndicatorState ? 2 : 0;  // 2 for right on, 0 for right off
        ESP32Can.CANWriteFrame(&tx_frame);

        // Unlock the mutex after CAN communication
        xSemaphoreGive(canMutex);
      }
    }
     if (!digitalRead(hazardButtonPin) && (currentMillis - lastButtonPressTime > debounceDelay)) {
       // Toggle the left indicator state
       hazardIndicatorState=!hazardIndicatorState;
      rightIndicatorState = hazardIndicatorState;
      leftIndicatorState = hazardIndicatorState;
     // lastButtonPressTime = currentMillis;  // Update the last button press time
      Serial.print(leftIndicatorState);
      Serial.println("Button Pressed");
      // Lock the mutex before sending CAN message
      if (xSemaphoreTake(canMutex, portMAX_DELAY) == pdTRUE) {
        // Send CAN message for left indicator
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x03;
        tx_frame.FIR.B.DLC = 1;
        tx_frame.data.u8[0] = hazardIndicatorState ? 3 : 0;  // 1 for left on, 0 for left off
        ESP32Can.CANWriteFrame(&tx_frame);
        
        // Unlock the mutex after CAN communication
        xSemaphoreGive(canMutex);
      }
     }
    // Delay for task scheduling
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// LED and Buzzer Task: Controls LED blinking and buzzer synchronization
void ledBuzzerTask(void *parameter) {
  unsigned long currentMillis;
  
  while (true) {
    
    // Blink the left indicator if it's on
   if (leftIndicatorState) {
     
        digitalWrite(leftLED, !digitalRead(leftLED));  // Toggle the LED
        digitalWrite(indicatorBuzzer, 1);  // Sync buzzer with LED
            vTaskDelay(1+3*digitalRead(rightLED));
            digitalWrite(indicatorBuzzer, 0);  // Sync buzzer with LED
      
     } 
    else {
       digitalWrite(leftLED, LOW);  // Turn off the left LED
      digitalWrite(indicatorBuzzer, LOW);  // Turn off the buzzer
     }

    // Blink the right indicator if it's on
    if (rightIndicatorState) {
       rightLastBlinkTime = currentMillis;
        digitalWrite(rightLED, !digitalRead(rightLED));  // Toggle the LED
      
        digitalWrite(indicatorBuzzer, 1);  // Sync buzzer with LED
            vTaskDelay(1+3*digitalRead(rightLED));
            digitalWrite(indicatorBuzzer, 0); 
     } 
    else {
      digitalWrite(rightLED, LOW);  // Turn off the right LED
      digitalWrite(indicatorBuzzer, LOW);  // Turn off the buzzer
    }

    // Delay for task scheduling
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// CAN Communication Task: Receives and processes CAN messages
void canCommunicationTask(void *parameter) {
  CAN_frame_t rx_frame;
  
  while (true) {
    // Receive CAN messages
    /*if (ESP32Can.CANReadFrame(&rx_frame) == ESP_OK) {
      if (rx_frame.MsgID == 0x03) {
        // Process CAN message for indicator states
        if (rx_frame.data.u8[0] == 1) {
          leftIndicatorState = true;
        } else if (rx_frame.data.u8[0] == 2) {
          rightIndicatorState = true;
        } else {
          leftIndicatorState = false;
          rightIndicatorState = false;
        }
      }
    }
*/
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
