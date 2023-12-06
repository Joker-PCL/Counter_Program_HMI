//=============================Include Header Files===========================//
// ESP32_Control v 1.2.8.1 https://github.com/suratin27/ESP32_Control
// ESP32 CONTROL https://www.arduinolibraries.info/libraries/esp32-control
// ESP32 CONTROL==>> ModbusSlaveSerial

#include <Arduino.h>
#include <ModbusSlaveSerial.h>
#include <Ticker.h>

#define USE_HARDWARE_SERIAL

//============================= Define Modbus Slave Serail Object =================================//
TaskHandle_t Task1;
TaskHandle_t Task2;

const TickType_t xDelayMs = pdMS_TO_TICKS(20);
ModbusSlaveSerial mb;


//============================= Set modbus address =================================//
#define DEVICE_CNT 8
const int RELAY_ADDRESS[DEVICE_CNT] = { 600, 601, 602, 603, 604, 605, 606, 608 };    // กำหนด address relay output
const int COUNTER_ADDRESS[DEVICE_CNT] = { 600, 601, 602, 603, 604, 605, 606, 608 };  // กำหนด address counter
const int ON_ADDRESS[DEVICE_CNT] = { 640, 641, 642, 643, 644, 645, 646, 648 };       // กำหนด address on/off counter
const int RESET_ADDRESS[DEVICE_CNT] = { 720, 721, 722, 723, 724, 725, 726, 728 };    // กำหนด address reset counter
const int SETCOUNT_ADDRESS = 700;                                                    // กำหนด address set จำนวนที่จะนับ
const int SETDELAY_ADDRESS = 701;                                                    // กำหนด address set หน่วงเวลา relay output
const int RESETALL_ADDRESS = 700;                                                    // กำหนด address reset all counter
const int START_ADDRESS = 701;                                                       // กำหนด address total

//============================= Counter Ticker =================================//
Ticker Counter;  // ticker with function Counter

int RELAY_TIMER;   // save RELAY_TIMER to EEPROM
int SENSOR_TIMER;  // save SENSOR_TIMER to EEPROM
int COUNT_UNIT;    // save COUNT_UNIT to EEPROM
bool START_STATE;  // save START_STATE to EEPROM


long unsigned int tick = 0;
const int SENSOR_PINS[DEVICE_CNT] = { 35, 34, 39, 36, 32, 33, 25, 26 };  // set sensorpins && EEPROM address on,off couter
const int RELAY_PINS[DEVICE_CNT] = { 17, 16, 2, 4, 13, 12, 14, 27 };     // set relaypins
bool RELAY_STATE[DEVICE_CNT] = { false };                                // set relay_state
unsigned int ITEM_COUNTER[DEVICE_CNT] = { 0 };

unsigned long currentMillis = millis();
long int previousMillis[DEVICE_CNT] = { 0 };

// Variable for Counter
int previousstate[DEVICE_CNT] = { 0 };
int currentstate[DEVICE_CNT] = { 0 };

void setup() {
  mb.config(&Serial, 9600, 3, 1);  // (Serial, Baudrate, rxPin, txPin)
  mb.setSlaveId(1);                // Slave address

  for (int i = 0; i < DEVICE_CNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
    mb.addHreg(COUNTER_ADDRESS[i]);
    mb.addCoil(ON_ADDRESS[i]);
    mb.addCoil(RELAY_ADDRESS[i]);
    mb.addCoil(RESET_ADDRESS[i]);
  }

  mb.addHreg(SETCOUNT_ADDRESS);
  mb.addHreg(SETDELAY_ADDRESS);
  mb.addCoil(RESETALL_ADDRESS);
  mb.addCoil(START_ADDRESS);

  /* Modbus Slave Connection Task - Run in CPU Core1 */
  xTaskCreatePinnedToCore(modbusSlaveTask, "modbusSlaveTask", 3000, (void*)NULL, 9, &Task1, 1);
  xTaskCreatePinnedToCore(completeCount, "completeCount", 3000, (void*)NULL, 8, &Task2, 1);
  Counter.attach_ms(5, CountItem);  // sample every 100ms, 0.5sec
}

void loop() {
  setting();
  relayTest();
  countReset();
  delay(50);
}

void modbusSlaveTask(void* pvParam) {
  for (;;) {
    mb.task();
    vTaskDelay(xDelayMs);
  }

  vTaskDelete(NULL);
}

// setting
void setting() {
  START_STATE = mb.Coil(START_ADDRESS);
  COUNT_UNIT = mb.Hreg(SETCOUNT_ADDRESS);
  RELAY_TIMER = mb.Hreg(SETDELAY_ADDRESS);
}

// Test relay
void relayTest() {
  if (!START_STATE) {
    for (int i = 0; i < DEVICE_CNT; i++) {
      // Attach relay pins to LAMP1_COIL register
      digitalWrite(RELAY_PINS[i], mb.Coil(RELAY_ADDRESS[i]));
    }
  }
}

// function Counter
void CountItem() {
  if (START_STATE) {
    for (int i = 0; i < DEVICE_CNT; i++) {
      if (mb.Coil(ON_ADDRESS[i])) {
        currentstate[i] = digitalRead(SENSOR_PINS[i]);

        if (currentstate[i] == 0 && previousstate[i] == 1) {
          ITEM_COUNTER[i]++;
          mb.Hreg(COUNTER_ADDRESS[i], ITEM_COUNTER[i]);
        }

        if (ITEM_COUNTER[i] >= COUNT_UNIT) {
          RELAY_STATE[i] = true;
          ITEM_COUNTER[i] = 0;
          mb.Hreg(COUNTER_ADDRESS[i], ITEM_COUNTER[i]);
        }

        previousstate[i] = currentstate[i];
      } else {
        RELAY_STATE[i] = false;
        ITEM_COUNTER[i] = 0;
        mb.Hreg(COUNTER_ADDRESS[i], ITEM_COUNTER[i]);
      }
    }
  }
}

// function CompleteCount
void completeCount(void* pvParam) {
  for (;;) {
    if (mb.Coil(START_ADDRESS)) {
      //  Capture Timing
      currentMillis = millis();

      // Check CompleteCount, relay activate
      for (int i = 0; i < DEVICE_CNT; i++) {
        if (RELAY_STATE[i] == true) {
          digitalWrite(RELAY_PINS[i], HIGH);
          mb.Coil(RELAY_ADDRESS[i], 1);
          RELAY_STATE[i] = false;
          previousMillis[i] = currentMillis;
        }
      }

      for (int x = 0; x < DEVICE_CNT; x++) {
        if (currentMillis - previousMillis[x] >= RELAY_TIMER) {
          digitalWrite(RELAY_PINS[x], LOW);
          mb.Coil(RELAY_ADDRESS[x], 0);
          previousMillis[x] = currentMillis;
        }
      }
    }

    vTaskDelay(xDelayMs);
  }

  vTaskDelete(NULL);
}

void countReset() {
  for (int i = 0; i < DEVICE_CNT; i++) {
    if (mb.Coil(RESET_ADDRESS[i])) {
      ITEM_COUNTER[i] = 0;
      mb.Hreg(COUNTER_ADDRESS[i], 0);
    } else {
      mb.Coil(RESET_ADDRESS[i], 0);
    }
  }

  if (mb.Coil(RESETALL_ADDRESS)) {
    for (int x = 0; x < DEVICE_CNT; x++) {
      ITEM_COUNTER[x] = 0;
      mb.Hreg(COUNTER_ADDRESS[x], 0);
    }
  }
}
