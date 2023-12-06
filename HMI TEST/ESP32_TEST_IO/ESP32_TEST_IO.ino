#include <Arduino.h>
#include <ModbusSlaveSerial.h>

#define USE_HARDWARE_SERIAL

#define DEVICE_CNT 8
const int BUTTON_ADDRESS[DEVICE_CNT] = { 600, 601, 602, 603, 604, 605, 606, 608 };    // กำหนด address relay output
const int TOTAL_ADDRESS = 601;                                                       // กำหนด address total

const int SENSOR_PINS[DEVICE_CNT] = { 35, 34, 39, 36, 32, 33, 25, 26 };  // set sensorpins && EEPROM address on,off couter
const int RELAY_PINS[DEVICE_CNT] = { 17, 16, 2, 4, 13, 12, 14, 27 };          // set relaypins
bool RELAY_STATE[DEVICE_CNT] = { false };                                // set relay_state

/*---------------------------------------------------------------------
          define Modbus Slave Serail Object
---------------------------------------------------------------------*/
TaskHandle_t Task1;
const TickType_t xDelay20ms = pdMS_TO_TICKS(20);

ModbusSlaveSerial mb;

/*---------------------------------------------------------------------
                        This part for Web monitor
---------------------------------------------------------------------*/
void initModbusSlave(HardwareSerial* _Serial, long _baud, uint8_t _rxPin, uint8_t _txPin, uint8_t _slave) {
  Serial.println("-----------------------.");
  Serial.println("Init Modbus Slave.");
  mb.config(_Serial, _baud, _rxPin, _txPin);
  mb.setSlaveId(_slave);

  for (int i = 0; i < DEVICE_CNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
    mb.addCoil(BUTTON_ADDRESS[i], 0);
  }

  mb.addHreg(TOTAL_ADDRESS);

  Serial.println("Done Init Modbus Slave.");
  Serial.println("-----------------------.");
}

void modbusSlaveTask(void* pvParam) {
  for (;;) {
    mb.task();
    vTaskDelay(xDelay20ms);
  }

  vTaskDelete(NULL);
}

void setup() {
  initModbusSlave(&Serial, 9600, 3, 1, 1);  //พารามิเตอร์ (Serial, Baudrate, rxPin, txPin, Slave address)

  /* Modbus Slave Connection Task - Run in CPU Core1 */
  xTaskCreatePinnedToCore(modbusSlaveTask, "modbusSlaveTask", 3000, (void*)NULL, 2, &Task1, 1);
}

unsigned long count = 0;
unsigned long previous_time = 0;

void loop() {
  if(millis() - previous_time >= 500) {
    count++;
    mb.Hreg(TOTAL_ADDRESS, count);  //นำค่าจาก Inputregistor 0 มาใส่ใน Holding registor 0
    previous_time = millis();
  }

  for (int i = 0; i < DEVICE_CNT; i++) {
    digitalWrite(RELAY_PINS[i], mb.Coil(BUTTON_ADDRESS[i]));
  }
  
}