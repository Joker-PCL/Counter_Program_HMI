#include <Arduino.h>
#include <ModbusSlaveSerial.h>

#define USE_HARDWARE_SERIAL

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

  mb.addCoil(0);  //Coil0
  mb.addCoil(1);  //Coil1
  mb.addCoil(2);  //Coil2

  mb.addIreg(0);  //Level
  mb.addIreg(1);  //Distance
  mb.addIreg(2);  //Flow rate

  mb.addHreg(0);
  mb.addHreg(1);
  mb.addHreg(2);
  mb.addHreg(3);
  mb.addHreg(4);

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
  Serial.begin(9600);
  initModbusSlave(&Serial2, 9600, 16, 17, 5);  //พารามิเตอร์ (Serial, Baudrate, rxPin, txPin, Slave address)

  /* Modbus Slave Connection Task - Run in CPU Core1 */
  xTaskCreatePinnedToCore(modbusSlaveTask, "modbusSlaveTask", 3000, (void*)NULL, 2, &Task1, 1);
}

unsigned long count = 0;
unsigned long previous_time = 0;

void loop() {
  if(millis() - previous_time >= 500) {
    count++;
    mb.Hreg(0, count); 
    previous_time = millis();
  }

  delay(50);
}