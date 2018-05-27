/*
 * Copyright (c) 2016 RedBear
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/**
 * @note This demo is Nordic HRM example.
 *       You could use nRF toolbox tool to test it.
 */
#define SENSOR_VCNL4010 0
#define SENSOR_TOF 1
#define SENSOR_TEMPERATURE 1

#include <nRF5x_BLE_API.h>
#include <Wire.h>
#if SENSOR_VCNL4010
#include "Adafruit_VCNL4010.h"
#endif
#if SENSOR_TOF
// https://github.com/pololu/vl53l0x-arduino
#include <VL53L0X.h>
#endif
#if SENSOR_TEMPERATURE
#include <FaBoTemperature_ADT7410.h>
#endif

const uint8_t PIN_LIGHT1 = A5;
const uint8_t PIN_LIGHT2 = A4;

#if SENSOR_TOF
const uint8_t XSHUT_PIN = D6;
const uint8_t TOF_FRONT_NEWADDR = 42; // TOF_UP = 41 (default)
#endif

#if SENSOR_VCNL4010
Adafruit_VCNL4010 vcnl;
#endif
#if SENSOR_TOF
VL53L0X tof_up;
VL53L0X tof_front;
#endif
#if SENSOR_TEMPERATURE
FaBoTemperature adt7410;
#endif

#define DEVICE_NAME       "Nordic_HRM"

BLE                       ble;
Ticker                    ticker_task1;
static uint32_t prevMillis = 0;

static uint8_t hrmCounter     = 100;
static uint8_t bpm[2]         = {0x00, hrmCounter};
static const uint8_t location = 0x03;

static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

// Create characteristic and service
GattCharacteristic   hrmRate(GattCharacteristic::UUID_HEART_RATE_MEASUREMENT_CHAR, bpm, sizeof(bpm), sizeof(bpm), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic   hrmLocation(GattCharacteristic::UUID_BODY_SENSOR_LOCATION_CHAR,(uint8_t *)&location, sizeof(location), sizeof(location),GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);
GattCharacteristic   *hrmChars[] = {&hrmRate, &hrmLocation, };
GattService          hrmService(GattService::UUID_HEART_RATE_SERVICE, hrmChars, sizeof(hrmChars) / sizeof(GattCharacteristic *));


void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected!");
  Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

void periodicCallback() {
  if (ble.getGapState().connected) {
    // Update the HRM measurement
    // First byte = 8-bit values, no extra info, Second byte = uint8_t HRM value
    // See --> https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml
    /*
    hrmCounter++;
    if (hrmCounter == 175)
        hrmCounter = 100;
    */

    bpm[1] = hrmCounter;
    ble.updateCharacteristicValue(hrmRate.getValueAttribute().getHandle(), bpm, sizeof(bpm));
  }
}

void setup() {
#if SENSOR_TOF
  // https://forum.pololu.com/t/vl53l0x-maximum-sensors-on-i2c-arduino-bus/10845/7
  pinMode(XSHUT_PIN, OUTPUT); // LOW: shutdown tof_up
#endif

  Serial.begin(9600);
  Serial.println("Nordic_HRM Demo ");
  Wire.begin();

#if SENSOR_VCNL4010
  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  // disable proximity sensing. use ambient sensing only
  vcnl.setLEDcurrent(0);
  vcnl.setFrequency(VCNL4010_1_95);
#endif

#if SENSOR_TOF
  // change address to use multiple VL53L0X
  tof_front.setAddress(TOF_FRONT_NEWADDR);
  pinMode(XSHUT_PIN, INPUT); // HIGH: boot tof_up. default addr
  delay(50);
  tof_up.init();
  tof_up.setTimeout(300);
  tof_up.setMeasurementTimingBudget(20000);
  tof_front.init();
  tof_front.setTimeout(300);
  tof_front.setMeasurementTimingBudget(20000);
#endif

#if SENSOR_TEMPERATURE
  adt7410.begin();
#endif

  ticker_task1.attach(periodicCallback, 1);
  ble.init();
  ble.onDisconnection(disconnectionCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t*)uuid16_list, sizeof(uuid16_list));
  ble.accumulateAdvertisingPayload(GapAdvertisingData::HEART_RATE_SENSOR_HEART_RATE_BELT);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  ble.addService(hrmService);
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  ble.startAdvertising();
}

void loop() {
  ble.waitForEvent();

  if (millis() - prevMillis > 1000) {
    prevMillis = millis();

    int light1 = analogRead(PIN_LIGHT1);
    int light2 = analogRead(PIN_LIGHT2);
    Serial.print("sensing ms: "); Serial.println(millis() - prevMillis); // ex. 11ms
    Serial.print("Light1: "); Serial.println(light1);
    Serial.print("Light2: "); Serial.println(light2);

#if SENSOR_VCNL4010
    uint16_t ambient = vcnl.readAmbient();
    //Serial.print("sensing ms: "); Serial.println(millis() - prevMillis); // ex. 117 ms

    Serial.print("Ambient: "); Serial.println(ambient);
    //Serial.print("Proximity: "); Serial.println(vcnl.readProximity());
    // 75:65535 = hrm:ambient
    hrmCounter = ambient * 75.0 / 65535.0 + 100;
    //Serial.print("hrm value: "); Serial.println(hrmCounter);
#endif

#if SENSOR_TOF
    uint16_t mm_up = tof_up.readRangeSingleMillimeters();
    if (!tof_up.timeoutOccurred()) {
      Serial.print("up ToF mm: "); Serial.println(mm_up);
      //hrmCounter = mm_up * 75.0 / 8192.0 + 100;
      //Serial.print("hrm value: "); Serial.println(hrmCounter);
    }

    uint16_t mm_front = tof_front.readRangeSingleMillimeters();
    if (!tof_front.timeoutOccurred()) {
      const uint16_t FRONT_MIN = 600;
      if (mm_front < FRONT_MIN) {
        Serial.print("XXX ");
      }
      Serial.print("front ToF mm: "); Serial.println(mm_front);
    }
#endif

#if SENSOR_TEMPERATURE
    float temp = adt7410.readTemperature();
    Serial.print("temperature: "); Serial.println(temp, 1);
    // 75:255 = hrm:(temp+55) // temp:[-55, 150]
    hrmCounter = (temp + 55) * 75.0 / 255.0 + 100;
    Serial.print("hrm value: "); Serial.println(hrmCounter);
#endif
  }
}

