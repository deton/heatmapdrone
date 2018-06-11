#include <nRF5x_BLE_API.h>
#include <VL53L0X.h> // https://github.com/pololu/vl53l0x-arduino
#include <FaBoTemperature_ADT7410.h>
#include "parrotmambo.h"

/// pilot plan
//
// floor model:
//
//      [north]
//  +-------------+  wall
//  |             |
//  | == == == == |  ceiling light line
//  |             |
//  | == == == == |
//  |             |
//  +-------------+  wall
//      [south]
//
// pilot state:
//  +-------------+
//  |           ^ | PS_E2W
//  | == ==>== == | PS_EAST
//  | ^           | PS_W2E
//  | == ==<== == | PS_WEST
//  |             |
//  +-------------+
//
enum PILOT_STATE {
  PS_INIT, PS_WEST, PS_W2E, PS_EAST, PS_E2W, PS_LAND
};
volatile enum PILOT_STATE pilotState = PS_INIT;

// substate for finding ceiling light on PS_W2E/E2W state
enum FINDLIGHT_STATE {
  FS_LEAVING_LIGHT, FS_APPROACHING_LIGHT, FS_ONLIGHT_WAIT, FS_ONLIGHT
};
volatile enum FINDLIGHT_STATE findlightState = FS_ONLIGHT;
volatile uint8_t prevDarker = 0;

const int AMBIENT_DARK_THRESHOLD = 115; // XXX
const int AMBIENT_LIGHT_THRESHOLD = 200; // XXX

const uint16_t PILOT_INTERVAL = 400; // 0: pilot on SENSING_INTERVAL [ms]
const int8_t FORWARD_VALUE = 30; // [-100,100]
const int8_t UP_VALUE = 40; // [-100,100]
const int8_t RIGHT_VALUE = 40; // [-100,100]
const uint32_t NEARWALL_WAIT = 1500; // wait before turn to reduce drift [ms]
const uint32_t TURN_WAIT = 800; // wait turn completion [ms]
const uint32_t ONLIGHT_WAIT = 1500; // wait before turn [ms]
const uint32_t CHANGEFLY_WAIT = 1000; // wait before changing right/left to avoid no effect [ms]

struct PilotRequest {
  bool land;
  int8_t forward;
  int8_t vertical_movement;
  int8_t right;
  int16_t turn;
  enum PILOT_STATE pilotState;
  bool force;
};

volatile bool keep_land = false;
volatile int8_t keep_forward = 0; // continue sending fly forward
volatile int8_t keep_right = 0;
volatile int8_t keep_vertical_movement = 0;

volatile uint32_t nearWallMillis = 0; // [ms]
volatile uint32_t reqTurnMillis = 0; // turn start time [ms]
volatile uint32_t onlightMillis = 0; // [ms]
volatile uint32_t changeFlyWaitMillis = 0; // wait before changing right/left [ms]

/// sensors
const uint16_t SENSING_INTERVAL = 200; // [ms]

const uint16_t FRONT_MIN = 2000; // 2m from wall (VL53L0X max sensing 2m)
const uint16_t UP_MIN = 300; // 30cm from ceiling
const uint16_t UP_MAX = 800; // 80cm from ceiling

const uint8_t PIN_LIGHTL = A5; // left light sensor NJL7502L
const uint8_t PIN_LIGHTR = A4; // right light sensor NJL7502L
const uint8_t PIN_XSHUT = D6;
const uint8_t TOF_FRONT_NEWADDR = 42; // TOF_UP = 41 (default)

static volatile int8_t triggerSensorPolling = 0;
static volatile int8_t triggerPilotPolling = 0;

VL53L0X tof_up;
VL53L0X tof_front;
FaBoTemperature adt7410;

/// sensing log data
struct LOGITEM {
  uint16_t ds; // millis() in deci(0.1)-seconds
  uint8_t type; // 0:temperature,(1:light), 10:takeoff,11:turn,12:fly,13:land
  int16_t value; // temperature*100 value/light/turn/fly value
} logdata[2048]; // mambo battery works 9m=540s
// len 5400: error "region RAM overflowed with stack"
static uint16_t logcount = 0;
const uint8_t LT_TEMPERATURE = 0;
const uint8_t LT_FRONT = 1;
const uint8_t LT_UP = 2;
const uint8_t LT_LIGHTL = 3; // left
const uint8_t LT_LIGHTR = 4; // right
const uint8_t LT_TAKEOFF = 10;
const uint8_t LT_LAND = 11;
const uint8_t LT_TURN = 12;
const uint8_t LT_FLY = 13;

void addlog(uint8_t type, int16_t value) {
  logdata[logcount].ds = millis() / 100;
  logdata[logcount].type = type;
  logdata[logcount].value = value;
  ++logcount;
}
void addflylog(int8_t forward, int8_t right, int8_t vertical) {
  uint16_t value = 0;
  value += (forward  > 0) ? 100 : (forward  < 0) ? 200 : 0;
  value += (right    > 0) ?  10 : (right    < 0) ?  20 : 0;
  value += (vertical > 0) ?   1 : (vertical < 0) ?   2 : 0;
  addlog(LT_FLY, value);
}

BLE ble;
ParrotMambo mambo(ble.gattClient());
Ticker sensingTicker;
Ticker pilotTicker;

/**
 * @brief  Function to decode advertisement or scan response data
 *
 * @param[in]  type            The data type that you want to get
 * @param[in]  advdata_len     The length of advertisement or scan reponse data
 * @param[in]  *p_advdata      The pointer of advertisement or scan reponse data
 * @param[out] *len            If type exist, this is the length of field data
 * @param[out] *p_field_data   If type exist, this is the pointer of field data
 *
 * @return NRF_SUCCESS or NRF_ERROR_NOT_FOUND
 */
uint32_t ble_advdata_parser(uint8_t type, uint8_t advdata_len, uint8_t *p_advdata, uint8_t *len, uint8_t *p_field_data) {
  uint8_t index=0;
  uint8_t field_length, field_type;

  while(index<advdata_len) {
    field_length = p_advdata[index];
    field_type   = p_advdata[index+1];
    if(field_type == type) {
      memcpy(p_field_data, &p_advdata[index+2], (field_length-1));
      *len = field_length - 1;
      return NRF_SUCCESS;
    }
    index += field_length + 1;
  }
  return NRF_ERROR_NOT_FOUND;
}

/**
 * @brief  Callback handle for scanning device
 *
 * @param[in]  *params   params->peerAddr            The peer's BLE address
 *                       params->rssi                The advertisement packet RSSI value
 *                       params->isScanResponse      Whether this packet is the response to a scan request
 *                       params->type                The type of advertisement
 *                                                   (enum from 0 ADV_CONNECTABLE_UNDIRECTED,ADV_CONNECTABLE_DIRECTED,ADV_SCANNABLE_UNDIRECTED,ADV_NON_CONNECTABLE_UNDIRECTED)
 *                       params->advertisingDataLen  Length of the advertisement data
 *                       params->advertisingData     Pointer to the advertisement packet's data
 */
static void scanCallBack(const Gap::AdvertisementCallbackParams_t *params) {
  uint8_t index;
  Serial.println("Scan CallBack ");
  Serial.print("PeerAddress: ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  uint8_t len;
  uint8_t adv_name[32];
  if( NRF_SUCCESS == ble_advdata_parser(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, params->advertisingDataLen, (uint8_t *)params->advertisingData, &len, adv_name) ) {
    adv_name[len] = 0; // '\0'
    Serial.print("Short name is : ");
    Serial.println((const char*)adv_name);
  }
  else if( NRF_SUCCESS == ble_advdata_parser(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, params->advertisingDataLen, (uint8_t *)params->advertisingData, &len, adv_name) ) {
    adv_name[len] = 0; // '\0'
    Serial.print("Complete name is : ");
    Serial.println((const char*)adv_name);

    if( memcmp("Mambo_", adv_name, 6) == 0x00 ) {
      Serial.println("Got device, stop scan ");
      ble.stopScan();
      ble.connect(params->peerAddr, BLEProtocol::AddressType::RANDOM_STATIC, NULL, NULL);
    }
  }
  Serial.println(" ");
}

void connectionCallBack(const Gap::ConnectionCallbackParams_t *params) {
  mambo.connectionCallback(params);
}

void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected"); //, start to scanning");
  mambo.disconnectionCallback(params);
  //ble.startScan(scanCallBack);
}

static void discoveryTerminationCallBack(Gap::Handle_t connectionHandle) {
  mambo.discoveryTerminationCallback(connectionHandle);
}

void hvxCallBack(const GattHVXCallbackParams *params) {
  mambo.hvxCallback(params);
}

void onDataWriteCallBack(const GattWriteCallbackParams *params) {
  mambo.onDataWriteCallback(params);
}

void onDataReadCallBack(const GattReadCallbackParams *params) {
  mambo.onDataReadCallback(params);
}

void onReadyCallBack() {
  mambo.takeoff();
  addlog(LT_TAKEOFF, 0);
  pilotState = PS_WEST;
}

void periodicSensingCallback() {
  if (triggerSensorPolling == 0) {
    triggerSensorPolling = 1;
  }
}

void periodicPilotCallback() {
  if (triggerPilotPolling == 0) {
    triggerPilotPolling = 1;
  }
}

void setupSensors() {
  // https://forum.pololu.com/t/vl53l0x-maximum-sensors-on-i2c-arduino-bus/10845/7
  pinMode(PIN_XSHUT, OUTPUT); // LOW: shutdown tof_up

  Wire.begin();

  // change address to use multiple VL53L0X
  tof_front.setAddress(TOF_FRONT_NEWADDR);
  pinMode(PIN_XSHUT, INPUT); // HIGH: boot tof_up. default addr
  delay(50);
  tof_front.init();
  tof_front.setTimeout(300);
  tof_front.setMeasurementTimingBudget(20000);
  tof_up.init();
  tof_up.setTimeout(300);
  tof_up.setMeasurementTimingBudget(20000);

  adt7410.begin();
}

void setup() {
  Serial.begin(9600);
  setupSensors();

  sensingTicker.attach_us(periodicSensingCallback, SENSING_INTERVAL * 1000);
  if (PILOT_INTERVAL != 0) {
    pilotTicker.attach_us(periodicPilotCallback, PILOT_INTERVAL * 1000);
  }
  ble.init();
  ble.onConnection(connectionCallBack);
  ble.onDisconnection(disconnectionCallBack);
  ble.gattClient().onServiceDiscoveryTermination(discoveryTerminationCallBack);
  ble.gattClient().onHVX(hvxCallBack);
  ble.gattClient().onDataWrite(onDataWriteCallBack);
  ble.gattClient().onDataRead(onDataReadCallBack);
  // scan interval : in milliseconds, valid values lie between 2.5ms and 10.24s
  // scan window :in milliseconds, valid values lie between 2.5ms and 10.24s
  // timeout : in seconds, between 0x0001 and 0xFFFF, 0x0000 disables timeout
  // activeScanning : true or false
  ble.setScanParams(1000, 200, 0, false);
  ble.startScan(scanCallBack);
  mambo.onReady(onReadyCallBack);
}

bool isDarker(int ambient) {
  return (ambient < AMBIENT_DARK_THRESHOLD);
}

bool isLighter(int ambient) {
  return (ambient >= AMBIENT_LIGHT_THRESHOLD);
}

enum FINDLIGHT_STATE getNewFindlightState(enum FINDLIGHT_STATE currentState, int ambient) {
  switch (findlightState) {
    case FS_LEAVING_LIGHT:
    case FS_ONLIGHT:
      if (!isDarker(ambient)) {
        prevDarker = 0;
        return currentState;
      }
      if (prevDarker == 0) {
        Serial.print("++prevDarker ");
        ++prevDarker;
        return currentState;
      }
      // if darker ambient sensor value continues
      Serial.print("approaching_light ");
      prevDarker = 0;
      return FS_APPROACHING_LIGHT;
    case FS_APPROACHING_LIGHT:
      if (!isLighter(ambient)) {
        onlightMillis = 0;
        return currentState;
      }
      if (onlightMillis == 0) {
        Serial.print("onlight0 ");
        onlightMillis = millis();
        return currentState;
      }
      Serial.print("onlightwait ");
      return FS_ONLIGHT_WAIT;
    case FS_ONLIGHT_WAIT:
      if (millis() - onlightMillis < ONLIGHT_WAIT) {
        return currentState;
      }
      Serial.print("onlight ");
      onlightMillis = 0;
      return FS_ONLIGHT;
  }
  return currentState;
}

int8_t senseForVerticalMovement() {
  if (!mambo.isFlying()) {
    return 0;
  }

  uint16_t mm_up = tof_up.readRangeSingleMillimeters();
  if (tof_up.timeoutOccurred()) {
    return 0;
  }
  //Serial.print("up ToF mm: "); Serial.println(mm_up); // from ceiling
  //addlog(LT_UP, mm_up); //DEBUG
  if (mm_up > UP_MAX) { // too far from ceiling
    Serial.print("up ");
    return UP_VALUE; // up
  } else if (mm_up < UP_MIN) { // too near from ceiling
    Serial.print("down ");
    return -UP_VALUE; // down
  }
  return 0;
}

bool senseFront(struct PilotRequest *req) {
  if (!mambo.isFlying()) {
    return false;
  }

  uint16_t mm_front = tof_front.readRangeSingleMillimeters();
  if (tof_front.timeoutOccurred()) {
    return false;
  }
  //Serial.print("front ToF mm: "); Serial.println(mm_front); // from wall
  if (mm_front > FRONT_MIN) {
    Serial.print("forward ");
    nearWallMillis = 0;
    req->forward = FORWARD_VALUE;
    return true;
  }

  // too near from wall
  addlog(LT_FRONT, mm_front); //DEBUG
  if (nearWallMillis == 0) {
    nearWallMillis = millis();
    Serial.print("nearWall ");
    req->forward = 0; // stop forward
    req->right = 0; // avoid forward by right != 0
    return true;
  }
  if (millis() - nearWallMillis < NEARWALL_WAIT) {
    return true;
  }

  // if near wall sensor value continues
  nearWallMillis = 0;
  switch (pilotState) {
    case PS_WEST:
      Serial.print("w2e ");
      req->turn = 90;
      req->forward = 0;
      req->right = 0;
      req->pilotState = PS_W2E;
      break;
    case PS_EAST:
      Serial.print("e2w ");
      req->turn = -90;
      req->forward = 0;
      req->right = 0;
      req->pilotState = PS_E2W;
      break;
    case PS_W2E:
    case PS_E2W:
    default:
      Serial.print("land ");
      req->land = true;
      break;
  }
  return true;
}

// decide left/right movement from left/right light sensor ADC values.
// trace light like line tracer.
int8_t decideLeftRightMovement(int left, int right) {
  if (max(left, right) < 30) { // absolute values are too dark
    return 0;
  }
  int diff = right - left;
  if (abs(diff) < right / 10) {
    return 0;
  }
  if (right > left) { // right is lighter
    return RIGHT_VALUE; // move right
  }
  if (right < left) { // left is lighter
    return -RIGHT_VALUE; // move left
  }
  return 0;
}

bool senseForPilot(struct PilotRequest *req) {
  req->pilotState = pilotState;

  if (reqTurnMillis > 0 && millis() - reqTurnMillis < TURN_WAIT) {
    return false;
  }
  reqTurnMillis = 0;

  req->vertical_movement = senseForVerticalMovement();
  bool hasReq = senseFront(req); // forward or turn

  int lightl = analogRead(PIN_LIGHTL);
  int lightr = analogRead(PIN_LIGHTR);
  Serial.print("lightl=");   Serial.print(lightl);
  Serial.print(", lightr="); Serial.println(lightr);
  if (mambo.isFlying()) {
    // PS_W2E/E2W: if find light line, turn_degrees(90/-90)
    // XXX: rewrite to use state machine and sensor events
    if (pilotState == PS_W2E || pilotState == PS_E2W) {
      int ambient = max(lightl, lightr);
      addlog(LT_LIGHTL, lightl); //DEBUG
      addlog(LT_LIGHTR, lightr); //DEBUG
      enum FINDLIGHT_STATE prev = findlightState;
      findlightState = getNewFindlightState(findlightState, ambient);
      if (findlightState == FS_ONLIGHT_WAIT) {
        req->forward = 0; // stop forward. hover a while to reduce drift after turn
        req->right = 0;
        return true;
      }
      if (prev == FS_ONLIGHT_WAIT && findlightState == FS_ONLIGHT) {
        if (pilotState == PS_W2E) {
          Serial.print("east ");
          req->turn = 90;
          req->pilotState = PS_EAST;
        } else { // PS_E2W
          Serial.print("west ");
          req->turn = -90;
          req->pilotState = PS_WEST;
        }
        return true;
      }
    } else if (req->turn == 0 && req->forward != 0
        && (pilotState == PS_WEST || pilotState == PS_EAST)) {
      addlog(LT_LIGHTL, lightl); //DEBUG
      addlog(LT_LIGHTR, lightr); //DEBUG
      // PS_WEST/EAST: trace light line
      req->right = decideLeftRightMovement(lightl, lightr);
      if (req->right != 0) {
        return true;
      }
    }
  }
  return hasReq;
}

bool needPilotRequest(struct PilotRequest *req) {
  if (!keep_land && req->land) {
    return true;
  }
  if (req->turn != 0) {
    return true;
  }
  if (changeFlyWaitMillis > 0) {
    if (millis() - changeFlyWaitMillis < CHANGEFLY_WAIT) {
      return false;
    }
    return true;
  }
  if (req->right != keep_right
      || req->forward != keep_forward
      || req->vertical_movement != keep_vertical_movement) {
    req->force = true;
    return true;
  }
  return false;
}

void sendPilotRequest(struct PilotRequest *req) {
  if (req->land) {
    keep_land = true;
  }
  if (keep_land) { // land is high priority
    mambo.land();
    addlog(LT_LAND, 0);
    pilotState = PS_LAND;
    return;
  }

  if (req->turn != 0) {
    keep_forward = 0;
    keep_right = 0;
    keep_vertical_movement = 0;
    mambo.turn_degrees(req->turn);
    addlog(LT_TURN, req->turn);
    reqTurnMillis = millis();
    if (pilotState != req->pilotState) { // turn 90/-90 degrees
        pilotState = req->pilotState;
        nearWallMillis = 0;
        prevDarker = 0;
        onlightMillis = 0;
        if (pilotState == PS_W2E || pilotState == PS_E2W) {
          Serial.println("leaving_light ");
          findlightState = FS_LEAVING_LIGHT;
        }
    }
    return;
  }

  if (changeFlyWaitMillis > 0) {
    if (millis() - changeFlyWaitMillis < CHANGEFLY_WAIT) {
      return;
    }
    changeFlyWaitMillis = 0;
  } else {
    if (req->right != 0 && req->right != keep_right) {
      changeFlyWaitMillis = millis();
      req->forward = 0; // stop forward
      req->right = 0; // avoid forward by right value
    }
  }

  keep_forward = req->forward;
  keep_right = req->right;
  keep_vertical_movement = req->vertical_movement;

  if (req->force || keep_forward != 0 || keep_right != 0 || keep_vertical_movement != 0) {
    mambo.fly(keep_right, keep_forward, 0, keep_vertical_movement);
    addflylog(keep_forward, keep_right, keep_vertical_movement);
  }
}

void loop() {
  static uint32_t prevTemperatureMillis = 0;
  struct PilotRequest req = {
    keep_land, keep_forward, keep_vertical_movement, keep_right, 0, pilotState, false
  };

  if (triggerSensorPolling == 1) {
    triggerSensorPolling = -1; // now sensing
    senseForPilot(&req);
    triggerSensorPolling = 0;

    if (millis() - prevTemperatureMillis > 2000) {
      prevTemperatureMillis = millis();
      float temp = adt7410.readTemperature();
      addlog(LT_TEMPERATURE, temp * 100);
      //Serial.print("temperature: "); Serial.println(temp, 1);
    }

    // send pilot command on sensing
    // || need to interrupt current keeping pilot request
    if (PILOT_INTERVAL == 0 || needPilotRequest(&req)) {
      triggerPilotPolling = 1;
    }
  }

  if (triggerPilotPolling == 1) {
    if (mambo.isFlying()) {
      triggerPilotPolling = -1;
      sendPilotRequest(&req);
    }
    triggerPilotPolling = 0;
    mambo.checkPing();
    return;
  }

  if (!mambo.isFlying() && Serial.available()) {
    int ch = Serial.read();
    if (ch == 'r') { // output logdata
      Serial.println();
      Serial.print(SENSING_INTERVAL); Serial.print(",");
      Serial.print(PILOT_INTERVAL); Serial.print(",");
      Serial.print(FORWARD_VALUE); Serial.print(",");
      Serial.print(RIGHT_VALUE); Serial.print(",");
      Serial.print(TURN_WAIT); Serial.println();
      for (int i = 0; i < logcount; i++) {
        struct LOGITEM *p = &logdata[i];
        Serial.print(p->ds); Serial.print(",");
        Serial.print(p->type); Serial.print(",");
        Serial.print(p->value); Serial.println();
      }
      logcount = 0;
    }
    return;
  }

  ble.waitForEvent(); // enter sleep and wait BLE event
}
