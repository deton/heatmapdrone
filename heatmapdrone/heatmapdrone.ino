#include <nRF5x_BLE_API.h>
#include "Adafruit_VCNL4010.h"
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
  FS_LEAVING_LIGHT, FS_APPROACHING_LIGHT, FS_ONLIGHT
};
volatile enum FINDLIGHT_STATE findlightState = FS_ONLIGHT;
volatile uint8_t prevDarker = 0;
volatile uint8_t prevLighter = 0;

const uint16_t AMBIENT_DARK_THRESHOLD = 2400; // XXX
const uint16_t AMBIENT_LIGHT_THRESHOLD = 3900; // XXX

// substate for tracing ceiling light on PS_EAST/WEST state
enum TRACE_STATE {
  TS_RIGHT, TS_LEFT, TS_ONLIGHT
};
volatile enum TRACE_STATE traceState = TS_ONLIGHT;
volatile enum TRACE_STATE recentReqTraceState = TS_ONLIGHT;
volatile int16_t waitingForward = 0; // waiting fly forward? (after left/right)
const int16_t WAIT_FORWARD = 300; // [ms]

const uint16_t PILOT_INTERVAL = 50; // [ms]
const uint16_t FORWARD_VALUE = 15; // [-100,100]
const uint16_t UP_VALUE = 40; // [-100,100]
const uint16_t TURN_RIGHT_VALUE = 20; // [degree]
const int8_t DRIFT_OFFSET = 0; // offset to fix drift on forward [-100,100]
const uint32_t TURNWAIT_90 = 500; // [ms]
const uint32_t TURNWAIT_RIGHT = TURNWAIT_90 * TURN_RIGHT_VALUE / 90; // [ms]

struct PilotRequest {
  bool land;
  int8_t forward;
  int8_t vertical_movement;
  int16_t turn;
  enum PILOT_STATE pilotState;
};

volatile bool keep_land = false;
volatile int8_t keep_forward = 0; // continue sending fly forward
volatile int8_t keep_vertical_movement = 0;

volatile uint8_t prevNearWall = 0;
volatile uint32_t reqTurnMillis = 0; // turn start time [ms]
volatile uint32_t reqTurnWait = TURNWAIT_90; // [ms]

/// sensors
// XXX: sensing takes 164ms.  If interval is 200, no additional pilot (forward)
//      request is sent and drift increase
const uint16_t SENSING_INTERVAL = 300; // [ms]

const uint16_t FRONT_MIN = 2000; // 2m from wall (VL53L0X max sensing 2m)
const uint16_t UP_MIN = 300; // 30cm from ceiling
const uint16_t UP_MAX = 1000; // 1m from ceiling

const uint8_t XSHUT_PIN = D4;
const uint8_t TOF_UP_NEWADDR = 42; // TOF_FRONT = 41 (default)

static volatile int8_t triggerSensorPolling = 0;
static volatile int8_t triggerPilotPolling = 0;

Adafruit_VCNL4010 vcnl;
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
const uint8_t LT_LIGHT = 1;
const uint8_t LT_FRONT = 2;
const uint8_t LT_UP = 3;
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

void vcnl4010_setAmbientContinuousMode(bool enable) {
  // XXX: Adafruit_VCNL4010.h has no API to set ambient parameter
  uint8_t data = 0;
  if (enable) {
    data |= 1 << 7; // continuous conversion mode for faster measurements
  }
  data |= 1 << 4; // ambient light measurement rate. default: 2 samples/s
  data |= 1 << 3; // auto offset compensation. default: enable
  data |= 5; // averaging function (number of measurements per run). default: 32conv.

  Wire.beginTransmission(VCNL4010_I2CADDR_DEFAULT);
  Wire.write(VCNL4010_AMBIENTPARAMETER);
  Wire.write(data);
  Wire.endTransmission();
}

void setupSensors() {
  // https://forum.pololu.com/t/vl53l0x-maximum-sensors-on-i2c-arduino-bus/10845/7
  pinMode(XSHUT_PIN, OUTPUT); // LOW: shutdown tof_front

  Wire.begin();

  if (! vcnl.begin()){
    Serial.println("VCNL4010 Sensor not found :(");
    while (1);
  }
  // disable proximity sensing. use ambient sensing only
  vcnl.setLEDcurrent(0);
  vcnl.setFrequency(VCNL4010_1_95);
  // set VCNL4010 to continuous mode for short time sensing
  // (default mode takes 117ms, continous mode takes 36ms)
  vcnl4010_setAmbientContinuousMode(true);

  // change address to use multiple VL53L0X
  tof_up.setAddress(TOF_UP_NEWADDR);
  pinMode(XSHUT_PIN, INPUT); // HIGH: boot tof_front. default addr
  delay(10);
  tof_up.init();
  tof_up.setTimeout(300);
  tof_up.setMeasurementTimingBudget(20000);
  tof_front.init();
  tof_front.setTimeout(300);
  tof_front.setMeasurementTimingBudget(20000);

  adt7410.begin();
}

void setup() {
  Serial.begin(9600);
  setupSensors();

  sensingTicker.attach_us(periodicSensingCallback, SENSING_INTERVAL * 1000);
  pilotTicker.attach_us(periodicPilotCallback, PILOT_INTERVAL * 1000);
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

bool isDarker(uint16_t ambient) {
  return (ambient < AMBIENT_DARK_THRESHOLD);
}

bool isLighter(uint16_t ambient) {
  return (ambient >= AMBIENT_LIGHT_THRESHOLD);
}

enum FINDLIGHT_STATE getNewFindlightState(enum FINDLIGHT_STATE currentState, uint16_t ambient) {
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
        prevLighter = 0;
        return currentState;
      }
      if (prevLighter == 0) {
        Serial.print("++prevLighter ");
        ++prevLighter;
        return currentState;
      }
      Serial.print("onlight ");
      prevLighter = 0;
      return FS_ONLIGHT;
  }
  return currentState;
}

enum TRACE_STATE getNewTraceState(enum TRACE_STATE currentState, uint16_t ambient) {
  switch (currentState) {
    case TS_ONLIGHT:
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
      prevDarker = 0;
      return (recentReqTraceState == TS_LEFT) ? TS_RIGHT : TS_LEFT;
    case TS_LEFT:
    case TS_RIGHT:
      if (!isDarker(ambient)) {
        prevDarker = 0;
        if (prevLighter == 0) {
          ++prevLighter;
          return currentState;
        }
        prevLighter = 0;
        return TS_ONLIGHT;
      }
      prevLighter = 0;
      if (prevDarker == 0) {
        ++prevDarker;
        return currentState;
      }
      prevDarker = 0;
      return (currentState == TS_LEFT) ? TS_RIGHT : TS_LEFT;
  }
  return currentState;
}

int16_t updateTraceState(uint16_t ambient) {
  enum TRACE_STATE prev = traceState;
  traceState = getNewTraceState(traceState, ambient);
  if (prev == traceState) {
    return 0;
  }
  if (traceState == TS_RIGHT) {
    Serial.print("right ");
    recentReqTraceState = traceState;
    return TURN_RIGHT_VALUE;
  } else if (traceState == TS_LEFT) {
    Serial.print("left ");
    recentReqTraceState = traceState;
    return -TURN_RIGHT_VALUE;
  }
  return 0;
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
    prevNearWall = 0;
    req->forward = FORWARD_VALUE;
    return true;
  }
  addlog(LT_FRONT, mm_front); //DEBUG

  // too near from wall
  if (prevNearWall < 5) {
    Serial.print("++prevNearWall ");
    ++prevNearWall;
    //if (prevNearWall >= 3) {
    //  req->forward = -FORWARD_VALUE; // backward
    //} else {
      req->forward = 0; // stop forward
    //}
    return true;
  }

  // if near wall sensor value continues
  switch (pilotState) {
    case PS_WEST:
      Serial.print("w2e ");
      req->turn = 90;
      req->forward = 0;
      req->pilotState = PS_W2E;
      break;
    case PS_EAST:
      Serial.print("e2w ");
      req->turn = -90;
      req->forward = 0;
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

bool senseForPilot(struct PilotRequest *req) {
  req->pilotState = pilotState;

  if (reqTurnMillis > 0 && millis() - reqTurnMillis < reqTurnWait) {
    return false;
  }
  reqTurnMillis = 0;

  req->vertical_movement = senseForVerticalMovement();
  bool hasReq = senseFront(req);

  uint16_t ambient = vcnl.readAmbient();
  //Serial.print("sensing ms: "); Serial.println(millis() - st); // ex.164ms (readAmbient: 117ms, tof: 23ms)

  Serial.print("ambient="); Serial.println(ambient);
  if (mambo.isFlying()) {
    // PS_W2E/E2W: if find light line, turn_degrees(90/-90)
    // XXX: rewrite to use state machine and sensor events
    if (pilotState == PS_W2E || pilotState == PS_E2W) {
      addlog(LT_LIGHT, ambient); //DEBUG
      enum FINDLIGHT_STATE prev = findlightState;
      findlightState = getNewFindlightState(findlightState, ambient);
      if (prev == FS_APPROACHING_LIGHT && findlightState == FS_ONLIGHT) {
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
    } else if (req->turn == 0 && (pilotState == PS_WEST || pilotState == PS_EAST) && waitingForward <= 0) {
      addlog(LT_LIGHT, ambient); //DEBUG
      // PS_WEST/EAST: trace light line
      req->turn = updateTraceState(ambient);
      if (req->turn != 0) {
        waitingForward = WAIT_FORWARD;
        return true;
      }
    }
  }
  return hasReq;
}

void sendPilotRequest(const struct PilotRequest& req) {
  if (req.land) {
    keep_land = true;
  }
  if (keep_land) { // land is high priority
    mambo.land();
    addlog(LT_LAND, 0);
    pilotState = PS_LAND;
    return;
  }

  if (req.turn != 0) {
    keep_forward = 0;
    keep_vertical_movement = 0;
    mambo.turn_degrees(req.turn);
    addlog(LT_TURN, req.turn);
    reqTurnMillis = millis();
    if (pilotState != req.pilotState) { // turn 90/-90 degrees
        pilotState = req.pilotState;
        prevNearWall = 0;
        prevDarker = 0;
        prevLighter = 0;
        reqTurnWait = TURNWAIT_90;
        if (pilotState == PS_W2E || pilotState == PS_E2W) {
          Serial.println("leaving_light ");
          findlightState = FS_LEAVING_LIGHT;
        } else { // PS_EAST/WEST
          // reset trace state and related vars
          traceState = TS_ONLIGHT;
          recentReqTraceState = (pilotState == PS_EAST) ? TS_RIGHT : TS_LEFT;
        }
    } else { // turn right/left
        reqTurnWait = TURNWAIT_RIGHT;
    }
    return;
  }
  keep_forward = req.forward;
  keep_vertical_movement = req.vertical_movement;

  if (keep_forward != 0 || keep_vertical_movement != 0 || reqTurnMillis > 0 && millis() - reqTurnMillis >= reqTurnWait - PILOT_INTERVAL * 2) {
    mambo.fly(DRIFT_OFFSET, keep_forward, 0, keep_vertical_movement);
    addlog(LT_FLY, keep_forward * 100 + keep_vertical_movement);
    if (keep_forward != 0) {
      waitingForward -= PILOT_INTERVAL;
    }
  }
}

void loop() {
  static uint32_t prevTemperatureMillis = 0;
  struct PilotRequest req = {
    keep_land, keep_forward, keep_vertical_movement, 0, pilotState
  };
  bool hasReq = false;

  if (triggerSensorPolling == 1) {
    triggerSensorPolling = -1; // now sensing
    hasReq = senseForPilot(&req);
    triggerSensorPolling = 0;

    if (millis() - prevTemperatureMillis > 2000) {
      prevTemperatureMillis = millis();
      float temp = adt7410.readTemperature();
      addlog(LT_TEMPERATURE, temp * 100);
      //Serial.print("temperature: "); Serial.println(temp, 1);
    }
  }

  if (triggerPilotPolling == 1) {
    if (mambo.isFlying()) {
      triggerPilotPolling = -1;
      sendPilotRequest(req);
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
      Serial.print(FORWARD_VALUE); Serial.print(",");
      Serial.print(TURNWAIT_90); Serial.println();
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
