#include <nRF5x_BLE_API.h>

#include "Adafruit_VCNL4010.h"
// https://github.com/pololu/vl53l0x-arduino
#include <VL53L0X.h>
#include <FaBoTemperature_ADT7410.h>

const uint8_t XSHUT_PIN = D4;
const uint8_t TOF_UP_NEWADDR = 42; // TOF_FRONT = 41 (default)

const uint16_t UP_MIN = 300; // 30cm from ceiling
const uint16_t UP_MAX = 1000; // 1m from ceiling

Adafruit_VCNL4010 vcnl;
VL53L0X tof_up;
VL53L0X tof_front;
FaBoTemperature adt7410;

static uint32_t prevSensingMillis = 0;
static uint32_t prevTemperatureMillis = 0;


// cf.
// https://github.com/Mechazawa/minidrone-js
// https://github.com/algolia/pdrone
// https://github.com/fetherston/npm-parrot-minidrone
// https://github.com/amymcgovern/pymambo
// https://github.com/voodootikigod/node-rolling-spider

BLE           ble;

// MD_CLASSES
// https://github.com/Parrot-Developers/arsdk-xml/blob/master/xml/minidrone.xml
const uint8_t MDC_PILOTING = 0x00;
const uint8_t MDC_SPEED_SETTINGS = 0x01;
const uint8_t MDC_ANIMATION = 0x04;
const uint8_t MDC_MEDIA_RECORD = 0x06;
const uint8_t MDC_PILOTING_SETTINGS = 0x08;
const uint8_t MDC_NAVIGATION_DATA_STATE = 0x18;

// MD_METHODS
const uint8_t MDM_TRIM = 0x00;
const uint8_t MDM_TAKEOFF = 0x01;
const uint8_t MDM_PCMD = 0x02;
const uint8_t MDM_LAND = 0x03;
const uint8_t MDM_EMERGENCY = 0x04;
const uint8_t MDM_PICTURE = 0x01;
const uint8_t MDM_FLIP = 0x00;
const uint8_t MDM_MAX_ALTITUDE = 0x00;
const uint8_t MDM_MAX_TILT = 0x01;
const uint8_t MDM_MAX_VERTICAL_SPEED = 0x00;
const uint8_t MDM_MAX_ROTATION_SPEED = 0x01;
const uint8_t MDM_DRONE_POSITION = 0x00;

// MD_DATA_TYPES
const uint8_t MDDT_ACK = 0x01;
const uint8_t MDDT_DATA = 0x02;
const uint8_t MDDT_LLD = 0x03; // low latency data
const uint8_t MDDT_DATA_WITH_ACK = 0x04;

const uint8_t MD_DEVICE_TYPE = 0x02;

const uint8_t MD_END = 0x00;

// to write:
//  flight_params('fa0a'), command('fa0b'), emergency('fa0c')
// notify (needs to subscribe as handshake):
//  battery('fb0f'), flight_status('fb0e'), 'fb1b', 'fb1c',
//  'fd22', 'fd23', 'fd24', 'fd52', 'fd53', 'fd54',
const int SIZE_CHARS = 13;
static UUID uuid_chars[SIZE_CHARS] = {
  UUID("9a66fa0a-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fa0b-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fa0c-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fb0f-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fb0e-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fb1b-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fb1c-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd22-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd23-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd24-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd52-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd53-0800-9191-11e4-012d1540cb8e"),
  UUID("9a66fd54-0800-9191-11e4-012d1540cb8e"),
};
enum IDX_CHARS {
  IDX_FLIGHT_PARAMS, IDX_COMMAND, IDX_EMERGENCY,
  IDX_BATTERY, IDX_FLIGHT_STATUS, IDX_FB1B, IDX_FB1C,
  IDX_FD22, IDX_FD23, IDX_FD24, IDX_FD52, IDX_FD53, IDX_FD54
};
static DiscoveredCharacteristic            dchars[SIZE_CHARS];
static DiscoveredCharacteristicDescriptor  dchar_descs[SIZE_CHARS] = {
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
  DiscoveredCharacteristicDescriptor(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0)),
};
static int discovering_char_desc;
static int subscribing_char_desc;
static uint8_t steps_flight_params; // step count for fa0a
static uint8_t steps_command;       // step count for fa0b
//static uint8_t steps_emergency;     // step count for fa0c

static uint32_t takeoffMillis;
static uint32_t lastWriteMillis;
static int testForward = 1;

static void scanCallBack(const Gap::AdvertisementCallbackParams_t *params);
static void discoveredServiceCallBack(const DiscoveredService *service);
static void discoveredCharacteristicCallBack(const DiscoveredCharacteristic *chars);
static void discoveryTerminationCallBack(Gap::Handle_t connectionHandle);
static void discoveredCharsDescriptorCallBack(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params);
static void discoveredDescTerminationCallBack(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params) ;

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

void startDiscovery(uint16_t handle) {
  ble.gattClient().launchServiceDiscovery(handle, discoveredServiceCallBack, discoveredCharacteristicCallBack);
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
  Serial.print("PerrAddress: ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  Serial.print("The Rssi : ");
  Serial.println(params->rssi, DEC);

  Serial.print("The adv_data : ");
  Serial.println((const char*)params->advertisingData);

  uint8_t len;
  uint8_t adv_name[31];
  if( NRF_SUCCESS == ble_advdata_parser(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, params->advertisingDataLen, (uint8_t *)params->advertisingData, &len, adv_name) ) {
    Serial.print("Short name len : ");
    Serial.println(len, DEC);
    Serial.print("Short name is : ");
    Serial.println((const char*)adv_name);
  }
  else if( NRF_SUCCESS == ble_advdata_parser(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, params->advertisingDataLen, (uint8_t *)params->advertisingData, &len, adv_name) ) {
    Serial.print("Complete name len : ");
    Serial.println(len, DEC);
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

void connectionCallBack( const Gap::ConnectionCallbackParams_t *params ) {
  uint8_t index;

  Serial.print("The conn handle : ");
  Serial.println(params->handle, HEX);

  Serial.print("  The peerAddr : ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  // start to discovery
  startDiscovery(params->handle);
}

void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected"); //, start to scanning");
  lastWriteMillis = takeoffMillis = 0;
  //ble.startScan(scanCallBack);
}

static void discoveredServiceCallBack(const DiscoveredService *service) {
  Serial.println("\r\n----Service Discovered");

  Serial.print("Service UUID type        : ");
  Serial.println(service->getUUID().shortOrLong(), HEX);// 0 16bit_uuid, 1 128bit_uuid
  Serial.print("Service UUID             : ");
  if(service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    Serial.println(service->getUUID().getShortUUID(), HEX);
  }
  else {
    uint8_t index;
    const uint8_t *uuid = service->getUUID().getBaseUUID();
    for(index=0; index<16; index++) {
      Serial.print(uuid[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  Serial.print("The service start handle : ");
  Serial.println(service->getStartHandle(), HEX);
  Serial.print("The service end handle   : ");
  Serial.println(service->getEndHandle(), HEX);
}

static void discoveredCharacteristicCallBack(const DiscoveredCharacteristic *chars) {
  for (int i = 0; i < SIZE_CHARS; i++) {
    if (chars->getUUID() == uuid_chars[i]) {
      dchars[i] = *chars;
    }
  }

  Serial.println("\r\n----Characteristic Discovered");
  Serial.print("Chars UUID type        : ");
  Serial.println(chars->getUUID().shortOrLong(), HEX);// 0 16bit_uuid, 1 128bit_uuid
  Serial.print("Chars UUID             : ");
  if(chars->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    Serial.println(chars->getUUID().getShortUUID(), HEX);
  }
  else {
    uint8_t index;
    const uint8_t *uuid = chars->getUUID().getBaseUUID();
    for(index=0; index<16; index++) {
      Serial.print(uuid[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }

  Serial.print("properties_read        : ");
  Serial.println(chars->getProperties().read(), DEC);
  Serial.print("properties_writeWoResp : ");
  Serial.println(chars->getProperties().writeWoResp(), DEC);
  Serial.print("properties_write       : ");
  Serial.println(chars->getProperties().write(), DEC);
  Serial.print("properties_notify      : ");
  Serial.println(chars->getProperties().notify(), DEC);

  Serial.print("declHandle             : ");
  Serial.println(chars->getDeclHandle(), HEX);
  Serial.print("valueHandle            : ");
  Serial.println(chars->getValueHandle(), HEX);
  Serial.print("lastHandle             : ");
  Serial.println(chars->getLastHandle(), HEX);
}

static void discoveryTerminationCallBack(Gap::Handle_t connectionHandle) {
  Serial.println("\r\n----discoveryTermination");
  for (discovering_char_desc = 0; discovering_char_desc < SIZE_CHARS; discovering_char_desc++) {
    if (dchars[discovering_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
      Serial.println(dchars[discovering_char_desc].getUUID().getShortUUID(), HEX);
      ble.gattClient().discoverCharacteristicDescriptors(dchars[discovering_char_desc], discoveredCharsDescriptorCallBack, discoveredDescTerminationCallBack);
      break; // continue after discoveredDescTerminationCallBack
    }
  }
}

static void discoveredCharsDescriptorCallBack(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params) {
  Serial.println("\r\n----discovered descriptor");
  Serial.print("Descriptor UUID type        : ");
  Serial.println(params->descriptor.getUUID().shortOrLong(), HEX);
  Serial.print("Descriptor UUID         : ");
  if (params->descriptor.getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    Serial.println(params->descriptor.getUUID().getShortUUID(), HEX);
  } else {
    uint8_t index;
    const uint8_t *uuid = params->descriptor.getUUID().getBaseUUID();
    for (index = 0; index < 16; index++) {
      Serial.print(uuid[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  dchar_descs[discovering_char_desc] = params->descriptor;
  Serial.print("connectionHandle       : ");
  Serial.println(params->descriptor.getConnectionHandle(), HEX);
  Serial.print("descriptor Handle      : ");
  Serial.println(params->descriptor.getAttributeHandle(), HEX);
}

static void takeoff() {
  Serial.println("takeoff");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_PILOTING, MDM_TAKEOFF, MD_END
  };
  ble.gattClient().write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = takeoffMillis = millis();
}

static void land() {
  Serial.println("land");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_PILOTING, MDM_LAND, MD_END
  };
  ble.gattClient().write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

static void fly(int8_t roll, int8_t pitch, int8_t yaw, int8_t vertical) {
  Serial.print("fly:");
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(yaw); Serial.print(",");
  Serial.print(vertical); Serial.println();
  uint8_t flag = 1; // Boolean flag to activate roll/pitch movement
  uint8_t buf[] = {
    MDDT_DATA, ++steps_flight_params, MD_DEVICE_TYPE, MDC_PILOTING, MDM_PCMD, MD_END,
    flag, (uint8_t)roll, (uint8_t)pitch, (uint8_t)yaw, (uint8_t)vertical, 0, 0, 0, 0
  };
  ble.gattClient().write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_FLIGHT_PARAMS].getConnectionHandle(), dchars[IDX_FLIGHT_PARAMS].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

static void forward() {
  fly(0, 100, 0, 0);
}

// write some command to avoid disconnect after 5 seconds of inactivity
static void ping() {
  Serial.println("ping");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_NAVIGATION_DATA_STATE, MDM_DRONE_POSITION, MD_END
  };
  ble.gattClient().write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

static int handshake() {
  Serial.print("subscribe minidrone notify: ");
  for (; subscribing_char_desc < SIZE_CHARS; subscribing_char_desc++) {
    if (dchar_descs[subscribing_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
      Serial.println(dchars[subscribing_char_desc].getUUID().getShortUUID(), HEX);
      uint16_t value = 0x0001;
      ble.gattClient().write(GattClient::GATT_OP_WRITE_REQ, dchars[subscribing_char_desc].getConnectionHandle(), dchar_descs[subscribing_char_desc].getAttributeHandle(), 2, (uint8_t *)&value);
      return 0; // continue after onDataWriteCallback
    }
  }
  Serial.println("handshake done");
  // ready
  // XXX: takeoff test
  takeoff();
  return 1;
}

static void discoveredDescTerminationCallBack(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params) {
  Serial.println("\r\n----discovery descriptor Termination");
  if (++discovering_char_desc >= SIZE_CHARS) {
    // start handshake
    subscribing_char_desc = 0;
    handshake();
    return;
  }
  for (; discovering_char_desc < SIZE_CHARS; discovering_char_desc++) {
    if (dchars[discovering_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
      Serial.println(dchars[discovering_char_desc].getUUID().getShortUUID(), HEX);
      ble.gattClient().discoverCharacteristicDescriptors(dchars[discovering_char_desc], discoveredCharsDescriptorCallBack, discoveredDescTerminationCallBack);
      break; // continue after discoveredDescTerminationCallBack
    }
  }
}

/** @brief  write callback handle
 *
 *  @param[in] *params   params->connHandle : The handle of the connection that triggered the event
 *                       params->handle : Attribute Handle to which the write operation applies
 *                       params->writeOp : OP_INVALID               = 0x00,  // Invalid operation.
 *                                           OP_WRITE_REQ             = 0x01,  // Write request.
 *                                           OP_WRITE_CMD             = 0x02,  // Write command.
 *                                           OP_SIGN_WRITE_CMD        = 0x03,  // Signed write command.
 *                                           OP_PREP_WRITE_REQ        = 0x04,  // Prepare write request.
 *                                           OP_EXEC_WRITE_REQ_CANCEL = 0x05,  // Execute write request: cancel all prepared writes.
 *                                           OP_EXEC_WRITE_REQ_NOW    = 0x06,  // Execute write request: immediately execute all prepared writes.
 *                       params->offset : Offset for the write operation
 *                       params->len : Length (in bytes) of the data to write
 *                       params->data : Pointer to the data to write
 */
void onDataWriteCallBack(const GattWriteCallbackParams *params) {
  Serial.print("GattClient write call back: ");
  Serial.println(params->handle, HEX);
  if (subscribing_char_desc < SIZE_CHARS) {
    ++subscribing_char_desc;
    handshake();
    return;
  }
}

/** @brief  read callback handle
 *
 *  @param[in] *params   params->connHandle : The handle of the connection that triggered the event
 *                       params->handle : Attribute Handle to which the write operation applies
 *                       params->offset : Offset for the write operation
 *                       params->len : Length (in bytes) of the data to write
 *                       params->data : Pointer to the data to write
 */
void onDataReadCallBack(const GattReadCallbackParams *params) {
  Serial.println("GattClient read call back ");
  Serial.print("The handle : ");
  Serial.println(params->handle, HEX);
  Serial.print("The offset : ");
  Serial.println(params->offset, DEC);
  Serial.print("The len : ");
  Serial.println(params->len, DEC);
  Serial.print("The data : ");
  for(uint8_t index=0; index<params->len; index++) {
    Serial.print( params->data[index], HEX);
  }
  Serial.println("");
}

/** @brief  hvx callback handle
 *
 *  @param[in] *params   params->connHandle : The handle of the connection that triggered the event
 *                       params->handle : Attribute Handle to which the write operation applies
 *                       params->type : BLE_HVX_NOTIFICATION = 0x01
 *                                      BLE_HVX_INDICATION   = 0x02
 *                       params->len : Length (in bytes) of the data to write
 *                       params->data : Pointer to the data to write
 */
void hvxCallBack(const GattHVXCallbackParams *params) {
  Serial.println("GattClient notify call back ");
  Serial.print("The handle : ");
  Serial.println(params->handle, HEX);
  // BC: flight_status(fb0e)
  // BF: battery(fb0f)
  Serial.print("The len : ");
  Serial.println(params->len, DEC);
  for(unsigned char index=0; index<params->len; index++) {
    Serial.print(params->data[index], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

void setupSensors() {
  // https://forum.pololu.com/t/vl53l0x-maximum-sensors-on-i2c-arduino-bus/10845/7
  pinMode(XSHUT_PIN, OUTPUT); // LOW: shutdown tof_front

  Wire.begin();

  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  // disable proximity sensing. use ambient sensing only
  vcnl.setLEDcurrent(0);
  vcnl.setFrequency(VCNL4010_1_95);

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
}

void loop() {
  if (testForward > 0 && takeoffMillis > 0 && millis() - takeoffMillis > 2000) {
    forward();
    testForward--;
  }
  if (takeoffMillis > 0 && millis() - takeoffMillis > 10000) {
    takeoffMillis = 0;
    land();
  }
  // avoid disconnect after 5 seconds of inactivity
  // (lastWriteMillis may be updated in forward()/land())
  if (lastWriteMillis > 0 && millis() - lastWriteMillis > 4000) {
    ping();
  }
  ble.waitForEvent();

  if (millis() - prevSensingMillis > 1000) {
    prevSensingMillis = millis();
    uint16_t mm_up = tof_up.readRangeSingleMillimeters();
    uint16_t mm_front = tof_front.readRangeSingleMillimeters();
    uint16_t ambient = vcnl.readAmbient();
    //Serial.print("sensing ms: "); Serial.println(millis() - prevSensingMillis); // ex.164

    if (!tof_up.timeoutOccurred()) {
      Serial.print("up ToF mm: "); Serial.println(mm_up);
      if (takeoffMillis > 0) {
        if (mm_up > UP_MAX) {
          fly(0, 0, 0, 50);
        } else if (mm_up < UP_MIN) {
          fly(0, 0, 0, -50);
        }
      }
    }
    if (!tof_front.timeoutOccurred()) {
      Serial.print("front ToF mm: "); Serial.println(mm_front);
    }
    Serial.print("Ambient: "); Serial.println(ambient);
  }

  if (millis() - prevTemperatureMillis > 2000) {
    prevTemperatureMillis = millis();
    float temp = adt7410.readTemperature();
    Serial.print("temperature: "); Serial.println(temp, 1);
  }
}
