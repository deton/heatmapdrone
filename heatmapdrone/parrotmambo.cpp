#include <Arduino.h>
#include "parrotmambo.h"

#define VERBOSE 1 // XXX: if 0, BLE connection does not complete occasionally

/// BLE and minidrone
// cf.
// https://github.com/Mechazawa/minidrone-js
// https://github.com/algolia/pdrone
// https://github.com/fetherston/npm-parrot-minidrone
// https://github.com/amymcgovern/pymambo
// https://github.com/voodootikigod/node-rolling-spider

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
const uint8_t MDM_CAP = 0x01;
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

static UUID uuid_chars[] = {
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
  IDX_BATTERY, IDX_FLIGHT_STATUS, IDX_COMMANDACK, IDX_EMERGENCYACK,
  IDX_FD22, IDX_FD23, IDX_FD24, IDX_FD52, IDX_FD53, IDX_FD54
};

ParrotMambo::ParrotMambo(GattClient& gattClient)
    : gattClient(gattClient), flyingState(FS_INIT), onReadyCallback(NULL),
    dchar_descs({
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
    }), discovering_char_desc(0), subscribing_char_desc(0),
    steps_flight_params(0), steps_command(0), steps_emergency(0),
    lastWriteMillis(0) {
}

void ParrotMambo::onReady(OnReadyCallback_t cb) {
  onReadyCallback = cb;
}

void ParrotMambo::connectionCallback(const Gap::ConnectionCallbackParams_t *params) {
#if VERBOSE
  uint8_t index;

  Serial.print("The conn handle : ");
  Serial.println(params->handle, HEX);

  Serial.print("  The peerAddr : ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif

  // start to discovery
  FunctionPointerWithContext<const DiscoveredService *> ds(this, &ParrotMambo::discoveredServiceCallback);
  FunctionPointerWithContext<const DiscoveredCharacteristic *> dc(this, &ParrotMambo::discoveredCharacteristicCallback);
  gattClient.launchServiceDiscovery(params->handle, ds, dc);
}

void ParrotMambo::disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params) {
  lastWriteMillis = 0;
}

void ParrotMambo::discoveredServiceCallback(const DiscoveredService *service) {
#if VERBOSE
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
#endif
}

void ParrotMambo::discoveredCharacteristicCallback(const DiscoveredCharacteristic *chars) {
  for (int i = 0; i < SIZE_CHARS; i++) {
    if (chars->getUUID() == uuid_chars[i]) {
      dchars[i] = *chars;
    }
  }

#if VERBOSE
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
#endif
}

void ParrotMambo::discoveryTerminationCallback(Gap::Handle_t connectionHandle) {
#if VERBOSE
  Serial.println("\r\n----discoveryTermination");
#endif
  for (discovering_char_desc = 0; discovering_char_desc < SIZE_CHARS; discovering_char_desc++) {
    if (dchars[discovering_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
#if VERBOSE
      Serial.println(dchars[discovering_char_desc].getUUID().getShortUUID(), HEX);
#endif
      discoverCharacteristicDescriptors();
      break; // continue after discoveredDescTerminationCallback
    }
  }
}

void ParrotMambo::discoverCharacteristicDescriptors() {
  FunctionPointerWithContext<const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *> dcd(this, &ParrotMambo::discoveredCharsDescriptorCallback);
  FunctionPointerWithContext<const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *> ddt(this, &ParrotMambo::discoveredDescTerminationCallback);
  gattClient.discoverCharacteristicDescriptors(dchars[discovering_char_desc], dcd, ddt);
}

void ParrotMambo::discoveredCharsDescriptorCallback(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params) {
  dchar_descs[discovering_char_desc] = params->descriptor;
#if VERBOSE
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
  Serial.print("connectionHandle       : ");
  Serial.println(params->descriptor.getConnectionHandle(), HEX);
  Serial.print("descriptor Handle      : ");
  Serial.println(params->descriptor.getAttributeHandle(), HEX);
#endif
}

// send flat trim (calibration) command for flight stability.
// (to reduce sliding in hover mode after crash)
void ParrotMambo::flattrim() {
  Serial.println("flattrim");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_PILOTING, MDM_TRIM, MD_END
  };
  gattClient.write(GattClient::GATT_OP_WRITE_REQ, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

void ParrotMambo::takeoff() {
  Serial.println("takeoff");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_PILOTING, MDM_TAKEOFF, MD_END
  };
  gattClient.write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

void ParrotMambo::land() {
  Serial.println("land");
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_PILOTING, MDM_LAND, MD_END
  };
  gattClient.write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

// @param roll: right/left [-100, 100]
// @param pitch: forward/backward [-100, 100]
// @param yaw: turn [-100, 100]
// @param vertical: up/down [-100, 100]
void ParrotMambo::fly(int8_t roll, int8_t pitch, int8_t yaw, int8_t vertical) {
  Serial.print("fly:");
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(yaw); Serial.print(",");
  Serial.println(vertical);
  uint8_t flag = 1; // Boolean flag to activate roll/pitch movement
  uint8_t buf[] = {
    MDDT_DATA, ++steps_flight_params, MD_DEVICE_TYPE, MDC_PILOTING, MDM_PCMD, MD_END,
    flag, (uint8_t)roll, (uint8_t)pitch, (uint8_t)yaw, (uint8_t)vertical, 0, 0, 0, 0
  };
  gattClient.write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_FLIGHT_PARAMS].getConnectionHandle(), dchars[IDX_FLIGHT_PARAMS].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

// Turn the mambo the specified number of degrees [-180, 180]
void ParrotMambo::turn_degrees(int16_t degrees) {
  Serial.print("turn_degrees "); Serial.println(degrees);
  uint16_t d = (uint16_t)degrees;
  //uint8_t *p = (uint8_t *)&degrees;
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_ANIMATION, MDM_CAP, MD_END,
    (uint8_t)(d & 0xFF), (uint8_t)(d >> 8) // little endian
  };
  gattClient.write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

// write some command to avoid disconnect after 5 seconds of inactivity
void ParrotMambo::checkPing() {
  // avoid disconnect after 5 seconds of inactivity
  // (lastWriteMillis may be updated in land()/fly()/turn_degrees())
  if (lastWriteMillis == 0 || millis() - lastWriteMillis < 4000) {
    return;
  }
  Serial.println("ping");
  navigationDataState();
}

void ParrotMambo::navigationDataState() {
  uint8_t buf[] = {
    MDDT_DATA, ++steps_command, MD_DEVICE_TYPE, MDC_NAVIGATION_DATA_STATE, MDM_DRONE_POSITION, MD_END
  };
  gattClient.write(GattClient::GATT_OP_WRITE_CMD, dchars[IDX_COMMAND].getConnectionHandle(), dchars[IDX_COMMAND].getValueHandle(), sizeof(buf), buf);
  lastWriteMillis = millis();
}

int ParrotMambo::handshake() {
#if VERBOSE
  Serial.print("subscribe minidrone notify: ");
#endif
  for (; subscribing_char_desc < SIZE_CHARS; subscribing_char_desc++) {
    if (dchar_descs[subscribing_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
      Serial.println(dchars[subscribing_char_desc].getUUID().getShortUUID(), HEX);
      uint16_t value = 0x0001;
      gattClient.write(GattClient::GATT_OP_WRITE_REQ, dchars[subscribing_char_desc].getConnectionHandle(), dchar_descs[subscribing_char_desc].getAttributeHandle(), 2, (uint8_t *)&value);
      return 0; // continue after onDataWriteCallback
    }
  }
  Serial.println("handshake done");
  // ready
  flattrim();
  return 1;
}

void ParrotMambo::discoveredDescTerminationCallback(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params) {
#if VERBOSE
  Serial.println("\r\n----discovery descriptor Termination");
#endif
  if (++discovering_char_desc >= SIZE_CHARS) {
    // start handshake
    subscribing_char_desc = 0;
    handshake();
    return;
  }
  for (; discovering_char_desc < SIZE_CHARS; discovering_char_desc++) {
    if (dchars[discovering_char_desc].getUUID() != UUID::ShortUUIDBytes_t(0)) {
      Serial.println(dchars[discovering_char_desc].getUUID().getShortUUID(), HEX);
      discoverCharacteristicDescriptors();
      break; // continue after discoveredDescTerminationCallback
    }
  }
}

int ParrotMambo::getCharsIdxByHandle(GattAttribute::Handle_t handle) {
  for (int i = 0; i < SIZE_CHARS; i++) {
    if (dchars[i].getValueHandle() == handle) {
      return i;
    }
  }
  return -1;
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
void ParrotMambo::onDataWriteCallback(const GattWriteCallbackParams *params) {
#if VERBOSE
  Serial.print("GattClient write call back: ");
  Serial.println(params->handle, HEX);
#endif
  if (subscribing_char_desc < SIZE_CHARS) {
    ++subscribing_char_desc;
    handshake();
    return;
  }

  // ACK for flattrim
  if (flyingState == FS_INIT) {
    // ACK for WRITE_REQ on IDX_COMMAND
    if (getCharsIdxByHandle(params->handle) == IDX_COMMAND) {
      if (onReadyCallback) {
        onReadyCallback();
      }
    }
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
void ParrotMambo::onDataReadCallback(const GattReadCallbackParams *params) {
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
void ParrotMambo::hvxCallback(const GattHVXCallbackParams *params) {
  if (params->type != BLE_HVX_NOTIFICATION) {
    return;
  }
  // BC: flight_status(fb0e)
  // BF: battery(fb0f)
  if (getCharsIdxByHandle(params->handle) != IDX_FLIGHT_STATUS) {
    return;
  }
  //   ack_id, packet_id, project_id, myclass_id, cmd_id, extra_id, param
  //   2, n, 2 (minidrone.xml), 3 (PilotingState), 1 (FlyingStateChanged), 0, e0, e1, e2, e3 (e3e2e1e0: little endian enum(uint16_t): landed=0, takingoff, hovering, flying, landing, emergency, rolling, init)
  //   2, n, 2 (minidrone.xml), 3 (PilotingState), 2 (AlertStateChanged), 0, e0, e1, e2, e3 (e3e2e1e0: little endian enum(uint16_t): none=0, user, cut_off, critical_battery, low_battery)
  //   2, n, 2 (minidrone.xml), 3 (PilotingState), 3 (AutoTakeOffModeChanged), 0, m
  //   2, n, 0 (common.xml), 5 (CommonState), 1 (BatteyStateChanged), 0, p (battery percentage)
  //   2, n, 0 (common.xml), 1E (RunState), 0 (RunIdChanged), 0, (string)
  if (params->len < 7) {
    return;
  }
  //static const uint8_t PROJECT_ID_COMMON = 0;
  static const uint8_t PROJECT_ID_MINIDRONE = 2;
  if (params->data[2] != PROJECT_ID_MINIDRONE) {
    return;
  }
  //static const uint8_t MYCLASS_ID_COMMONSTATE = 5;
  //static const uint8_t MYCLASS_ID_RUNSTATE = 30; // 0x1E
  static const uint8_t MYCLASS_ID_PILOTINGSTATE = 3;
  if (params->data[3] != MYCLASS_ID_PILOTINGSTATE) {
    return;
  }
  //static const uint8_t CMD_ID_FLATTRIMCHANGED = 0;
  static const uint8_t CMD_ID_FLYINGSTATECHANGED = 1;
  //static const uint8_t CMD_ID_ALERTSTATECHANGED = 2;
  //static const uint8_t CMD_ID_AUTOTAKEOFFMODECHANGED = 3;
  //static const uint8_t CMD_ID_BATTERYSTATECHANGED = 0;
  //static const uint8_t CMD_ID_RUNIDCHANGED = 0;
  if (params->data[4] != CMD_ID_FLYINGSTATECHANGED) {
    //Serial.print("piloting state changed (other than flying state):");
    //Serial.println(params->data[4]);
    return;
  }
  flyingState = (enum FLYING_STATE)params->data[6];
}

bool ParrotMambo::isFlying() {
  switch (flyingState) {
    case FS_HOVERING:
    case FS_FLYING:
      return true;
    case FS_TAKINGOFF:
    case FS_LANDING:
      return false;
    case FS_LANDED:
    case FS_EMERGENCY:
    case FS_INIT:
    default:
      return false;
  }
}
