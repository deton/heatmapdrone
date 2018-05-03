#pragma once
#include <nRF5x_BLE_API.h>

class ParrotMambo {
  public:
    typedef void (*OnReadyCallback_t)();

    ParrotMambo(GattClient& gattClient);
    void onReady(OnReadyCallback_t onReadyCallback);

    void connectionCallback(const Gap::ConnectionCallbackParams_t *params);
    void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params);
    void discoveryTerminationCallback(Gap::Handle_t connectionHandle);
    void hvxCallback(const GattHVXCallbackParams *params);
    void onDataWriteCallback(const GattWriteCallbackParams *params);
    void onDataReadCallback(const GattReadCallbackParams *params);

    void flattrim();
    void takeoff();
    void land();
    void fly(int8_t roll, int8_t pitch, int8_t yaw, int8_t vertical);
    void turn_degrees(int16_t degrees);
    void checkPing();
    bool isFlying();

  private:
    void discoveredServiceCallback(const DiscoveredService *service);
    void discoveredCharacteristicCallback(const DiscoveredCharacteristic *chars);
    void discoveredCharsDescriptorCallback(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params);
    void discoveredDescTerminationCallback(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params);
    void discoverCharacteristicDescriptors();
    int getCharsIdxByHandle(GattAttribute::Handle_t handle);

    int handshake();
    void navigationDataState();

    GattClient& gattClient;

    enum FLYING_STATE {
      FS_LANDED = 0, FS_TAKINGOFF, FS_HOVERING, FS_FLYING, FS_LANDING,
      FS_EMERGENCY, FS_ROLLING, FS_INIT
    };
    enum FLYING_STATE flyingState;

    OnReadyCallback_t onReadyCallback;

    // to write:
    //  flight_params('fa0a'), command('fa0b'), emergency('fa0c')
    // notify (needs to subscribe as handshake):
    //  battery('fb0f'), flight_status('fb0e'), ack_for_command('fb1b'),
    //  ack_for_emergency('fb1c'),
    //  ftp channels('fd22', 'fd23', 'fd24', 'fd52', 'fd53', 'fd54')
    static const int SIZE_CHARS = 13;

    DiscoveredCharacteristic           dchars[SIZE_CHARS];
    DiscoveredCharacteristicDescriptor dchar_descs[SIZE_CHARS];

    int discovering_char_desc;
    int subscribing_char_desc;
    uint8_t steps_flight_params; // step count for fa0a
    uint8_t steps_command;       // step count for fa0b
    uint8_t steps_emergency;     // step count for fa0c

    uint32_t lastWriteMillis;
};
