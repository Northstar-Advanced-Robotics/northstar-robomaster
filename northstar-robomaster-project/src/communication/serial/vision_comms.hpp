#ifndef VISION_COMMS_HPP
#define VISION_COMMS_HPP

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_odometry.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/turret/constants/turret_constants.hpp"

#include "uart_constants.hpp"

namespace src::serial
{
class VisionComms : public tap::communication::serial::DJISerial
{
public:
    static constexpr size_t VISION_COMMS_BAUD_RATE = 115'200;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;
    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;

    enum MessageType : uint16_t
    {
        TURRET_AIM_DATA = 1,
        ROBOT_ID = 2,
        ALIVE = 3,
        ODOMETRY = 4,
        AUTO_PATH = 5,
        // REF_DATA = 6
        HEALTH = 6,
        REF_TURRET_DATA = 7,
        VISION_LOCALIZATION = 8
    };

    struct RefData
    {
        // tap::communication::serial::RefSerialData::Rx::GameData game_data;
        // tap::communication::serial::RefSerialData::Rx::RobotData robot_data;

        // Robot Data
        tap::communication::serial::RefSerialData::RobotId robotId;
        uint8_t robotLevel;
        uint16_t previousHp;

        uint16_t currentHp;
        uint16_t maxHp;
        tap::communication::serial::RefSerialData::Rx::RobotPower_t robotPower;
        tap::communication::serial::RefSerialData::Rx::ArmorId damagedArmorId;
        tap::communication::serial::RefSerialData::Rx::DamageType damageType;
        float receivedDps;
        tap::communication::serial::RefSerialData::Rx::ChassisData chassis;
        tap::communication::serial::RefSerialData::Rx::TurretData turret;
        tap::communication::serial::RefSerialData::Rx::RobotHpData allRobotHp;
        uint16_t remainingCoins;
        tap::communication::serial::RefSerialData::Rx::RobotBuffStatus robotBuffStatus;
        tap::communication::serial::RefSerialData::Rx::RFIDActivationStatus_t rfidStatus;

        // Game Data

        tap::communication::serial::RefSerialData::Rx::GameType gameType;
        tap::communication::serial::RefSerialData::Rx::GameStage gameStage;
        uint16_t stageTimeRemaining;
        uint64_t unixTime;
        tap::communication::serial::RefSerialData::Rx::GameWinner gameWinner;
        tap::communication::serial::RefSerialData::Rx::EventData eventData;
        tap::communication::serial::RefSerialData::Rx::SupplierAction supplier;
        tap::communication::serial::RefSerialData::Rx::DartInfo dartInfo;
        tap::communication::serial::RefSerialData::Rx::AirSupportData airSupportData;
        tap::communication::serial::RefSerialData::Rx::DartStationInfo dartStation;
        tap::communication::serial::RefSerialData::Rx::GroundRobotPositions positions;
        tap::communication::serial::RefSerialData::Rx::RadarMarkProgress radarProgress;
        tap::communication::serial::RefSerialData::Rx::SentryInfo sentry;
        tap::communication::serial::RefSerialData::Rx::RadarInfo radar;

        // uint8_t game_result;
        // float all_robot_hp[6];

        // uint8_t site_event_data;
        // uint8_t warning_data;
        // uint8_t dart_info;

        // uint8_t robot_status;
        // uint16_t power_and_heat;
        // float robot_position[2];
        // uint8_t robot_buff_status;
        // uint8_t receive_damage;
        // uint8_t projectile_launch;
        // uint16_t bullets_remain;
        // uint8_t rfid_status;
        // uint8_t dart_station_info[4];
        // float ground_robot_position[2];
        // uint8_t radar_progress;
        // uint8_t sentry_info;
        // uint8_t radar_info;
        // custom_data;
    };

    struct TurretOdometryData
    {
        float pitch;
        float yaw;
        float roll;

        // float pitch_vel;
        // float yaw_vel;
        // float roll_vel;

    } modm_packed;

    struct ChassisOdometryData
    {
        // float pos_x;
        // float pos_y;
        // float pos_z;

        float vel_x;
        float vel_y;
        // float vel_z;

    } modm_packed;

    struct OdometryData
    {
        uint32_t timestamp;
        ChassisOdometryData chassis_data;
        TurretOdometryData turret_data;
    } modm_packed;

    struct AprilTagLocalizationData
    {
        float posX;
        float posY;
    } modm_packed;

    src::chassis::ChassisOdometry* chassisOdometry;

    src::chassis::ChassisAutoDrive* chassisAutoDrive;

    tap::motor::DjiMotor* pitchMotor;

    struct TurretAimData
    {
        float yaw;
        float pitch;
        float distance;
        tap::communication::serial::RefSerialData::RobotId robotId;
        float maxErrorYaw;
        float maxErrorPitch;
    };

    struct PlateDims
    {
        float width;
        float height;
    };

    std::unordered_map<int, PlateDims> plateLookup{
        {1, {.2f, .15f}},   // hero plate dimentions in mm
        {7, {.15f, .15f}},  // sentry plate dimentions in mm
        {3, {.15f, .15f}},  // don't know id 3 is correct
    };

    VisionComms(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionComms);
    mockable ~VisionComms();

    mockable void initializeCV();

    mockable void initializeUartDelays();

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    mockable bool isCvOnline() const;

    mockable inline const TurretAimData& getLastAimData(uint8_t turretID = 1) const
    {
        return lastAimData[turretID];
    }

    mockable inline bool isAimDataUpdated(uint8_t turretID = 1) const
    {
        return aimDataUpdated[turretID];
    }

    mockable inline bool getSomeTurretHasTarget(uint8_t turretID = 1) const
    {
        return isAimDataUpdated();
    }

    mockable inline void attachOdometry(src::chassis::ChassisOdometry* chassisOdometry)
    {
        this->chassisOdometry = chassisOdometry;
    }

    mockable inline void attachAutoDrive(src::chassis::ChassisAutoDrive* chassisAutoDrive)
    {
        this->chassisAutoDrive = chassisAutoDrive;
    }

    mockable inline void attachPitchMotor(tap::motor::DjiMotor* pitchMotor)
    {
        this->pitchMotor = pitchMotor;
    }

    mockable void sendMessage();

private:
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1'000;

    tap::arch::MilliTimeout cvOfflineTimeout;

    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    bool aimDataUpdated[control::turret::NUM_TURRETS] = {};

    mockable void sendRobotIdMessage();

    mockable void sendRobotOdometry();

    mockable void sendHealthData();

    mockable void sendTurretRefData();

    bool decodeToTurretAimData(const ReceivedSerialMessage& message);

    bool decodeToOdometeryData(const ReceivedSerialMessage& message);

    bool decodeToAutoPathData(const ReceivedSerialMessage& message);

    bool decodeToVisionAprilTagLocalization(const ReceivedSerialMessage& message);
};
}  // namespace src::serial

#endif  // VISION_COMMS_HPP