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
/**
 * @ingroup communication
 *
 * The UART link between the MCB and the onboard vision computer.
 *
 * Both halves of the link run over a single UART port using taproot's `DJISerial` framing. Inbound
 * messages (aim data, odometry corrections, auto-drive paths, and remote input forwarded from the
 * vision computer) arrive through `messageReceiveCallback`; outbound messages (odometry, health,
 * robot ID) are pushed by periodic calls to `sendMessage`.
 *
 * The link is considered offline once no message of any kind has been received for
 * `TIME_OFFLINE_CV_AIM_DATA_MS`; see `isCvOnline`.
 */
class VisionComms : public tap::communication::serial::DJISerial
{
public:
    static constexpr size_t VISION_COMMS_BAUD_RATE = 115'200;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;
    static constexpr tap::communication::serial::Uart::UartPort VISION_COMMS_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart1;

    /**
     * Message type IDs carried in the `DJISerial` header. Both sides of the link must agree on
     * these values.
     */
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
        VISION_LOCALIZATION = 8,
        FLY_SKY_DATA = 9,
        VT13_DATA = 10,
        RESTART_DETECTOR = 11
    };

    /**
     * A snapshot of referee system state forwarded to the vision computer.
     *
     * Mirrors the subset of `tap::communication::serial::RefSerialData` that the vision computer
     * needs, flattened into a single struct so it can be serialized in one message.
     */
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

    /**
     * Turret orientation and yaw rate, in radians and radians/second, sent as part of
     * `OdometryData`.
     */
    struct TurretOdometryData
    {
        float pitch;
        float yaw;
        float roll;

        // float pitch_vel;
        float yaw_vel;
        // float roll_vel;

    } modm_packed;

    /**
     * Chassis velocity in meters/second, sent as part of `OdometryData`.
     */
    struct ChassisOdometryData
    {
        // float pos_x;
        // float pos_y;
        // float pos_z;

        float vel_x;
        float vel_y;
        // float vel_z;

    } modm_packed;

    /**
     * The full odometry packet sent to the vision computer, timestamped so that vision can
     * compensate for link latency when computing an aim solution.
     */
    struct OdometryData
    {
        uint32_t timestamp;
        ChassisOdometryData chassis_data;
        TurretOdometryData turret_data;
    } modm_packed;

    /**
     * An absolute field pose computed by the vision computer from an AprilTag sighting, used to
     * correct accumulated drift in the chassis odometry.
     */
    struct AprilTagLocalizationData
    {
        float posX;
        float posY;
        float heading;
        uint32_t timestamp;
    } modm_packed;

    /// Odometry to correct when a localization message arrives. `nullptr` until `attachOdometry`.
    src::chassis::ChassisOdometry* chassisOdometry;

    /// Auto drive to feed paths to. `nullptr` until `attachAutoDrive`.
    src::chassis::ChassisAutoDrive* chassisAutoDrive;

    /// Remote to inject forwarded FlySky/VT13 input into. `nullptr` until `attachRemote`.
    tap::communication::serial::Remote* remote;

    /// Time in milliseconds of the last VT13 remote packet, used to detect that remote dropping
    /// out.
    uint32_t lastReadVT13 = 0;
    /// Time in milliseconds of the last FlySky remote packet, used to detect that remote dropping
    /// out.
    uint32_t lastReadFlySky = 0;

    /// Milliseconds without a packet after which a forwarded remote is marked disconnected.
    uint16_t REMOTE_TIMEOUT = 1000;

    /// Pitch motor sampled when reporting turret odometry. `nullptr` until `attachPitchMotor`.
    tap::motor::DjiMotor* pitchMotor;

    /**
     * An aim solution produced by the vision computer.
     *
     * `yaw` and `pitch` are the angles the turret should be driven to, in radians. `maxErrorYaw`
     * and `maxErrorPitch` bound how far off aim the turret may be while still being considered on
     * target, and are derived from the apparent size of the plate at `distance`.
     */
    struct TurretAimData
    {
        float yaw;
        float pitch;
        float distance;
        tap::communication::serial::RefSerialData::RobotId robotId;
        float maxErrorYaw;
        float maxErrorPitch;
    };

    /// Physical width and height of an armor plate, in meters.
    struct PlateDims
    {
        float width;
        float height;
    };

    /// Armor plate dimensions keyed by robot ID, used to convert plate size into an aim tolerance.
    std::unordered_map<int, PlateDims> plateLookup{
        {1, {.2f, .15f}},   // hero plate dimentions in mm
        {7, {.15f, .15f}},  // sentry plate dimentions in mm
        {3, {.15f, .15f}},  // don't know id 3 is correct
    };

    /**
     * @param[in] drivers The global drivers object, used for UART access and remote state.
     */
    VisionComms(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionComms);
    mockable ~VisionComms();

    /**
     * Opens the vision UART port and starts the offline timeout. Must be called before any
     * message can be sent or received.
     */
    mockable void initializeCV();

    /**
     * Staggers the periodic outbound messages so that odometry, health, and turret ref data are
     * not all transmitted in the same control loop iteration, which would otherwise saturate the
     * UART. Call once at startup, after `initializeCV`.
     */
    mockable void initializeUartDelays();

    /**
     * Dispatches a fully received message to the decoder for its `MessageType` and refreshes the
     * offline timeout. Called by `DJISerial` from the receive path; not intended to be called
     * directly.
     *
     * @param[in] completeMessage The message whose header and CRC have already been validated.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    /**
     * @return `true` if a message has been received from the vision computer within the last
     *      `TIME_OFFLINE_CV_AIM_DATA_MS` milliseconds.
     */
    mockable bool isCvOnline() const;

    /**
     * @param[in] turretID Index of the turret to query.
     * @return The most recent aim solution received for the given turret. The contents are stale
     *      unless `isAimDataUpdated` returns `true`.
     */
    mockable inline const TurretAimData& getLastAimData(uint8_t turretID = 0) const
    {
        return lastAimData[turretID];
    }

    /**
     * @param[in] turretID Index of the turret to query.
     * @return `true` if the last aim message contained a valid solution for the given turret,
     *      i.e. vision is currently tracking a target for it.
     */
    mockable inline bool isAimDataUpdated(uint8_t turretID = 0) const
    {
        return aimDataUpdated[turretID];
    }

    /**
     * @param[in] turretID Unused; retained so callers can pass a turret index uniformly.
     * @return `true` if any turret currently has a vision target.
     */
    mockable inline bool getSomeTurretHasTarget(uint8_t turretID = 0) const
    {
        return isAimDataUpdated();
    }

    /**
     * Registers the odometry that AprilTag localization messages will correct.
     *
     * @param[in] chassisOdometry The odometry to update. Localization messages are ignored while
     *      this is unset.
     */
    mockable inline void attachOdometry(src::chassis::ChassisOdometry* chassisOdometry)
    {
        this->chassisOdometry = chassisOdometry;
    }

    /**
     * Registers the auto drive that received path messages will be handed to.
     *
     * @param[in] chassisAutoDrive The auto drive to update. Path messages are ignored while this
     *      is unset.
     */
    mockable inline void attachAutoDrive(src::chassis::ChassisAutoDrive* chassisAutoDrive)
    {
        this->chassisAutoDrive = chassisAutoDrive;
    }

    /**
     * Registers the remote that FlySky and VT13 input forwarded over this link will be written
     * into.
     *
     * @param[in] remote The remote to update. Forwarded remote messages are ignored while this is
     *      unset.
     */
    mockable inline void attachRemote(tap::communication::serial::Remote* remote)
    {
        this->remote = remote;
    }

    /**
     * Registers the pitch motor sampled when building outbound turret odometry.
     *
     * @param[in] pitchMotor The turret's pitch motor.
     */
    mockable inline void attachPitchMotor(tap::motor::DjiMotor* pitchMotor)
    {
        this->pitchMotor = pitchMotor;
    }

    /**
     * Sends whichever periodic messages are due this iteration and, if the operator is holding
     * `ctrl+x`, a request that the vision computer restart its detector. Call once per control
     * loop iteration.
     */
    mockable void sendMessage();

private:
    /// Milliseconds without any received message after which the vision computer is considered
    /// offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1'000;

    /// Restarted on every received message; expires when the vision computer goes offline.
    tap::arch::MilliTimeout cvOfflineTimeout;

    /// The most recent aim solution received for each turret.
    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    /// Whether the corresponding entry of `lastAimData` holds a live target rather than a stale
    /// one.
    bool aimDataUpdated[control::turret::NUM_TURRETS] = {};

    /// Asks the vision computer to restart its plate detector.
    mockable void sendCvRestartMessage();

    /// Replies to a `ROBOT_ID` request with this robot's referee system ID, so vision knows which
    /// team's plates to target.
    mockable void sendRobotIdMessage();

    /// Sends the current chassis velocity and turret orientation so vision can compensate for
    /// robot motion when computing an aim solution.
    mockable void sendRobotOdometry();

    /// Sends the health of every robot on the field, used by the sentry to prioritize targets.
    mockable void sendHealthData();

    // mockable void sendTurretRefData();

    /**
     * Unpacks a `TURRET_AIM_DATA` message into `lastAimData` and updates `aimDataUpdated`.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was long enough to hold a solution for every turret.
     */
    bool decodeToTurretAimData(const ReceivedSerialMessage& message);

    /**
     * Unpacks an `ODOMETRY` message from the vision computer.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was well formed.
     */
    bool decodeToOdometeryData(const ReceivedSerialMessage& message);

    /**
     * Unpacks an `AUTO_PATH` message and hands the path to the attached auto drive.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was well formed and an auto drive is attached.
     */
    bool decodeToAutoPathData(const ReceivedSerialMessage& message);

    /**
     * Unpacks a `VISION_LOCALIZATION` message and corrects the attached odometry with the
     * absolute field pose it carries.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was well formed and odometry is attached.
     */
    bool decodeToVisionAprilTagLocalization(const ReceivedSerialMessage& message);

    /**
     * Unpacks a `FLY_SKY_DATA` message into the attached remote. Ignored while the VT13 remote is
     * connected, so the two forwarded remotes cannot fight over the same state.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was well formed.
     */
    bool decodeToFlySkyRemote(const ReceivedSerialMessage& message);

    /**
     * Unpacks a `VT13_DATA` message into the attached remote. Ignored while the FlySky remote is
     * connected.
     *
     * @param[in] message The message to decode.
     * @return `true` if the message was well formed.
     */
    bool decodeToVT13Remote(const ReceivedSerialMessage& message);
};
}  // namespace src::serial

#endif  // VISION_COMMS_HPP