#include "vision_comms.hpp"

namespace src::serial
{
VisionComms::VisionComms(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COMMS_RX_UART_PORT),
      lastAimData(),
      chassisOdometry(nullptr),
      chassisAutoDrive(nullptr),
      pitchMotor(nullptr)
{
}

VisionComms::~VisionComms() {}

void VisionComms::initializeCV()
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    drivers->uart.init<VISION_COMMS_TX_UART_PORT, VISION_COMMS_BAUD_RATE>();
}

void VisionComms::initializeUartDelays()
{
    sendHealthMsgTimeout.stop();
    sendRefTurretDataMsgTimeout.stop();
    sendRobotIDMsgTimeout.stop();
    sendOdometryMsgTimeout.stop();
    messageOffsetInitializationTimeout.restart(TIME_BEFORE_UART_START);
}

void VisionComms::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    switch (completeMessage.messageType)
    {
        case MessageType::TURRET_AIM_DATA:
        {
            decodeToTurretAimData(completeMessage);
            return;
        }
        case MessageType::ROBOT_ID:
        {
            sendRobotIdMessage();
            return;
        }
        case MessageType::ALIVE:
        {
            cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
            return;
        }
        case MessageType::ODOMETRY:
        {
            decodeToOdometeryData(completeMessage);
            return;
        }

        case MessageType::AUTO_PATH:
        {
            decodeToAutoPathData(completeMessage);
            return;
        }

        case MessageType::VISION_LOCALIZATION:
        {
            decodeToVisionAprilTagLocalization(completeMessage);
            return;
        }

        default:
            break;
    }
}

bool VisionComms::decodeToTurretAimData(const ReceivedSerialMessage& message)
{
    int curreIndex = 0;
    for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
    {
        if ((curreIndex + sizeof(TurretAimData) - sizeof(float) * 2 - 2) >
            message.header.dataLength)
        {
            return false;  // Not enough data for another turret aim data
        }

        TurretAimData& aimData = lastAimData[i];
        memcpy(&aimData.yaw, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        memcpy(&aimData.pitch, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        if (aimData.yaw == 0 && aimData.pitch == 0)
        {
            aimDataUpdated[i] = false;
        }
        else
        {
            aimDataUpdated[i] = true;
        }

        memcpy(&aimData.distance, &message.data[curreIndex], sizeof(float));
        curreIndex += sizeof(float);

        memcpy(&aimData.robotId, &message.data[curreIndex], sizeof(aimData.robotId));
        curreIndex += sizeof(aimData.robotId);

        if (aimData.distance != 0)
        {
            auto it = plateLookup.find(uint8_t(aimData.robotId));
            if (it != plateLookup.end())
            {
                PlateDims dims = it->second;
                aimData.maxErrorYaw = atan((dims.width / 2) / aimData.distance);
                aimData.maxErrorPitch = atan((dims.height / 2) / aimData.distance);
            }
        }
    }
    return true;
}

bool VisionComms::decodeToOdometeryData(const ReceivedSerialMessage& message)
{
    if (sizeof(OdometryData) > message.header.dataLength)
    {
        return false;
    }

    OdometryData cleanData;

    std::memcpy(&cleanData, message.data, sizeof(OdometryData));

    // chassisOdometry->setGlobalPosition({cleanData.chassis_data.pos_x,
    // cleanData.chassis_data.pos_y});

    return true;
}

bool VisionComms::decodeToAutoPathData(const ReceivedSerialMessage& message)
{
    assert(chassisAutoDrive != nullptr);

    if (sizeof(CubicBezier::CurveData) > message.header.dataLength)
    {
        return false;
    }

    CubicBezier::CurveData wireData;

    std::memcpy(&wireData, message.data, sizeof(CubicBezier::CurveData));

    CubicBezier* newCurve = new CubicBezier(wireData);

    chassisAutoDrive->resetPath();
    chassisAutoDrive->setCurve(newCurve);

    return true;
}

bool VisionComms::decodeToVisionAprilTagLocalization(const ReceivedSerialMessage& message)
{
    assert(chassisOdometry != nullptr);

    if (sizeof(VisionComms::AprilTagLocalizationData) > message.header.dataLength)
    {
        return false;
    }

    VisionComms::AprilTagLocalizationData localizationData;
    std::memcpy(&localizationData, message.data, sizeof(VisionComms::AprilTagLocalizationData));
    chassisOdometry->setGlobalPosition({localizationData.posX, localizationData.posY});

    return true;
}

void VisionComms::sendMessage()
{  // TODO: make these depend on which robot type is selected to make sure that we only send what we
   // need
    if (messageOffsetInitializationTimeout.isExpired())
    {
        sendRobotOdometry();
        sendRobotIdMessage();
        sendHealthData();
        sendTurretRefData();
    }
    else
    {
        if (sendOdometryMsgTimeout.isStopped() &&
            messageOffsetInitializationTimeout.timeRemaining() < TIME_BEFORE_SENDING_ODOMETRY_MSG)
        {
            sendOdometryMsgTimeout.restart();
        }

        if (sendRobotIDMsgTimeout.isStopped() &&
            messageOffsetInitializationTimeout.timeRemaining() < TIME_BEFORE_SENDING_ROBOT_ID_MSG)
        {
            sendRobotIDMsgTimeout.restart();
        }

        if (sendHealthMsgTimeout.isStopped() &&
            messageOffsetInitializationTimeout.timeRemaining() < TIME_BEFORE_SENDING_HEALTH_MSG)
        {
            sendHealthMsgTimeout.restart();
        }

        if (sendRefTurretDataMsgTimeout.isStopped() &&
            messageOffsetInitializationTimeout.timeRemaining() <
                TIME_BTWN_SENDING_REF_TURRET_DATA_MSG)
        {
            sendRefTurretDataMsgTimeout.restart();
        }
    }
}

void VisionComms::sendRobotIdMessage()
{
    if (sendRobotIDMsgTimeout.execute())
    {
        DJISerial::SerialMessage<1> robotTypeMessage;
        robotTypeMessage.messageType = MessageType::ROBOT_ID;
        robotTypeMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        robotTypeMessage.setCRC16();
        drivers->uart.write(
            VISION_COMMS_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&robotTypeMessage),
            sizeof(robotTypeMessage));
    }
}

void VisionComms::sendRobotOdometry()
{
    if (sendOdometryMsgTimeout.execute())
    {
        assert(chassisOdometry != nullptr);
        assert(pitchMotor != nullptr);

        DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;

        odometryMessage.messageType = MessageType::ODOMETRY;

        OdometryData* data = reinterpret_cast<OdometryData*>(odometryMessage.data);

        modm::Vector2f global_pos = chassisOdometry->getPositionGlobal();
        modm::Vector2f global_vel = chassisOdometry->getVelocityGlobal();

        data->timestamp = tap::arch::clock::getTimeMicroseconds();

        // Chassis data
        // data->chassis_data.pos_x = global_pos.x;  // meters
        // data->chassis_data.pos_y = global_pos.y;  // meters
        // data->chassis_data.pos_z = 0;             // TODO: this assumes the robot is on level
        // ground. Odometry should support z for varied height fields

        data->chassis_data.vel_x = global_vel.x;  // meters/second
        data->chassis_data.vel_y = global_vel.y;  // meters/second
        // data->chassis_data.vel_z = 0;             // TODO: see z on position (it doesn't exist)

        // Turret Data
        data->turret_data.pitch = pitchMotor->getPositionWrapped();  // radians
        data->turret_data.yaw = drivers->bmi088.getYaw();            // radians
        data->turret_data.roll = drivers->bmi088.getRoll();          // radians

        // data->turret_data.pitch_vel = pitchMotor->getShaftRPM() / 60 * M_TWOPI;
        // data->turret_data.yaw_vel = drivers->bmi088.getGz();
        // data->turret_data.roll_vel = drivers->bmi088.getGx();

        odometryMessage.setCRC16();
        drivers->uart.write(
            VISION_COMMS_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&odometryMessage),
            sizeof(odometryMessage));
    }
}

void VisionComms::sendHealthData()
{
    if (sendHealthMsgTimeout.execute())
    {
        DJISerial::SerialMessage<sizeof(uint16_t)> message;

        message.messageType = MessageType::HEALTH;

        // save into the message
        message.data[0] = static_cast<uint16_t>(drivers->refSerial.getRobotData().currentHp);

        message.setCRC16();
        drivers->uart.write(
            VISION_COMMS_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&message),
            sizeof(message));
    }
}

void VisionComms::sendTurretRefData()
{
    if (sendRefTurretDataMsgTimeout.execute())
    {
        tap::communication::serial::RefSerialData::Rx::TurretData refTurretData =
            drivers->refSerial.getRobotData().turret;

        DJISerial::SerialMessage<sizeof(refTurretData)> message;

        message.messageType = MessageType::REF_TURRET_DATA;

        // Offset 0
        uint16_t bulletSpeed = static_cast<uint16_t>(refTurretData.bulletSpeed);
        std::memcpy(&message.data[0], &bulletSpeed, 2);

        // Offset 2
        uint16_t bullets17 = static_cast<uint16_t>(refTurretData.bulletsRemaining17);
        std::memcpy(&message.data[2], &bullets17, 2);

        // Offset 4
        uint16_t bullets42 = static_cast<uint16_t>(refTurretData.bulletsRemaining42);
        std::memcpy(&message.data[4], &bullets42, 2);

        // Offset 6
        uint16_t bulletType = static_cast<uint16_t>(refTurretData.bulletType);
        std::memcpy(&message.data[6], &bulletType, 2);

        // Offset 8
        uint16_t coolingRate = static_cast<uint16_t>(refTurretData.coolingRate);
        std::memcpy(&message.data[8], &coolingRate, 2);

        // Offset 10
        uint16_t firingFreq = static_cast<uint16_t>(refTurretData.firingFreq);
        std::memcpy(&message.data[10], &firingFreq, 2);

        // Offset 12
        uint16_t heat17_1 = static_cast<uint16_t>(refTurretData.heat17ID1);
        std::memcpy(&message.data[12], &heat17_1, 2);

        // Offset 14
        uint16_t heat17_2 = static_cast<uint16_t>(refTurretData.heat17ID2);
        std::memcpy(&message.data[14], &heat17_2, 2);

        // Offset 16
        uint16_t heat42 = static_cast<uint16_t>(refTurretData.heat42);
        std::memcpy(&message.data[16], &heat42, 2);

        // Offset 18
        uint16_t heatLimit = static_cast<uint16_t>(refTurretData.heatLimit);
        std::memcpy(&message.data[18], &heatLimit, 2);

        // Offset 20
        uint16_t launchTime =
            static_cast<uint16_t>(refTurretData.lastReceivedLaunchingInfoTimestamp);
        std::memcpy(&message.data[20], &launchTime, 2);

        // Offset 22
        uint16_t launchMech = static_cast<uint16_t>(refTurretData.launchMechanismID);
        std::memcpy(&message.data[22], &launchMech, 2);

        // Offset 24
        uint16_t yaw = static_cast<uint16_t>(refTurretData.yaw);
        std::memcpy(&message.data[24], &yaw, 2);

        message.setCRC16();
        drivers->uart.write(
            VISION_COMMS_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&message),
            sizeof(message));
    }
}

// Should do something like this (from ARUW)

// void VisionCoprocessor::sendRobotTypeData()
// {
//     if (sendRobotIdTimeout.execute())
//     {
//         DJISerial::SerialMessage<1> robotTypeMessage;
//         robotTypeMessage.messageType = CV_MESSAGE_TYPE_ROBOT_ID;
//         robotTypeMessage.data[0] =
//         static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
//         robotTypeMessage.setCRC16();
//         drivers->uart.write(
//             VISION_COPROCESSOR_TX_UART_PORT,
//             reinterpret_cast<uint8_t*>(&robotTypeMessage),
//             sizeof(robotTypeMessage));
//     }
// }

bool VisionComms::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

}  // namespace src::serial