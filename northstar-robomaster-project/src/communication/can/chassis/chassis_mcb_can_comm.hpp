/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-turret-mcb.
 *
 * aruw-turret-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-turret-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-turret-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

 #ifndef CHASSIS_MCB_CAN_COMM_HPP_
 #define CHASSIS_MCB_CAN_COMM_HPP_
 
 #include "tap/architecture/periodic_timer.hpp"
 #include "tap/architecture/timeout.hpp"
 #include "tap/communication/can/can_rx_listener.hpp"
 #include "tap/communication/gpio/digital.hpp"
 
 #include "modm/architecture/interface/register.hpp"
 
 namespace src {
    class Drivers;
}
 
 /**
  * The CAN link from the turret MCB (the follower) to the chassis MCB (the leader).
  *
  * The turret board owns the turret IMU and the limit switch, while the chassis board runs the
  * commands that need them, so this class streams IMU axis data and turret status up to the chassis
  * and receives back a bitmask of requests (open the hopper, recalibrate the IMU, turn the laser
  * on).
  *
  * Because the two boards keep independent clocks, a periodic four-timestamp exchange estimates the
  * offset between them so that data sent from the turret can be interpreted on the chassis'
  * timebase; see `getTimeRelativeToLeaderMicroseconds`.
  */
 class ChassisMcbCanComm
 {
 public:
     /**
      * Requests the chassis MCB can raise in a `CHASSIS_MCB_COMMAND_RX_CAN_ID` message. Each
      * request occupies one bit of the single command byte.
      */
     enum class RxCommandMsgBitmask : uint8_t
     {
         OPEN_HOPPER = modm::Bit0,
         RECALIBRATE_IMU = modm::Bit1,
         TURN_LASER_ON = modm::Bit2,
     };
     MODM_FLAGS8(RxCommandMsgBitmask);
 
     /**
      * @param[in] drivers The global drivers object, used for CAN access and timing.
      */
     ChassisMcbCanComm(tap::Drivers* drivers);
     DISALLOW_COPY_AND_ASSIGN(ChassisMcbCanComm);
 
     /**
      * Attaches the CAN receive listeners for the command and time synchronization messages. Must
      * be called before any message will be received.
      */
     void init();
 
     /**
      * @return `true` while the chassis MCB is asking for the hopper lid to be open.
      */
     inline bool getOpenHopperRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::OPEN_HOPPER);
     }
 
     /**
      * @return `true` if the chassis MCB has requested an IMU recalibration. Stays set until
      *      `clearImuRecalibration` is called, so the request cannot be missed between iterations.
      */
     inline bool getImuRecalibrationRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::RECALIBRATE_IMU);
     }
 
     /**
      * @return `true` while the chassis MCB is asking for the aiming laser to be on.
      */
     inline bool getLaserOnRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::TURN_LASER_ON);
     }
 
     /**
      * Acknowledges the pending IMU recalibration request. Call once the recalibration has
      * actually been started, otherwise it will be triggered again on the next iteration.
      */
     inline void clearImuRecalibration()
     {
         commandMsgBitmask.reset(RxCommandMsgBitmask::RECALIBRATE_IMU);
     }
 
     /**
      * @return `true` if a message has been received from the chassis MCB within the last
      *      `DISCONNECT_TIMEOUT_PERIOD` milliseconds. Returns `false` before the first message
      *      ever arrives.
      */
     inline bool isConnected() const
     {
         return !chassisMcbConnectedTimeout.isExpired() && !chassisMcbConnectedTimeout.isStopped();
     }
 
     /**
      * @return The current time in microseconds, expressed on the chassis MCB's clock.
      */
     inline uint32_t getLeaderTimeMicroseconds()
     {
         return getTimeRelativeToLeaderMicroseconds(tap::arch::clock::getTimeMicroseconds());
     }
 
     /**
      * Converts a timestamp taken on this board's clock into the chassis MCB's timebase using the
      * most recent synchronization exchange.
      *
      * @param[in] followerTime A time in microseconds as measured on this board.
      * @return The same instant expressed in chassis MCB microseconds.
      */
     inline uint32_t getTimeRelativeToLeaderMicroseconds(uint32_t followerTime)
     {
         return followerTime + lastestSyncData.getCorrelationTimeOffsetMicroseconds();
     }
 
     /**
      * Sends the turret IMU's x, y, and z axis data to the chassis MCB as three CAN messages. Call
      * at the IMU's sample rate.
      */
     void sendIMUData();
 
     /**
      * Sends a clock synchronization request if one is due, timestamping it so the round trip can
      * be used to estimate the offset between the two boards' clocks. Call once per iteration; the
      * request itself is rate limited to `SYNC_REQUEST_PERIOD`.
      */
     void sendSynchronizationRequest();
 
     /**
      * Reports turret-side status the chassis MCB cannot read for itself.
      *
      * @param[in] limitSwitchPin The pin the turret's limit switch is wired to, sampled and sent
      *      as part of the status message.
      */
     void sendTurretStatusData(tap::gpio::Digital::InputPin limitSwitchPin);
 
 private:
     /// CAN IDs used by this link. Must match the IDs the chassis MCB is listening on.
     enum CanIDs
     {
         SYNC_TX_CAN_ID = 0x1f8,
         SYNC_RX_CAN_ID = 0x1f9,
         TURRET_STATUS_TX_CAN_ID = 0x1fa,
         X_AXIS_TX_CAN_ID = 0x1fb,
         Y_AXIS_TX_CAN_ID = 0x1fc,
         Z_AXIS_TX_CAN_ID = 0x1fd,
         CHASSIS_MCB_COMMAND_RX_CAN_ID = 0x1fe,
     };
 
     /**
      * The four timestamps of one clock synchronization round trip: when this board sent the
      * request, when the chassis received it, when the chassis replied, and when this board
      * received the reply.
      */
     struct SyncData
     {
         int64_t followerReqTimeUs;
         int64_t leaderReceiveReqTimeUs;
         int64_t leaderResponseTimeUs;
         int64_t followerReceiveResponseTimeUs;
         /**
          * @return How far ahead the chassis MCB's clock runs, in microseconds. Averaging the two
          *      legs of the round trip cancels out the (assumed symmetric) transmission delay.
          */
         int64_t getCorrelationTimeOffsetMicroseconds()
         {
             return ((leaderReceiveReqTimeUs - followerReqTimeUs) +
                     (leaderResponseTimeUs - followerReceiveResponseTimeUs)) /
                    2;
         }
     };
 
     /**
      * One IMU axis packed for transmission: angle and angular velocity as fixed point, linear
      * acceleration, and a sequence number the receiver uses to detect dropped messages.
      */
     struct AxisMessageData
     {
         int16_t angleFixedPoint;
         int16_t angleAngularVelocityRaw;
         int16_t linearAcceleration;
         uint8_t seq;
     } modm_packed;
 
     /// The CAN bus shared with the chassis MCB.
     static constexpr tap::can::CanBus CHASSIS_IMU_CAN_BUS = tap::can::CanBus::CAN_BUS2;
     /// Milliseconds without a received message after which the chassis MCB is considered
     /// disconnected.
     static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 1000;
     /// Milliseconds between clock synchronization requests.
     static constexpr uint32_t SYNC_REQUEST_PERIOD = 100;
 
     /// Signature of the member functions that handle a received CAN message.
     using CanCommListenerFunc = void (ChassisMcbCanComm::*)(const modm::can::Message& message);
 
     /**
      * Adapts taproot's CAN receive listener to a member function of the enclosing
      * `ChassisMcbCanComm`, so one handler class can serve every message ID this link listens on.
      */
     class MainMcbRxHandler : public tap::can::CanRxListener
     {
     public:
         /**
          * @param[in] drivers The global drivers object.
          * @param[in] id The CAN ID to listen for.
          * @param[in] cB The CAN bus to listen on.
          * @param[in] msgHandler The object `funcToCall` is invoked on.
          * @param[in] funcToCall The member function that handles a matching message.
          */
         MainMcbRxHandler(
             tap::Drivers* drivers,
             uint32_t id,
             tap::can::CanBus cB,
             ChassisMcbCanComm* msgHandler,
             CanCommListenerFunc funcToCall);
         /**
          * Forwards a received message to the configured member function.
          *
          * @param[in] message The received CAN message.
          */
         void processMessage(const modm::can::Message& message) override;
 
     private:
         ChassisMcbCanComm* msgHandler;
         CanCommListenerFunc funcToCall;
     };
 
     tap::Drivers* drivers;
 
     /// The most recently received set of chassis MCB requests.
     RxCommandMsgBitmask_t commandMsgBitmask;
 
     /// Restarted on every received message; expires when the chassis MCB stops responding.
     tap::arch::MilliTimeout chassisMcbConnectedTimeout;
 
     /// Paces the outgoing clock synchronization requests.
     tap::arch::PeriodicMilliTimer timeSyncLoopTimeout{SYNC_REQUEST_PERIOD};
 
     /// The synchronization exchange currently in flight.
     SyncData currProcessingSyncData;
     /// The most recently completed synchronization exchange, used to compute the clock offset.
     SyncData lastestSyncData;
 
     MainMcbRxHandler chassisCommandHandler;
     MainMcbRxHandler timeSyncronizationHandler;
 
     /// Incremented on each IMU message so the receiver can detect drops.
     uint32_t imuDataSeq = 0;
     /// Divides down the send rate to blink a status LED as a visual heartbeat.
     uint32_t blinkCounter = 0;
 
     /**
      * Packs and sends one IMU axis as an `AxisMessageData` message.
      *
      * @param[in] messageID The axis' CAN ID.
      * @param[in] angle The angle about the axis, in radians.
      * @param[in] angularVelocity The angular velocity about the axis, in radians/second.
      * @param[in] linearAcceleration The linear acceleration along the axis, in m/s^2.
      */
     void sendAxisData(
         CanIDs messageID,
         float angle,
         float angularVelocity,
         float linearAcceleration);
 
     /**
      * Stores a received command bitmask and refreshes the connection timeout.
      *
      * @param[in] message The received command message.
      */
     void handleChassisCommandMessage(const modm::can::Message& message);
     /**
      * Completes a synchronization round trip by recording the chassis' timestamps and the local
      * receive time, then promoting the result to `lastestSyncData`.
      *
      * @param[in] message The received synchronization reply.
      */
     void handleTimeSynchronizationMessage(const modm::can::Message& message);
 };
 
 #endif  // CHASSIS_MCB_CAN_COMM_HPP_
 