// //#ifdef VTM13
// /*
//  * VTM13 iBUS receiver driver for Taproot.
//  * Compatible with VTM13 receiver and transmitter.
//  *
//  * Based on Remote.cpp style (object-based, Drivers passed in).
//  */

// #include "vtm13.hpp"

// #include "tap/architecture/clock.hpp"
// #include "tap/control/remote_map_state.hpp"
// #include "tap/drivers.hpp"

// namespace tap::communication::serial
// {
// void VTM13::initialize()
// {
//     drivers->uart.init<Uart::UartPort::Uart1, 921600, Uart::Parity::Disabled>();
// }

// void VTM13::read()
// {
//     if (tap::arch::clock::getTimeMilliseconds() - lastRead > VTM13_DISCONNECT_TIMEOUT)
//     {
//         connected = false;
//     }

//     uint8_t data;
//     while (drivers->uart.read(Uart::UartPort::Uart1, &data))
//     {
//         dataFrameBuffer[dataIndex] = data;
//         dataIndex++;
//         lastRead = tap::arch::clock::getTimeMilliseconds();
//     }

//     if (tap::arch::clock::getTimeMilliseconds() - lastRead > VTM13_READ_TIMEOUT)
//     {
//         clearDataBuffer();
//     }

//     if (dataIndex >= DATA_FRAME_BYTE_SIZE)
//     {
//         connected = true;
//         parseDataFrame();
//         clearDataBuffer();
//     }
// }

// void VTM13::clearDataBuffer()
// {
//     dataIndex = 0;
//     for (int i = 0; i < DATA_FRAME_BYTE_SIZE; i++)
//     {
//         dataFrameBuffer[i] = 0;
//     }
//     drivers->uart.discardReceiveBuffer(Uart::UartPort::Uart1);
// }

// void VTM13::reset()
// {
//     state = VTM13_State{};
//     clearDataBuffer();
// }

// bool VTM13::isConnected() const { return connected; }

// void VTM13::parseDataFrame()
// {
//     const uint8_t* rx = &dataFrameBuffer[2];

//     auto readBits = [&](int bitOffset, int length) -> uint32_t {
//         if (length == 0) return 0u;
//         int byteIndex = bitOffset / 8;
//         int bitIndex = bitOffset % 8;
//         int bitsToCover = bitIndex + length;
//         int bytesToRead = (bitsToCover + 7) / 8;

//         uint32_t acc = 0;
//         for (int i = 0; i < bytesToRead; ++i)
//         {
//             acc |= static_cast<uint32_t>(rx[byteIndex + i]) << (8 * i);
//         }

//         acc >>= bitIndex;
//         if (length >= 32) return acc;
//         uint32_t mask = (1u << length) - 1u;
//         return acc & mask;
//     };

//     // the payload (rx) starts at bit 16, so subtract 16 to get offsets relative to rx.
//     auto rel = [](int absoluteOffset) -> int { return absoluteOffset - 16; };

//     // Extract 11-bit channels, convert to signed by subtracting center (1024)
//     state.rightHorizontal = static_cast<int16_t>(readBits(rel(16), 11));
//     state.rightHorizontal -= 1024;

//     state.rightVertical = static_cast<int16_t>(readBits(rel(27), 11));
//     state.rightVertical -= 1024;

//     state.leftVertical = static_cast<int16_t>(readBits(rel(38), 11));
//     state.leftVertical -= 1024;

//     state.leftHorizontal = static_cast<int16_t>(readBits(rel(49), 11));
//     state.leftHorizontal -= 1024;

//     // Switches / buttons
//     state.mode = static_cast<uint8_t>(readBits(rel(60), 2));
//     state.pause = (readBits(rel(62), 1) != 0);
//     state.leftButton = (readBits(rel(63), 1) != 0);
//     state.rightButton = (readBits(rel(64), 1) != 0);

//     // Dial (11 bits, centered)
//     state.dial = static_cast<int16_t>(readBits(rel(65), 11));
//     state.dial -= 1024;

//     state.trigger = (readBits(rel(76), 1) != 0);

//     // Mouse movements (signed 16-bit - interpret as two's complement)
//     state.mouseX = static_cast<int16_t>(readBits(rel(80), 16));
//     state.mouseY = static_cast<int16_t>(readBits(rel(96), 16));
//     state.mouseScroll = static_cast<int16_t>(readBits(rel(112), 16));

//     // Mouse buttons (2 bits each; store as small integers)
//     state.mouseLeft = static_cast<uint8_t>(readBits(rel(128), 2));
//     state.mouseRight = static_cast<uint8_t>(readBits(rel(130), 2));
//     state.mouseMiddle = static_cast<uint8_t>(readBits(rel(132), 2));

//     // Keyboard (16 bits)
//     state.keyboard = static_cast<uint16_t>(readBits(rel(136), 16));

//     // CRC (16 bits at the end of frame)
//     state.crc = static_cast<uint16_t>(readBits(rel(152), 16));

//     tap::control::RemoteMapState mapState;
//     mapState.initKeys(remote.key);
//     mapState.updateState(*this);
//     drivers->commandMapper.handleKeyStateChange(mapState);
//     drivers->commandMapper.pollTriggerBindings();

//     drivers->commandMapper.handleKeyStateChange(
//         state.keyboard,
//         state.leftButton ? tap::communication::serial::Remote::SwitchState::DOWN
//                          : tap::communication::serial::Remote::SwitchState::UP,
//         state.rightButton ? tap::communication::serial::Remote::SwitchState::DOWN
//                           : tap::communication::serial::Remote::SwitchState::UP,
//         state.mouseLeft != 0,
//         state.mouseRight != 0);

//     updateCounter++;
// }

// }  // namespace tap::communication::serial