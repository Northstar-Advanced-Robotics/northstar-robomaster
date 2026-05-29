// //#ifdef VTM13

// #ifndef VTM13_REMOTE_HPP
// #define VTM13_REMOTE_HPP

// #include <cstdint>

// #ifndef PLATFORM_HOSTED
// #include "modm/platform.hpp"
// #endif

// #include "tap/communication/serial/remote.hpp"
// #include "tap/util_macros.hpp"

// namespace tap
// {
// class Drivers;
// }

// struct VTM13ChannelOffset
// {
//     u_int8_t offset;  // bits
//     u_int8_t length;  // bits
// };

// struct VTM13_State
// {
//     // Stick channels (11-bit each, centered at 1024 in raw frame; we convert to signed).
//     int16_t rightHorizontal;  // Channel 0 (offset 16, 11 bits)
//     int16_t rightVertical;    // Channel 1 (offset 27, 11 bits)
//     int16_t leftVertical;     // Channel 2 (offset 38, 11 bits)
//     int16_t leftHorizontal;   // Channel 3 (offset 49, 11 bits)

//     // Switches / buttons
//     uint8_t mode;      // MODE_SWITCH (offset 60, 2 bits) => {0=C,1=N,2=S}
//     bool pause;        // PAUSE_BUTTON (offset 62, 1 bit)
//     bool leftButton;   // LEFT_BUTTON (offset 63, 1 bit)
//     bool rightButton;  // RIGHT_BUTTON (offset 64, 1 bit)
//     int16_t dial;      // DIAL (offset 65, 11 bits, centered at 1024)
//     bool trigger;      // TRIGGER (offset 76, 1 bit)

//     // Mouse (signed 16-bit values)
//     int16_t mouseX;       // MOUSE_X (offset 80, 16 bits) - signed
//     int16_t mouseY;       // MOUSE_Y (offset 96, 16 bits) - signed
//     int16_t mouseScroll;  // MOUSE_SCROLL (offset 112, 16 bits) - signed

//     // Mouse button bitfields (2 bits each according to the spec)
//     uint8_t mouseLeft;    // MOUSE_LEFT (offset 128, 2 bits)
//     uint8_t mouseRight;   // MOUSE_RIGHT (offset 130, 2 bits)
//     uint8_t mouseMiddle;  // MOUSE_MIDDLE (offset 132, 2 bits)

//     // Keyboard: 16-bit bitmask (offset 136, 16 bits).
//     uint16_t keyboard;  // KEYBOARD (offset 136, 16 bits)

//     // CRC read from the frame (raw)
//     uint16_t crc;  // CYCLIC_REDUNDANCY_CHECK (offset 152, 16 bits)
// };

// namespace tap::communication::serial
// {
// class VTM13
// {
// public:
//     static constexpr uint8_t DATA_FRAME_BYTE_SIZE = 21;
//     static constexpr uint32_t VTM13_DISCONNECT_TIMEOUT = 100;  // ms
//     static constexpr uint32_t VTM13_READ_TIMEOUT = 50;         // ms

//     VTM13(Drivers* drivers) : drivers(drivers) {}
//     DISALLOW_COPY_AND_ASSIGN(VTM13)
//     mockable ~VTM13() = default;

//     void initialize();
//     void read();
//     void reset();

//     bool isConnected() const;

//     mockable uint32_t getUpdateCounter() const;

// private:
//     uint32_t updateCounter = 0;

//     void parseDataFrame();

//     void clearDataBuffer();

//     Drivers* drivers;

//     uint8_t dataFrameBuffer[DATA_FRAME_BYTE_SIZE]{0};
//     tap::communication::serial::Remote::RemoteInfo remote;

//     bool connected = false;
//     uint32_t lastRead = 0;
//     uint8_t syncState = 0;
//     uint8_t dataIndex = 0;
// };

// }  // namespace tap::communication::serial

// #endif

// //#endif  // VTM13