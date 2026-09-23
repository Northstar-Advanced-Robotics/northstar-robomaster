#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/client_display/graphics_objects/graphics_container.hpp"
#include "modm/processing/protothread/protothread.hpp"

#include "drivers.hpp"

namespace src::control::client_display
{
class UISubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{  // go to ::modm::pt::Protothread to learn about protothreads
public:
    using Color =
        tap::communication::serial::RefSerialData::Tx::GraphicColor;   // makes it so you can use
                                                                       // UISubsystem::Color::CYAN
    static constexpr uint16_t SCREEN_WIDTH = 1920;                     // pixels. x=0 is left
    static constexpr uint16_t SCREEN_HEIGHT = 1080;                    // pixels. y=0 is bottom
    static constexpr uint16_t HALF_SCREEN_WIDTH = SCREEN_WIDTH / 2;    // pixels
    static constexpr uint16_t HALF_SCREEN_HEIGHT = SCREEN_HEIGHT / 2;  // pixels

private:  // Private Variables
    tap::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
    tap::arch::MilliTimeout delayTimeout;  // for not sending things too fast and dropping packets

    static uint32_t currGraphicName;  // for getUnusedGraphicName
    static int8_t currLayer;

    // for protothread
    bool needToClearAllLayers = true;
    int8_t timesResetIteration = 0;
    static constexpr int TARGET_NUM_OBJECTS =
        7;  // could change this to test, but don't make this larger than 7
    GraphicsObject* objectsToSend[TARGET_NUM_OBJECTS];
    int8_t graphicsIndex = 0;
    int8_t innerGraphicsIndex = 0;
    int8_t numToSend = 0;
    int8_t layerToClear = 0;
    GraphicsObject* nextGraphicsObject = nullptr;
    tap::communication::serial::RefSerialData::Tx::DeleteGraphicLayerMessage
        messageDel;  // only for RefSerialData::Tx::getWaitTimeAfterGraphicSendMs
    tap::communication::serial::RefSerialData::Tx::Graphic1Message message1;
    tap::communication::serial::RefSerialData::Tx::Graphic2Message message2;
    tap::communication::serial::RefSerialData::Tx::Graphic5Message message5;
    tap::communication::serial::RefSerialData::Tx::Graphic7Message message7;
    tap::communication::serial::RefSerialData::Tx::GraphicCharacterMessage messageCharacter;

    // when get UIDrawCommand, it should set this
    GraphicsContainer* topLevelContainer = nullptr;

    static constexpr int NUM_LAYERS = 10;  // layers 0-9
    int layersState[NUM_LAYERS];           // 0 is clear, 1 is not clear, 2 is needs cleared

public:  // Public Methods
    UISubsystem(tap::Drivers* driver);
    ~UISubsystem() {}  // Intentionally blank

    /*
     * Call this function once, outside of the main loop.
     * It registers the subsystem.
     */
    void initialize();

    void refresh() override;

    static uint32_t getUnusedGraphicName();

    /* Returns -1 if there aren't any unused layers
     * Only call this if you plan on calling deleteAndHideLayer */
    static int8_t getUnusedLayer();

    /* Marks a layer to be cleared next time the UISubsystem gets a chance.
     * Will also mark items on this layer as hidden to make sure they don't immediately get
     * drawn again. */
    static void clearAndHideLayer(int8_t layer);

    /*
     * Puts name into array (changing it in place), and returns array
     */
    static uint8_t* formatGraphicName(uint8_t array[3], uint32_t name);

    void setTopLevelContainer(GraphicsContainer* container);

    static void fixAngle(uint16_t* a)
    {
        *a %= 360;  // set a to the remainder after dividing by 360, so if it was 361 it would
                    // now be 1
    }

private:         // Private Methods
    bool run();  // for protothread
};
}  // namespace src::control::client_display