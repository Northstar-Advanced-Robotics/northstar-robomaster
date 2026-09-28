#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "modm/processing/protothread/protothread.hpp"

#include "drivers.hpp"

namespace src::control::client_display::graphics
{
using namespace tap::communication::serial;

/**
 * @ingroup client_display
 *
 * Draws the operator HUD by sending graphics to the referee system's client display.
 *
 * The link to the server is slow and drops packets if written too quickly, so drawing is spread
 * across many control loop iterations: each pass walks the graphics tree, collects up to
 * `TARGET_NUM_OBJECTS` graphics that have changed, sends them as one message, and waits out the
 * server's required delay before the next. That wait is why this is a protothread; `run` resumes
 * where it left off rather than blocking the control loop.
 *
 * Graphics are organized into layers, which the server can clear wholesale. Layers and graphic
 * names are handed out statically, so every graphic on the robot gets a distinct identity no
 * matter which part of the code created it.
 */
class UISubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{  // go to ::modm::pt::Protothread to learn about protothreads
public:
    /// Shorthand for the server's color enum, so callers can write `UISubsystem::Color::CYAN`.
    using Color =
        RefSerialData::Tx::GraphicColor;  // makes it so you can use UISubsystem::Color::CYAN
    /// Client display width in pixels. X increases to the right from 0 at the left edge.
    static constexpr uint16_t SCREEN_WIDTH = 1920;  // pixels. x=0 is left
    /// Client display height in pixels. Y increases upward from 0 at the bottom edge.
    static constexpr uint16_t SCREEN_HEIGHT = 1080;  // pixels. y=0 is bottom
    /// The horizontal center of the screen, in pixels.
    static constexpr uint16_t HALF_SCREEN_WIDTH = SCREEN_WIDTH / 2;  // pixels
    /// The vertical center of the screen, in pixels.
    static constexpr uint16_t HALF_SCREEN_HEIGHT = SCREEN_HEIGHT / 2;  // pixels

private:  // Private Variables
    /// The global drivers object.
    tap::Drivers* drivers;
    /// Sends the assembled graphic messages to the referee system.
    RefSerialTransmitter refSerialTransmitter;
    /// Enforces the server's required gap between messages; sending faster drops packets.
    tap::arch::MilliTimeout delayTimeout;  // for not sending things too fast and dropping packets

    /// The next graphic name to hand out. Static, so names stay unique across every graphic on the
    /// robot.
    static uint32_t currGraphicName;  // for getUnusedGraphicName
    /// The next layer to hand out. Static, for the same reason as `currGraphicName`.
    static int8_t currLayer;

    // for protothread
    /// Set at startup so the display is wiped before anything is drawn, since the server may still
    /// hold graphics from a previous run.
    bool needToClearAllLayers = true;
    /// How many times the graphics tree has been rewound, used to pace full traversals.
    int8_t timesResetIteration = 0;
    /// How many graphics to pack into one message. The server accepts batches of 1, 2, 5, or 7,
    /// and each message carries the same fixed overhead, so 7 wastes the least bandwidth per
    /// graphic. Do not raise this above 7.
    static constexpr int TARGET_NUM_OBJECTS =
        7;  // could change this to test, but don't make this larger than 7
    /// The graphics collected for the message currently being assembled.
    GraphicsObject* objectsToSend[TARGET_NUM_OBJECTS];
    /// How many graphics have been collected into `objectsToSend` so far.
    int8_t graphicsIndex = 0;
    /// Index into the message being filled in, as the collected graphics are written out.
    int8_t innerGraphicsIndex = 0;
    /// How many graphics the message about to be sent contains, which selects the message size.
    int8_t numToSend = 0;
    /// The layer being cleared, while a clear is in progress.
    int8_t layerToClear = 0;
    /// The graphic pulled from the tree that has not yet been placed in a batch.
    GraphicsObject* nextGraphicsObject = nullptr;
    /// Layer delete message. Kept as a member only so its size can be passed to
    /// `getWaitTimeAfterGraphicSendMs`.
    RefSerialData::Tx::DeleteGraphicLayerMessage
        messageDel;  // only for RefSerialData::Tx::getWaitTimeAfterGraphicSendMs
    /// Outbound message holding one graphic.
    RefSerialData::Tx::Graphic1Message message1;
    /// Outbound message holding two graphics.
    RefSerialData::Tx::Graphic2Message message2;
    /// Outbound message holding five graphics.
    RefSerialData::Tx::Graphic5Message message5;
    /// Outbound message holding seven graphics.
    RefSerialData::Tx::Graphic7Message message7;
    /// Outbound message holding one string graphic, which cannot be batched with the others.
    RefSerialData::Tx::GraphicCharacterMessage messageCharacter;

    // when get UIDrawCommand, it should set this
    /// The root of the graphics tree this subsystem draws. Nothing is drawn until a draw command
    /// supplies one via `setTopLevelContainer`.
    GraphicsContainer* topLevelContainer = nullptr;

    /// How many layers the server supports, numbered 0 through 9.
    static constexpr int NUM_LAYERS = 10;  // layers 0-9
    /// Per-layer state: 0 clear, 1 holding graphics, 2 marked to be cleared.
    int layersState[NUM_LAYERS];  // 0 is clear, 1 is not clear, 2 is needs cleared

public:  // Public Methods
    /**
     * @param[in] driver The global drivers object.
     */
    UISubsystem(tap::Drivers* driver);
    ~UISubsystem() {}  // Intentionally blank

    /**
     * Call this function once, outside of the main loop.
     * It registers the subsystem.
     */
    void initialize();

    /// Advances the drawing protothread by one step. Called once per control loop iteration by the
    /// scheduler.
    void refresh() override;

    /// @return A graphic name not yet handed out. Every graphic needs a distinct name for the
    /// server to tell them apart.
    static uint32_t getUnusedGraphicName();

    /** Returns -1 if there aren't any unused layers
     * Only call this if you plan on calling deleteAndHideLayer */
    static int8_t getUnusedLayer();

    /** Marks a layer to be cleared next time the UISubsystem gets a chance.
     * Will also mark items on this layer as hidden to make sure they don't immediately get
     * drawn again. */
    static void clearAndHideLayer(int8_t layer);

    /**
     * Puts name into array (changing it in place), and returns array
     */
    static uint8_t* formatGraphicName(uint8_t array[3], uint32_t name);

    /**
     * Sets the graphics tree this subsystem draws.
     *
     * @param[in] container The root container. Not owned; it must outlive this subsystem.
     */
    void setTopLevelContainer(GraphicsContainer* container);

    /**
     * Wraps an angle into the 0-359 range the server expects.
     *
     * @param[in,out] a The angle in degrees, modified in place.
     */
    static void fixAngle(uint16_t* a)
    {
        *a %= 360;  // set a to the remainder after dividing by 360, so if it was 361 it would
                    // now be 1
    }

private:  // Private Methods
    /**
     * The protothread body: clears any layers that need it, walks the graphics tree collecting
     * changed graphics, sends them in batches, and waits out the server's required delay between
     * messages.
     *
     * @return `true` while the protothread is still running.
     */
    bool run();  // for protothread
};
}  // namespace src::control::client_display::graphics