#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"

#include "graphics_object.hpp"

using namespace tap::communication::serial;

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A single drawable graphic: the leaf of the `GraphicsObject` tree.
 *
 * Takes care of everything common to drawing one graphic on the referee system's HUD: claiming a
 * unique graphic name at construction, tracking which layer it was last drawn on, and choosing
 * between add, modify, and delete based on whether the server already has this graphic. Subclasses
 * only have to say whether their appearance has changed (`needsRedrawn`) and fill in the shape
 * itself (`finishConfigGraphicData`).
 *
 * Whether a graphic is added or modified matters: the server rejects a modify for a graphic it has
 * never seen, and a graphic that was deleted by a layer clear has to be added again rather than
 * modified.
 */
class AtomicGraphicsObject : public GraphicsObject
{
public:
    /**
     * Claims a unique graphic name for this object, which the server uses to tell graphics apart.
     *
     * @param[in] color The color to draw in.
     */
    AtomicGraphicsObject(RefSerialData::Tx::GraphicColor color) : color(color)
    {
        UISubsystem::formatGraphicName(graphicNameArray, UISubsystem::getUnusedGraphicName());
    }

    /// @return 1 if this graphic's appearance has changed since it was last drawn, 0 otherwise.
    int countNeedRedrawn() final { return needsRedrawn(); }

    /**
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

    /**
     * @return `this` if the graphic still needs to be sent this traversal, `nullptr` once it has
     *      been handed out. A graphic needs sending if its appearance changed, it was hidden or
     *      shown, or its layer changed.
     */
    GraphicsObject* getNext() final
    {
        if (countIndex == 0 && !markedToDraw &&
            (isHidden != wasHidden || layer != prevLayer || needsRedrawn()))
        {
            countIndex = 1;
            return this;
        }
        return nullptr;
    }

    /// @return Always 1; a single graphic behaves as a container of one.
    int size() final
    {
        return 1;  // container of one object
    }

    /**
     * Fills in the shape-specific fields of a graphic message, after the generic fields have been
     * set by `configGraphicData`.
     *
     * @param[out] graphicData The message to populate.
     */
    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) = 0;

    /**
     * Populates a graphic message with this object's name, operation, layer, and color, then hands
     * off to `finishConfigGraphicData` for the shape. Records the hidden state and layer as sent,
     * so the next call knows whether to add or modify.
     *
     * @param[out] graphicData The message to populate.
     */
    void configGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configGraphicGenerics(
            graphicData,
            graphicNameArray,
            getNextOperation(),
            layer < 0 ? 0 : layer,  // UISubsystem::getUnusedLayer might return -1 when there
                                    // aren't any unused layers,
            // if someone doesn't check if it did this protects trying to send -1 to the server
            color);
        wasHidden = isHidden;
        prevLayer = layer;
        finishConfigGraphicData(graphicData);
    }

    /// Rewinds this object's iteration state so it will be handed out again on the next traversal.
    void resetIteration() final { countIndex = 0; }

    /**
     * Notes that a layer was cleared. If it was this object's layer the server no longer has the
     * graphic, so the next draw must add it rather than modify it.
     *
     * @param[in] clearedLayer The layer that was cleared.
     */
    void layerHasBeenCleared(int8_t clearedLayer) final
    {
        if (clearedLayer == layer)
            wasHidden = true;  // was deleted, doesn't set if I want to be hidden or not
    }

    /// Notes that every layer was cleared, so the next draw must add this graphic rather than
    /// modify it.
    void allLayersCleared() final
    {
        wasHidden = true;  // was deleted, doesn't set if I want to be hidden or not
    }

    /// The color to draw in. Can be assigned directly; the change takes effect the next time this
    /// graphic is drawn.
    RefSerialData::Tx::GraphicColor color;  // can set this directly, will appear next time drawn

    /// Marks this graphic to be deleted from the screen the next time it is drawn.
    void hide() final { isHidden = true; }

    /// Marks this graphic to be drawn again after being hidden.
    void show() final { isHidden = false; }

    /// Clears this graphic's draw mark, making it eligible for the next batch.
    void resetDrawMarks() final { markedToDraw = false; }

    /// Marks this graphic as already included in the batch being assembled, so the traversal does
    /// not hand it out twice.
    void markToDraw() final { markedToDraw = true; }

private:
    /**
     * @return The operation to send: delete if the graphic is hidden, add if the server does not
     *      currently have it, and modify otherwise.
     */
    RefSerialData::Tx::GraphicOperation getNextOperation()
    {
        if (isHidden)
        {
            return RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE;
        }
        else
        {
            if (wasHidden)
                return RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD;
            else
                return RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY;
        }
    }

protected:
    /// Whether this graphic should currently be off the screen.
    bool isHidden = false;
    /// Whether the server is currently without this graphic, either because it was hidden or
    /// because its layer was cleared. Starts `true`, since nothing has been drawn yet.
    bool wasHidden = true;
    /// Whether this graphic is already in the batch being assembled.
    bool markedToDraw = false;

    /// The unique three-byte name the server identifies this graphic by.
    uint8_t graphicNameArray[3];
    /// The layer to draw on. Subclasses may change it; the change is picked up on the next draw.
    int8_t layer = 0;

private:
    /// The layer this graphic was last drawn on, used to detect a layer change. Initialized to a
    /// value no real layer can take so the first draw always counts as a change.
    int8_t prevLayer = -2;
};

}  // namespace src::control::client_display::graphics