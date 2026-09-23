#pragma once

#include "control/client_display/ui_subsystem.hpp"

#include "graphics_object.hpp"

namespace src::control::client_display
{
class AtomicGraphicsObject : public GraphicsObject
{
public:
    AtomicGraphicsObject(tap::communication::serial::RefSerialData::Tx::GraphicColor color)
        : color(color)
    {
        UISubsystem::formatGraphicName(graphicNameArray, UISubsystem::getUnusedGraphicName());
    }

    int countNeedRedrawn() final { return needsRedrawn(); }

    /*
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

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

    int size() final
    {
        return 1;  // container of one object
    }

    virtual void finishConfigGraphicData(
        tap::communication::serial::RefSerialData::Tx::GraphicData* graphicData) = 0;

    void configGraphicData(
        tap::communication::serial::RefSerialData::Tx::GraphicData* graphicData) final
    {
        tap::communication::serial::RefSerialTransmitter::configGraphicGenerics(
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

    void resetIteration() final { countIndex = 0; }

    void layerHasBeenCleared(int8_t clearedLayer) final
    {
        if (clearedLayer == layer)
            wasHidden = true;  // was deleted, doesn't set if I want to be hidden or not
    }

    void allLayersCleared() final
    {
        wasHidden = true;  // was deleted, doesn't set if I want to be hidden or not
    }

    tap::communication::serial::RefSerialData::Tx::GraphicColor
        color;  // can set this directly, will appear next time drawn

    void hide() final { isHidden = true; }

    void show() final { isHidden = false; }

    void resetDrawMarks() final { markedToDraw = false; }

    void markToDraw() final { markedToDraw = true; }

private:
    tap::communication::serial::RefSerialData::Tx::GraphicOperation getNextOperation()
    {
        if (isHidden)
        {
            return tap::communication::serial::RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE;
        }
        else
        {
            if (wasHidden)
                return tap::communication::serial::RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD;
            else
                return tap::communication::serial::RefSerialData::Tx::GraphicOperation::
                    GRAPHIC_MODIFY;
        }
    }

protected:
    bool isHidden = false;
    bool wasHidden = true;
    bool markedToDraw = false;

    uint8_t graphicNameArray[3];
    int8_t layer = 0;

private:
    int8_t prevLayer = -2;
};

}  // namespace src::control::client_display