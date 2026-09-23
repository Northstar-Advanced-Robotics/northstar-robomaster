#include "ui_subsystem.hpp"

namespace src::control::client_display
{
// define the static variable, if this isnt here things go wrong saying it is a undefined
// reference
uint32_t UISubsystem::currGraphicName = 0;
int8_t UISubsystem::currLayer = 1;  // layer 0 is where most stuff is, so asking for a layer gets
                                    // you 1 or higher (or -1 if there aren't any more layers)

UISubsystem::UISubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      refSerialTransmitter(drivers)
{
    // assume all layers had something on them from a previous team, so all need cleared
    for (int i = 0; i < NUM_LAYERS; i++) layersState[i] = 2;

    graphicsIndex = 0;  // might start with one or two already in the array from last time, so
                        // set it once outside of run
}

uint32_t UISubsystem::getUnusedGraphicName()
{
    if (currGraphicName > 0xffffff)
    {
        // maybe signal some error? turn on the red led?
        // But we would need to draw 16 million things to get here.
        // You would need to draw each pixel on the screen individally, 8 times
        // so probably never going to happen
        return currGraphicName;
    }
    else
    {
        return currGraphicName++;
    }
}

int8_t UISubsystem::getUnusedLayer()
{
    if (currLayer > 9)
    {
        return -1;
    }
    else
    {
        return currLayer++;
    }
}

uint8_t* UISubsystem::formatGraphicName(uint8_t array[3], uint32_t name)
{
    // shifts and bitwise and
    array[0] = static_cast<uint8_t>((name >> 16) & 0xff);
    array[1] = static_cast<uint8_t>((name >> 8) & 0xff);
    array[2] = static_cast<uint8_t>(name & 0xff);
    return array;
}

void UISubsystem::initialize() { drivers->commandScheduler.registerSubsystem(this); }

// guaranteed to be called, whether we have a command (topLevelContainer) or not
void UISubsystem::refresh() { run(); }

bool UISubsystem::run()
{  // run has to do with prototheads
    if (!this->isRunning())
    {
        restart();  // Restart the thread
    }

    PT_BEGIN();  // ignore this error, it still builds, need to figure out how to make vscode
                 // not angry at this
    // inside of a protothread, you aren't able to make new variables, errors with: 'jump to
    // case label' so make new variables in the hpp and set their values here

    // drivers->leds.set(tap::gpio::Leds::Red, false);
    // drivers->leds.set(tap::gpio::Leds::Green, false);

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

    needToClearAllLayers = true;
    layerToClear = -1;
    // clear any layers with state 2
    for (innerGraphicsIndex = 0; innerGraphicsIndex < NUM_LAYERS; innerGraphicsIndex++)
    {
        if (layersState[innerGraphicsIndex] == 2)
        {
            layerToClear = innerGraphicsIndex;
        }
        if (layersState[innerGraphicsIndex] == 1)
        {
            needToClearAllLayers = false;  // if all are 0(cleared) or 2(needs to be cleared),
                                           // then we can clear all, otherwise we can't
        }
    }  // loop ends when the layer at innerGraphicsIndex needs cleared, or all layers were
       // checked and all were clear

    if (needToClearAllLayers)
    {
        PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
        for (innerGraphicsIndex = 0; innerGraphicsIndex < NUM_LAYERS; innerGraphicsIndex++)
        {
            layersState[innerGraphicsIndex] = 0;
        }
        if (topLevelContainer)
        {
            topLevelContainer->allLayersCleared();
        }
    }
    else if (layerToClear != -1)
    {
        PT_CALL(refSerialTransmitter.deleteGraphicLayer(
            RefSerialTransmitter::Tx::DELETE_GRAPHIC_LAYER,
            layerToClear));
        layersState[layerToClear] = 0;
        if (topLevelContainer)
        {
            topLevelContainer->layerHasBeenCleared(layerToClear);
        }
        needToClearAllLayers = true;
    }

    // if any cleared, using the same variable
    if (needToClearAllLayers)
    {
        delayTimeout.restart(2 * RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageDel));
        PT_WAIT_UNTIL(delayTimeout.execute());

        // maybe some object saved from the last iteration just got removed, we don't want to
        // accidentally draw it could check if the 1 or 2 saved objects were on a cleared layer,
        // but it probably isn't worth it clearing layers probably doesn't happen enough, those
        // objects will be visited on the next iteration
        graphicsIndex = 0;
    }

    // what was in the while loop
    timesResetIteration = 0;
    topLevelContainer->resetDrawMarks();
    while (timesResetIteration < 2)
    {
        nextGraphicsObject = topLevelContainer->getNext();

        // if we run out of objects, try looping around to the start
        if (!nextGraphicsObject)
        {
            topLevelContainer->resetIteration();
            timesResetIteration++;
            continue;
        }

        if (nextGraphicsObject->isStringGraphic())
        {
            // if it is a string, keep the array as it is and send the string on its own
            nextGraphicsObject->configCharacterData(&messageCharacter);
            if (layersState[messageCharacter.graphicData.layer] == 0)
                layersState[messageCharacter.graphicData.layer] = 1;
            PT_CALL(refSerialTransmitter.sendGraphic(&messageCharacter));
            delayTimeout.restart(
                2 * RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageCharacter));
            PT_WAIT_UNTIL(delayTimeout.execute());
        }
        else
        {
            // if it isn't a string, add it to the array and see if it is full
            nextGraphicsObject->markToDraw();  // mark the object as 'to draw' to prevent it
                                               // from being gotten again from the container
            objectsToSend[graphicsIndex++] = nextGraphicsObject;
            if (graphicsIndex == TARGET_NUM_OBJECTS) break;  // if full, stop trying to find more
        }
    }

    numToSend = graphicsIndex;
    if (numToSend == 3 || numToSend == 4)
    {
        // can't send 3, will send 2 and save 1 for later
        // can't send 4, will send 2 and save 2 for later
        numToSend = 2;
    }
    if (numToSend == 6)
    {
        // can't send 6, will send 5 and save 1 for later
        numToSend = 5;
    }

    // drivers->leds.set(tap::gpio::Leds::Red, graphicsIndex == 1);
    // drivers->leds.set(tap::gpio::Leds::Green, graphicsIndex == 7);

    // so we have up to 7 objects to update. Would do a switch case but might confict with
    // protothread's switch case might be able to do something silly with (void) pointers
    if (numToSend == 1)
    {
        objectsToSend[0]->configGraphicData(&message1.graphicData);
        if (layersState[message1.graphicData.layer] == 0)
            layersState[message1.graphicData.layer] = 1;
        PT_CALL(refSerialTransmitter.sendGraphic(&message1));
        delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message1));
    }
    else if (numToSend == 2)
    {
        for (innerGraphicsIndex = 0; innerGraphicsIndex < numToSend; innerGraphicsIndex++)
        {
            objectsToSend[innerGraphicsIndex]->configGraphicData(
                &message2.graphicData[innerGraphicsIndex]);
            if (layersState[message2.graphicData[innerGraphicsIndex].layer] == 0)
                layersState[message2.graphicData[innerGraphicsIndex].layer] = 1;
        }
        PT_CALL(refSerialTransmitter.sendGraphic(&message2));
        delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
    }
    else if (numToSend == 5)
    {
        for (innerGraphicsIndex = 0; innerGraphicsIndex < numToSend; innerGraphicsIndex++)
        {
            objectsToSend[innerGraphicsIndex]->configGraphicData(
                &message5.graphicData[innerGraphicsIndex]);
            if (layersState[message5.graphicData[innerGraphicsIndex].layer] == 0)
                layersState[message5.graphicData[innerGraphicsIndex].layer] = 1;
        }
        PT_CALL(refSerialTransmitter.sendGraphic(&message5));
        delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
    }
    else if (numToSend == 7)
    {
        for (innerGraphicsIndex = 0; innerGraphicsIndex < numToSend; innerGraphicsIndex++)
        {
            objectsToSend[innerGraphicsIndex]->configGraphicData(
                &message7.graphicData[innerGraphicsIndex]);
            if (layersState[message7.graphicData[innerGraphicsIndex].layer] == 0)
                layersState[message7.graphicData[innerGraphicsIndex].layer] = 1;
        }
        PT_CALL(refSerialTransmitter.sendGraphic(&message7));
        delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message7));
    }

    if (graphicsIndex == 4)
    {                                         // save 2
        objectsToSend[0] = objectsToSend[2];  // copy the third item into the first slot
        objectsToSend[1] = objectsToSend[3];  // copy the fourth item into the second slot
        graphicsIndex = 2;                    // start with those two next time
    }
    else if (graphicsIndex == 3 || graphicsIndex == 6)
    {  // save 1
        objectsToSend[0] =
            objectsToSend[graphicsIndex - 1];  // move the one we are saving to the front
        graphicsIndex = 1;                     // start with that one next time
    }
    else
    {  // don't save any
        graphicsIndex = 0;
    }

    // if we sent something, wait for it so we don't lose packets
    if (numToSend > 0) PT_WAIT_UNTIL(delayTimeout.execute());

    // start the whole thing over
    PT_END();
    restart();
}

// This is required for the UISubsystem to have anything to draw.
// Use with to nullptr to remove the top level container.
void UISubsystem::setTopLevelContainer(GraphicsContainer* container)
{
    if (container)
    {
        topLevelContainer->resetIteration();
        // drivers->leds.set(tap::gpio::Leds::Blue, true);
    }
    topLevelContainer = container;

    // all layers need cleared
    for (innerGraphicsIndex = 0; innerGraphicsIndex < NUM_LAYERS; innerGraphicsIndex++)
    {
        layersState[innerGraphicsIndex] = 2;
    }
}
}  // namespace src::control::client_display