#pragma once

#include "tap/communication/serial/ref_serial_transmitter.hpp"

using namespace tap::communication::serial;

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Base of everything drawable on the client display.
 *
 * Both individual graphics and containers of graphics are `GraphicsObject`s, so `UISubsystem` can
 * walk an arbitrarily nested display without caring which is which.
 */
class GraphicsObject
{
public:
    /**
     * Simple objects return 0 or 1, for if they need redrawn.
     * Container objects return a number, for how many things
     * need redrawn. Container objects must also keep track of
     * an index, explained in setCountIndex
     */
    virtual int countNeedRedrawn() = 0;  // a virtual method allows polymorphism
    // a non virtual method that is overridden will use the definition of the method from the
    // declared type setting the virtual method to 0 means it is 'pure virtual', and the
    // existence of any pure virtual methods means the object can't be instantiated, like an
    // abstract class in Java.

    /**
     * Allows iteration of the tree-like structure of containers
     * containing containers.
     *
     * Why we need to iterate:
     * We want to send 7 graphics at a time, to maximize efficiency.
     * If a container has say 10 graphics that need redrawing, we
     * want to send the first 7 then know to skip those 7 next time.
     * We need to skip them so that in case every time all of those
     * first 7 want redrawn again, we don't get stuck on them without
     * ever redrawing the next 3.
     *
     * Why 7 graphics at a time is efficient:
     * We can send 1, 2, 5, or 7. Sending any number involves some overhead
     * (like message headers, see ref_serial_data.hpp)
     * There is a constant amount of overhead for sending any number of graphics,
     * so sending 7 means there is a seventh as much overhead per graphic than
     * sending 1 at a time.
     *
     * Non containers (AtomicGraphicsObject's) are treated as containers
     * of 1 object (so getNext(0) returns the GraphicsObject itself,
     * then nullptr until resetIteration() is called). You can use the
     * result of this for an if statement, nullptr is falsey and an actual
     * GrapicsObject* is truey.
     *
     * Example traversal: A has B and Q, and B has X, Y, and Z, and Q, X, Y,
     * and Z are not containers: A looks like [X, Y, Z, Q] when traversing.
     *
     * This will only return AtomicGraphicsObject's, but making the return
     * type that will lead to a problematic circle, with AtomicGraphicsObject
     * and GraphicsObject including eachother. Might be fixed with separate
     * cpp and hpp files.
     */
    virtual GraphicsObject* getNext() = 0;

    /// Rewinds this object's iteration state so a fresh traversal starts from the beginning.
    /// Containers rewind everything they hold.
    virtual void resetIteration() = 0;

    /**
     * For facilitating flattening of containers of containers. Simple
     * objects have a size of 1, and containers call size() on each
     * of their objects.
     *
     * Might not be needed.
     */
    virtual int size() = 0;

    /**
     * Containers do nothing, AtomicGraphicsObject's fill the graphic data
     */
    virtual void configGraphicData(RefSerialData::Tx::GraphicData*) {}
    virtual void configCharacterData(RefSerialData::Tx::GraphicCharacterMessage*) {}

    /**
     * For when a layer gets cleared. This should make it so next
     * time this object or all contained objects are told to draw,
     * they use GRAPHIC_ADD and not GRAPHIC_MODIFY if they were on
     * the layer cleared.
     */
    virtual void layerHasBeenCleared(int8_t) = 0;
    virtual void allLayersCleared() = 0;

    /**
     * Graphics representing strings need to be sent as a CharacterMessage,
     * and can't be sent in a group of 7 like other graphics can.
     */
    virtual bool isStringGraphic() { return false; }

    /// Marks this object to be deleted from the screen the next time it is drawn. Containers hide
    /// everything they hold.
    virtual void hide() = 0;

    /// Marks this object to be drawn again after being hidden. Containers show everything they
    /// hold.
    virtual void show() = 0;

    /**
     * @param[in] hidden `true` to hide this object, `false` to show it.
     */
    void setHidden(bool hidden) { hidden ? hide() : show(); }

    /// Clears the marks set by `markToDraw`, so the objects sent in the last batch become eligible
    /// for iteration again.
    virtual void resetDrawMarks() = 0;
    virtual void markToDraw(){};  // only applies to objects, marking a container to draw
                                  // doesn't make sense

protected:
    /// How far into this object the current traversal has reached. Simple objects use 0 and 1;
    /// containers use it as an index into their children.
    u_int16_t countIndex = 0;
};

}  // namespace src::control::client_display::graphics