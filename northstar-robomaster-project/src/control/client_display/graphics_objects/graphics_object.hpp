#pragma once

#include "tap/communication/serial/ref_serial_transmitter.hpp"

namespace src::control::client_display
{
/* Both containers and non containers are a GraphicsObject. The UISubsystem deals with most
 * things at a GraphicsObject level. */
class GraphicsObject
{
public:
    /*
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

    /*
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

    virtual void resetIteration() = 0;

    /*
     * For facilitating flattening of containers of containers. Simple
     * objects have a size of 1, and containers call size() on each
     * of their objects.
     *
     * Might not be needed.
     */
    virtual int size() = 0;

    /*
     * Containers do nothing, AtomicGraphicsObject's fill the graphic data
     */
    virtual void configGraphicData(tap::communication::serial::RefSerialData::Tx::GraphicData*) {}
    virtual void configCharacterData(
        tap::communication::serial::RefSerialData::Tx::GraphicCharacterMessage*)
    {
    }

    /*
     * For when a layer gets cleared. This should make it so next
     * time this object or all contained objects are told to draw,
     * they use GRAPHIC_ADD and not GRAPHIC_MODIFY if they were on
     * the layer cleared.
     */
    virtual void layerHasBeenCleared(int8_t) = 0;
    virtual void allLayersCleared() = 0;

    /*
     * Graphics representing strings need to be sent as a CharacterMessage,
     * and can't be sent in a group of 7 like other graphics can.
     */
    virtual bool isStringGraphic() { return false; }

    virtual void hide() = 0;

    virtual void show() = 0;

    void setHidden(bool hidden) { hidden ? hide() : show(); }

    virtual void resetDrawMarks() = 0;
    virtual void markToDraw(){};  // only applies to objects, marking a container to draw
                                  // doesn't make sense

protected:
    u_int16_t countIndex = 0;
};

}  // namespace src::control::client_display