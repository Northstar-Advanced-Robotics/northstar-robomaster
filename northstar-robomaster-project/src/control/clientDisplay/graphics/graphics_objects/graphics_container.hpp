#pragma once

#include <vector>

#include "graphics_object.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A `GraphicsObject` that holds other graphics objects, including other containers.
 *
 * Grouping graphics lets a whole HUD element be hidden, shown, or cleared with one call, and lets
 * the `UISubsystem` walk an arbitrarily nested display as a flat sequence when assembling batches
 * to send. Every operation is simply forwarded to the children.
 *
 * Contained objects are held by raw pointer and not owned, so they must outlive the container.
 */
class GraphicsContainer : public GraphicsObject
{
public:
    /// @return How many graphics in this container, at any depth, need to be redrawn.
    int countNeedRedrawn() final
    {  // final here means no more overriding, if more overriding is wanted replace final with
       // override
        // sum all contained objects counts
        int r = 0;
        for (GraphicsObject* p : objects)
        {
            r += p->countNeedRedrawn();
        }
        return r;
    }

    /**
     * Hands out the next graphic in this container that needs redrawing, descending into nested
     * containers. The index is only advanced past a child once that child reports it has nothing
     * left, so a child with several graphics to send is revisited until it is exhausted.
     *
     * @return The next graphic needing redraw, or `nullptr` once this traversal has exhausted the
     *      container. Call `resetIteration` to start over.
     */
    GraphicsObject* getNext() final
    {
        GraphicsObject* r = nullptr;

        // note that it is possible to skip this loop entirely if countIndex==objects.size(),
        // allowing the container above this one to check the container after this one
        // no int i = something, so start with semicolon
        for (; countIndex < objects.size(); countIndex++)
        {
            if (r) break;  // if we have something, don't ask for a new thing
            r = objects.at(countIndex)->getNext();
            if (r) countIndex--;  // if the container returned something, it might return more
        }

        // we found something in the loop: return it
        // we didn't find something in the loop: return nullptr, unchanged through the loop
        return r;
    }

    // not just objects.size() because containers can contain other containers
    /// @return The total number of graphics in this container, counting through nested containers
    /// rather than just the immediate children.
    int size() final
    {
        int r = 0;
        for (GraphicsObject* p : objects)
        {
            r += p->size();
        }
        return r;
    }

    /// Rewinds this container and everything it holds, so the next traversal starts from the
    /// beginning.
    void resetIteration() final
    {
        countIndex = 0;
        for (GraphicsObject* p : objects)
        {
            p->resetIteration();
        }
    }

    /** When adding, make sure you don't lose the object from leaving scope */
    /**
     * Adds an object to this container.
     *
     * @param[in] obj The object to add. Not owned; it must outlive this container, so take care
     *      not to pass something that goes out of scope.
     */
    void addGraphicsObject(GraphicsObject* obj) { objects.push_back(obj); }

    /**
     * Tells everything in this container that a layer was cleared.
     *
     * @param[in] layer The layer that was cleared.
     */
    void layerHasBeenCleared(int8_t layer) final
    {
        for (GraphicsObject* p : objects)
        {
            p->layerHasBeenCleared(layer);
        }
    }

    /// Tells everything in this container that every layer was cleared.
    void allLayersCleared() final
    {
        for (GraphicsObject* p : objects)
        {
            p->allLayersCleared();
        }
    }

    /// Hides everything in this container.
    void hide() final
    {
        for (GraphicsObject* p : objects)
        {
            p->hide();
        }
    }

    /// Shows everything in this container.
    void show() final
    {
        for (GraphicsObject* p : objects)
        {
            p->show();
        }
    }

    /// Clears the draw marks on everything in this container, making them eligible for the next
    /// batch.
    void resetDrawMarks() final
    {
        for (GraphicsObject* p : objects)
        {
            p->resetDrawMarks();
        }
    }

private:
    /// The contained objects, in the order they are traversed. Not owned.
    std::vector<GraphicsObject*> objects;
};

}  // namespace src::control::client_display::graphics