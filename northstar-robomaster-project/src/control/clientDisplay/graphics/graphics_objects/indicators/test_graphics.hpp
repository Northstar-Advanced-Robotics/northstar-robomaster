#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Draws one of each graphic type alongside the bounding boxes computed for them.
 *
 * Used to check `TextSizer`'s width estimates against what the server actually renders, by drawing
 * the predicted box and the real text together and seeing whether they line up.
 */
class TestGraphics : public GraphicsContainer
{
public:
    TestGraphics()
    {
        bounds1Before.color = UISubsystem::Color::PINK;
        bounds2Before.color = UISubsystem::Color::PINK;
        bounds3Before.color = UISubsystem::Color::PINK;

        int1.outputRect(&bounds1After);
        // string2.outputRect(&bounds2After);
        // float3.outputRect(&bounds3After);

        addGraphicsObject(&arc1);

        addGraphicsObject(&line1);
        addGraphicsObject(&line2);
        addGraphicsObject(&rect1);
        addGraphicsObject(&rect3);
        addGraphicsObject(&bounds2Before);
        // addGraphicsObject(&string2);       // should appear before any of the ones before
        // (assuming TestGraphics is first in its container)
        addGraphicsObject(&bounds2After);  // seventh object

        addGraphicsObject(&bounds1Before);
        addGraphicsObject(&int1);  // should appear with the other ones
        addGraphicsObject(&bounds1After);

        addGraphicsObject(&arc2);  // this one updates

        addGraphicsObject(&bounds3Before);
        // addGraphicsObject(&float3);  // should appear with the other ones
        addGraphicsObject(&bounds3After);  // seventh object

        addGraphicsObject(&circ1);
        addGraphicsObject(&circ2);
        addGraphicsObject(&ell1);

        addGraphicsObject(&rect2);
    }

    void update()
    {
        if (angle == 360) angle = 0;
        angle++;
        arc2.startAngle = angle;
        arc2.endAngle = angle + 30;
    }

private:
    Line line1{UISubsystem::Color::CYAN, 20, 700, 70, 750, 5};
    Line line2{UISubsystem::Color::CYAN, 80, 700, 80, 750, 5};
    UnfilledRectangle rect1{UISubsystem::Color::CYAN, 100, 700, 50, 50, 1};
    UnfilledRectangle rect2{UISubsystem::Color::CYAN, 170, 700, 50, 50, 10};
    UnfilledRectangle rect3{UISubsystem::Color::CYAN, 100, 770, 50, 50, 10};

    UnfilledRectangle bounds1Before{UISubsystem::Color::ORANGE, 100, 550, 100, 100, 1};
    IntegerGraphic int1{4259, &bounds1Before};
    UnfilledRectangle bounds1After{UISubsystem::Color::GREEN, 100, 550, 100, 100, 1};

    UnfilledRectangle bounds2Before{UISubsystem::Color::ORANGE, 500, 550, 100, 100, 1};
    // StringGraphic string2{"Test", &bounds2Before};
    UnfilledRectangle bounds2After{UISubsystem::Color::PURPLISH_RED, 500, 550, 100, 100, 1};

    UnfilledRectangle bounds3Before{UISubsystem::Color::ORANGE, 1100, 550, 100, 100, 1};
    // FloatGraphic float3{8.12345f, &bounds3Before};
    UnfilledRectangle bounds3After{UISubsystem::Color::PURPLISH_RED, 1100, 550, 100, 100, 1};

    UnfilledCircle circ1{UISubsystem::Color::CYAN, 1800, 300, 25, 1};
    UnfilledCircle circ2{UISubsystem::Color::CYAN, 1800, 600, 25, 5};
    Arc arc1{UISubsystem::Color::CYAN, 100, 700, 50, 50, 0, 30, 1};
    UnfilledEllipse ell1{UISubsystem::Color::CYAN, 1600, 500, 100, 50, 1};

    uint16_t angle = 0;
    Arc arc2{UISubsystem::Color::YELLOW, 170, 770, 50, 50, 0, 30, 1};
};

}  // namespace src::control::client_display::graphics