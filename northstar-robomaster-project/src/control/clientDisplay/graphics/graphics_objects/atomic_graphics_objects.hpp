#pragma once

#include "atomic_graphics_object.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A straight line between two points.
 *
 * The endpoint, thickness, and color fields are public and can be assigned directly; the change is
 * picked up the next time the HUD is drawn.
 */
class Line : public AtomicGraphicsObject
{
public:
    /**
     * @param[in] color The line's color.
     * @param[in] x1 The first endpoint's x coordinate, in pixels from the left edge.
     * @param[in] y1 The first endpoint's y coordinate, in pixels from the bottom edge.
     * @param[in] x2 The second endpoint's x coordinate.
     * @param[in] y2 The second endpoint's y coordinate.
     * @param[in] thickness The line's thickness, in pixels.
     */
    Line(
        RefSerialData::Tx::GraphicColor color,
        uint16_t x1,
        uint16_t y1,
        uint16_t x2,
        uint16_t y2,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          x1(x1),
          y1(y1),
          x2(x2),
          y2(y2),
          thickness(thickness)
    {
    }

    /// Constructs a degenerate white line at the origin, to be filled in later.
    Line() : Line(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configLine(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevX1 == x1 && prevY1 == y1 && prevX2 == x2 &&
            prevY2 == y2 && prevColor == color);
    }

    /// The line's endpoints and thickness, in pixels. Assign directly; the change appears the next
    /// time this graphic is drawn.
    uint16_t x1, y1, x2, y2, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevX1 = x1;
        prevY1 = y1;
        prevX2 = x2;
        prevY2 = y2;
        prevColor = color;
    }

    uint16_t prevThickness, prevX1, prevY1, prevX2, prevY2 = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * A rectangle outline, positioned by its lower left corner.
 *
 * Note that the server takes two opposite corners, so `width` and `height` are added to `x` and `y`
 * when the message is built.
 */
class UnfilledRectangle : public AtomicGraphicsObject
{
public:
    /**
     * @param[in] color The outline's color.
     * @param[in] x The lower left corner's x coordinate, in pixels from the left edge.
     * @param[in] y The lower left corner's y coordinate, in pixels from the bottom edge.
     * @param[in] width The rectangle's width, in pixels.
     * @param[in] height The rectangle's height, in pixels.
     * @param[in] thickness The outline's thickness, in pixels.
     */
    UnfilledRectangle(
        RefSerialData::Tx::GraphicColor color,
        uint16_t x,
        uint16_t y,
        uint16_t width,
        uint16_t height,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          x(x),
          y(y),
          width(width),
          height(height),
          thickness(thickness)
    {
    }

    /// Constructs a degenerate white rectangle at the origin, to be filled in later.
    UnfilledRectangle() : UnfilledRectangle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1)
    {
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configRectangle(thickness, x, y, width + x, height + y, graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevX == x && prevY == y && prevWidth == width &&
            prevHeight == height && prevColor == color);
    }

    uint16_t x, y, width, height,
        thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

    uint16_t prevThickness, prevX, prevY, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * A circle outline, positioned by its center.
 */
class UnfilledCircle : public AtomicGraphicsObject
{
public:
    /**
     * @param[in] color The outline's color.
     * @param[in] cx The center's x coordinate, in pixels from the left edge.
     * @param[in] cy The center's y coordinate, in pixels from the bottom edge.
     * @param[in] r The radius, in pixels.
     * @param[in] thickness The outline's thickness, in pixels.
     */
    UnfilledCircle(
        RefSerialData::Tx::GraphicColor color,
        uint16_t cx,
        uint16_t cy,
        uint16_t r,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          cx(cx),
          cy(cy),
          r(r),
          thickness(thickness)
    {
    }

    /// Constructs a degenerate white circle at the origin, to be filled in later.
    UnfilledCircle() : UnfilledCircle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configCircle(thickness, cx, cy, r, graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevCx == cx && prevCy == cy && prevR == r &&
            prevColor == color);
    }

    /// The circle's center, radius, and outline thickness, in pixels. Assign directly; the change
    /// appears the next time this graphic is drawn.
    uint16_t cx, cy, r, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevR = r;
        prevColor = color;
    }

    uint16_t prevThickness, prevCx, prevCy, prevR = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * An ellipse outline, positioned by its center and sized by its semi-axes.
 */
class UnfilledEllipse : public AtomicGraphicsObject
{
public:
    UnfilledEllipse(
        RefSerialData::Tx::GraphicColor color,
        uint16_t cx,
        uint16_t cy,
        uint16_t width,
        uint16_t height,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          cx(cx),
          cy(cy),
          width(width),
          height(height),
          thickness(thickness)
    {
    }

    UnfilledEllipse() : UnfilledEllipse(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configEllipse(thickness, cx, cy, width, height, graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width &&
            prevHeight == height && prevColor == color);
    }

    uint16_t cx, cy, width, height,
        thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

    uint16_t prevThickness, prevCx, prevCy, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * An elliptical arc: the portion of an ellipse outline between two angles.
 *
 * Angles are in degrees measured clockwise from straight up, which is the server's convention, not
 * the counterclockwise-from-x-axis convention used for physical angles elsewhere in the codebase.
 */
class Arc : public AtomicGraphicsObject
{
public:
    /**
     * @param[in] color The arc's color.
     * @param[in] cx The center's x coordinate, in pixels from the left edge.
     * @param[in] cy The center's y coordinate, in pixels from the bottom edge.
     * @param[in] width The horizontal semi-axis, in pixels.
     * @param[in] height The vertical semi-axis, in pixels.
     * @param[in] startAngle Where the arc begins, in degrees clockwise from up.
     * @param[in] endAngle Where the arc ends, in degrees clockwise from up.
     * @param[in] thickness The arc's thickness, in pixels.
     */
    Arc(RefSerialData::Tx::GraphicColor color,
        uint16_t cx,
        uint16_t cy,
        uint16_t width,
        uint16_t height,
        uint16_t startAngle,
        uint16_t endAngle,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          startAngle(startAngle),
          endAngle(endAngle),
          cx(cx),
          cy(cy),
          width(width),
          height(height),
          thickness(thickness)
    {
    }

    /// Constructs a degenerate white arc at the origin, to be filled in later.
    Arc() : Arc(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        RefSerialTransmitter::configArc(
            startAngle,
            endAngle,
            thickness,
            cx,
            cy,
            width,
            height,
            graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevCx == cx && prevCy == cy && prevWidth == width &&
            prevHeight == height && prevColor == color && prevStartAngle == startAngle &&
            prevEndAngle == endAngle);
    }

    /// Where the arc begins and ends, in degrees clockwise from straight up. Assign directly; the
    /// change appears the next time this graphic is drawn.
    uint16_t startAngle, endAngle;  // can set this directly, will appear next time drawn, 0 is
                                    // up, positive is clockwise, in degrees
    uint16_t cx, cy, width, height,
        thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevWidth = width;
        prevHeight = height;
        prevStartAngle = startAngle;
        prevEndAngle = endAngle;
        prevColor = color;
    }

    uint16_t prevThickness, prevCx, prevCy, prevWidth, prevStartAngle, prevEndAngle, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * One arc of the concentric ring drawn around the center of the screen, used for gauges such as the
 * hit ring and lane assist.
 *
 * The arc is positioned to line up with the parenthesis-shaped brackets the client draws by
 * default, on either the left or right side. Higher lane numbers sit further inside. Rather than
 * setting angles directly, callers give a fraction of the bracket's span, so a gauge can be filled
 * without knowing the underlying angles.
 */
class LargeCenteredArc : public Arc
{
public:
    /**
     * @param[in] isLeft `true` for the left bracket, `false` for the right.
     * @param[in] lane Which concentric ring to draw on; 0 sits just inside the bracket and higher
     *      numbers sit further in.
     */
    LargeCenteredArc(bool isLeft, uint16_t lane) : Arc(), isLeft(isLeft), lane(lane)
    {
        cx = UISubsystem::HALF_SCREEN_WIDTH;
        cy = UISubsystem::HALF_SCREEN_HEIGHT;
        setLower(0);
        setHigher(1);
        thickness = THICKNESS;
        width = SIZE0 - lane * THICKNESS;
        height = SIZE0 - lane * THICKNESS;
    }

    /**
     * Moves the end of the arc nearer the bottom of the bracket.
     *
     * @param[in] r Position along the bracket, from 0 at the bottom to 1 at the top.
     */
    void setLower(float r)
    {
        if (isLeft)
        {
            startAngle = static_cast<uint16_t>(std::lerp(START_ANGLE_LEFT, END_ANGLE_LEFT, r));
        }
        else
        {
            endAngle = static_cast<uint16_t>(std::lerp(END_ANGLE_RIGHT, START_ANGLE_RIGHT, r));
        }

        fixZeroThickness();
    }

    /**
     * Moves the end of the arc nearer the top of the bracket.
     *
     * @param[in] r Position along the bracket, from 0 at the bottom to 1 at the top.
     */
    void setHigher(float r)
    {
        if (isLeft)
        {
            endAngle = static_cast<uint16_t>(std::lerp(START_ANGLE_LEFT, END_ANGLE_LEFT, r));
        }
        else
        {
            startAngle = static_cast<uint16_t>(std::lerp(END_ANGLE_RIGHT, START_ANGLE_RIGHT, r));
        }

        fixZeroThickness();
    }

    /**
     * @param[in] newIsLeft `true` to draw on the left bracket, `false` for the right.
     */
    void setIsLeft(bool newIsLeft) { isLeft = newIsLeft; }

private:
    /// Arc thickness, in pixels. Also the spacing between consecutive lanes.
    static constexpr uint16_t THICKNESS = 5;  // pixels
    /// Semi-axis of lane 0, in pixels, sized to sit just inside the bracket the client draws.
    static constexpr uint16_t SIZE0 =
        392;  // pixels, makes it so we are just inside the left parenthesis thingy if in lane
              // 0, higher number lanes are further in

    /// Angle of the bottom of the left bracket, in degrees clockwise from up.
    static constexpr uint16_t START_ANGLE_LEFT =
        227;  // degrees, lines up with the bottom of the left parenthesis thingy that is drawn
              // by default
    /// Angle of the top of the left bracket, in degrees clockwise from up.
    static constexpr uint16_t END_ANGLE_LEFT = 313;  // degrees, lines up with the top

    /// Angle of the bottom of the right bracket, mirrored from the left.
    static constexpr uint16_t START_ANGLE_RIGHT =
        START_ANGLE_LEFT - 180;  // degrees, lines up with the bottom of the left parenthesis
                                 // thingy that is drawn by default
    /// Angle of the top of the right bracket, mirrored from the left.
    static constexpr uint16_t END_ANGLE_RIGHT =
        END_ANGLE_LEFT - 180;  // degrees, lines up with the top

    /// Whether this arc is drawn on the left bracket.
    bool isLeft;
    /// Which concentric ring this arc sits on.
    uint16_t lane;

    // in the event the start and end angle are the same, hide the arc, not have it be a full
    // circle
    /// Hides the arc when its two ends coincide. The server would otherwise draw a full ellipse,
    /// which is the opposite of the intended empty gauge.
    void fixZeroThickness() { setHidden(startAngle == endAngle); }
};

/**
 * @ingroup client_display
 *
 * Works out where a piece of text has to be placed for the server to render it inside a given box.
 *
 * The server anchors text by its baseline rather than a corner and gives no way to query how wide a
 * string will come out, so the width is estimated from the character count and font size using a
 * ratio measured against the real display. Mixed into the text graphics so they can be positioned
 * with the same corner-and-size convention as the shapes.
 */
class TextSizer
{
public:
    /// The text box's lower left corner and height, in pixels. Assign directly; the change appears
    /// the next time this graphic is drawn.
    uint16_t height, x, y = 0;  // can set this directly, will appear next time drawn
    /// The estimated width of the rendered text, in pixels. Read only in practice: it is
    /// recomputed from the text and height on every draw.
    uint16_t width = 0;         // can read this, setting will be in vain, reset next time drawn

    /**
     * @param[in] len The number of characters the text will render as.
     */
    TextSizer(uint16_t len) : len(len) {}
    /**
     * @param[in] len The number of characters the text will render as.
     * @param[in] x The text box's lower left x coordinate, in pixels.
     * @param[in] y The text box's lower left y coordinate, in pixels.
     * @param[in] height The text height, in pixels, which is also the font size.
     */
    TextSizer(uint16_t len, uint16_t x, uint16_t y, uint16_t height)
        : height(height),
          x(x),
          y(y),
          len(len)
    {
    }

    /** resizes this text to fit within the given rect, ignoring width because it doesn't cut off
     * the contained text */
    void inputRect(UnfilledRectangle* rect)
    {
        x = rect->x;
        y = rect->y;
        height = rect->height;
        calculateNumbers();  // caller might want to have an updated width
    }

    /** resizes the given rect to bound this text, will include width */
    void outputRect(UnfilledRectangle* rect)
    {
        calculateNumbers();
        rect->x = x;
        rect->y = y;
        rect->width = width;
        rect->height = height;
    }

    /** call this if you set text and height and want an up to date width */
    void calculateNumbers()
    {
        fontSize = height;
        textX = x;
        textY = y + height;
        calculateWidth();
    }

private:
    // need to test/tune, was from ui website
    /// Numerator of the character-width-to-font-size ratio used to estimate rendered width.
    static constexpr uint16_t WIDTH_OFFSET_MULT = 19;
    /// Denominator of the character-width-to-font-size ratio used to estimate rendered width.
    static constexpr uint16_t WIDTH_OFFSET_DIV = 47;

    // need polymorphism later to get len because width depends on len
    /// Re-estimates the rendered width from the current character count and font size.
    void calculateWidth()
    {
        width = fontSize * len - fontSize * WIDTH_OFFSET_MULT / WIDTH_OFFSET_DIV;
    }

protected:
    /// The font size and baseline position the server is actually given. Derived from `x`, `y`,
    /// and `height`; set them and call `calculateNumbers` rather than writing these.
    uint16_t fontSize, textX,
        textY = 0;     // can read these, but don't set these, set with setTextNumbers
    /// How many characters the text renders as; 3 for `123` or `ABC`. Drives the width estimate.
    uint16_t len = 0;  // for sending integer 123 or text ABC, len would be 3. Not sure about
                       // floats yet, need to test

    /**
     * @param[in] n The integer to measure.
     * @return How many characters it renders as, counting the minus sign for negatives.
     */
    uint16_t intLen(int32_t n)
    {
        if (n == 0) return 1;
        if (n < 0) return 1 + intLen(-n);
        return std::floor(std::log10(n) + 1);
    }

    // assumes 1.2 will be shown 1.200; 4 as 4.000; and 1.11111 as 1.111, need to test
    /**
     * @param[in] n The float to measure.
     * @return How many characters it renders as, assuming the server's three decimal places plus
     *      the point, and counting the minus sign for negatives.
     */
    uint16_t floatLen(float n)
    {
        if (n >= 0 && n < 1) return 5;
        if (n < 0) return 1 + floatLen(-n);
        return std::floor(std::log10(n) + 5);
    }

    // assumes null terminated, also strings longer than 30 will say size 30 because only 30 can
    // be send in one message
    /**
     * @param[in] str The null-terminated string to measure.
     * @return Its length, capped at 30, which is all one character message can carry.
     */
    uint16_t stringLen(const char* str)
    {
        uint16_t r = strlen(str);
        return r < 30 ? r : 30;
    }

    /**
     * Sets the character count and re-estimates the width.
     *
     * @param[in] newLen The number of characters the text renders as.
     */
    void setLen(uint16_t newLen)
    {
        len = newLen;
        calculateWidth();
    }
};

/**
 * @ingroup client_display
 *
 * An integer rendered as text on the HUD.
 *
 * The server formats the number itself, so only the value has to be sent, and the graphic is only
 * redrawn when the value or its appearance actually changes.
 */
class IntegerGraphic : public AtomicGraphicsObject, public TextSizer
{
public:
    /**
     * Constructs the graphic sized and positioned to fit inside an existing rectangle, which is
     * how a number is placed inside a box already on the HUD.
     *
     * @param[in] newInteger The value to display.
     * @param[in] rect The rectangle to fit within. Its color and thickness are adopted too.
     */
    IntegerGraphic(int32_t newInteger, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(intLen(newInteger)),
          thickness(rect->thickness),
          integer(newInteger)
    {
        inputRect(rect);
    }

    /// Constructs a white zero at the origin, to be filled in later.
    IntegerGraphic() : IntegerGraphic(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1){};

    /**
     * @param[in] color The text color.
     * @param[in] newInteger The value to display.
     * @param[in] x The text box's lower left x coordinate, in pixels.
     * @param[in] y The text box's lower left y coordinate, in pixels.
     * @param[in] height The text height, in pixels.
     * @param[in] thickness The stroke thickness, in pixels.
     */
    IntegerGraphic(
        RefSerialData::Tx::GraphicColor color,
        int32_t newInteger,
        uint16_t x,
        uint16_t y,
        uint16_t height,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(intLen(newInteger), x, y, height),
          thickness(thickness)
    {
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        setLen(intLen(integer));
        calculateNumbers();
        RefSerialTransmitter::configInteger(
            fontSize,
            thickness,
            textX,
            textY,
            integer,
            graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevX == x && prevY == y && prevHeight == height &&
            prevColor == color && prevInteger == integer);
    }

    /// The stroke thickness, in pixels.
    uint16_t thickness = 0;
    /// The value being displayed. Assign directly; the change appears the next time this graphic
    /// is drawn.
    int32_t integer = 0;

    /// @return `true` if the value has changed since it was last drawn. Lets a caller resize the
    /// surrounding box only when the number of digits could have changed.
    bool integerChanged() { return prevInteger != integer; }

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevInteger = integer;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    int32_t prevInteger = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * A float rendered as text on the HUD, with three decimal places.
 */
class FloatGraphic : public AtomicGraphicsObject, public TextSizer
{
public:
    /**
     * Constructs the graphic sized and positioned to fit inside an existing rectangle.
     *
     * @param[in] newFloat The value to display.
     * @param[in] rect The rectangle to fit within. Its color and thickness are adopted too.
     */
    FloatGraphic(float newFloat, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(floatLen(newFloat)),
          thickness(rect->thickness),
          _float(newFloat)
    {
        inputRect(rect);
    }

    /**
     * @param[in] color The text color.
     * @param[in] newFloat The value to display.
     * @param[in] x The text box's lower left x coordinate, in pixels.
     * @param[in] y The text box's lower left y coordinate, in pixels.
     * @param[in] height The text height, in pixels.
     * @param[in] thickness The stroke thickness, in pixels.
     */
    FloatGraphic(
        RefSerialData::Tx::GraphicColor color,
        float newFloat,
        uint16_t x,
        uint16_t y,
        uint16_t height,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(floatLen(newFloat), x, y, height),
          thickness(thickness)
    {
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final
    {
        setLen(floatLen(_float));
        calculateNumbers();
        // the 3 is decimal precision, need to see what changing it does
        RefSerialTransmitter::configFloatingNumber(
            fontSize,
            3,
            thickness,
            textX,
            textY,
            _float,
            graphicData);
        setPrev();
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevX == x && prevY == y && prevHeight == height &&
            prevColor == color && prevFloat == _float);
    }

    uint16_t thickness = 0;
    /// The value being displayed. Assign directly; the change appears the next time this graphic
    /// is drawn.
    float _float = 0;

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevFloat = _float;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    float prevFloat = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

/**
 * @ingroup client_display
 *
 * A string rendered as text on the HUD.
 *
 * Strings go to the server in their own message type and cannot be batched with the other
 * graphics, which is what `isStringGraphic` tells the `UISubsystem`. The text is copied into a
 * fixed buffer, since the server accepts at most 30 characters in one message.
 */
class StringGraphic : public AtomicGraphicsObject, public TextSizer
{
private:
    /// Size of the text buffer: 30 characters plus a null terminator.
    static constexpr int STRING_SIZE = 31;  // not sure if it should be 30 or 31

public:
    /**
     * Constructs the graphic sized and positioned to fit inside an existing rectangle.
     *
     * @param[in] newString The text to display, null terminated.
     * @param[in] rect The rectangle to fit within. Its color and thickness are adopted too.
     */
    StringGraphic(const char* newString, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(stringLen(newString)),
          thickness(rect->thickness)
    {
        inputRect(rect);
        setString(newString);
    }

    /**
     * @param[in] color The text color.
     * @param[in] newString The text to display, null terminated.
     * @param[in] x The text box's lower left x coordinate, in pixels.
     * @param[in] y The text box's lower left y coordinate, in pixels.
     * @param[in] height The text height, in pixels.
     * @param[in] thickness The stroke thickness, in pixels.
     */
    StringGraphic(
        RefSerialData::Tx::GraphicColor color,
        const char* newString,
        uint16_t x,
        uint16_t y,
        uint16_t height,
        uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(stringLen(newString), x, y, height),
          thickness(thickness)
    {
        setString(newString);
    }

    /**
     * Replaces the displayed text and re-estimates its width.
     *
     * @param[in] newString The text to display, null terminated. Truncated to 30 characters.
     */
    void setString(const char* newString)
    {
        strncpy(string, newString, STRING_SIZE);
        calculateNumbers();
    }

    void configCharacterData(RefSerialData::Tx::GraphicCharacterMessage* characterData) final
    {
        setLen(stringLen(string));
        calculateNumbers();
        configGraphicData(&characterData->graphicData);
        RefSerialTransmitter::configCharacterMsg(
            fontSize,
            thickness,
            textX,
            textY,
            string,
            characterData);
        setPrev();
    }

    // StringGraphics fill the data differently. configGraphicGenerics still needs called, but
    // finishConfigGraphicData shouldn't do anything extra
    void finishConfigGraphicData(__attribute__((unused))
                                 RefSerialData::Tx::GraphicData* graphicData) final
    {
    }

    bool needsRedrawn() final
    {
        return !(
            prevThickness == thickness && prevX == x && prevY == y && prevHeight == height &&
            prevColor == color && !std::strncmp(string, oldString, STRING_SIZE));
    }

    uint16_t thickness = 0;
    /// The text being displayed. Prefer `setString`, which also updates the width estimate.
    char string[STRING_SIZE];

    /// @return Always `true`; the `UISubsystem` sends this graphic on its own rather than batching
    /// it with the shapes.
    bool isStringGraphic() final { return true; }

private:
    void setPrev()
    {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevColor = color;
        strncpy(oldString, string, STRING_SIZE);
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    char oldString[STRING_SIZE];
    RefSerialData::Tx::GraphicColor prevColor;
};

}  // namespace src::control::client_display::graphics