#pragma once

#include "atomic_graphics_object.hpp"

namespace src::control::client_display::graphics
{
class Line : public AtomicGraphicsObject
{
public:
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

class UnfilledRectangle : public AtomicGraphicsObject
{
public:
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

class UnfilledCircle : public AtomicGraphicsObject
{
public:
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

class Arc : public AtomicGraphicsObject
{
public:
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

class LargeCenteredArc : public Arc
{
public:
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

    void setIsLeft(bool newIsLeft) { isLeft = newIsLeft; }

private:
    static constexpr uint16_t THICKNESS = 5;  // pixels
    static constexpr uint16_t SIZE0 =
        392;  // pixels, makes it so we are just inside the left parenthesis thingy if in lane
              // 0, higher number lanes are further in

    static constexpr uint16_t START_ANGLE_LEFT =
        227;  // degrees, lines up with the bottom of the left parenthesis thingy that is drawn
              // by default
    static constexpr uint16_t END_ANGLE_LEFT = 313;  // degrees, lines up with the top

    static constexpr uint16_t START_ANGLE_RIGHT =
        START_ANGLE_LEFT - 180;  // degrees, lines up with the bottom of the left parenthesis
                                 // thingy that is drawn by default
    static constexpr uint16_t END_ANGLE_RIGHT =
        END_ANGLE_LEFT - 180;  // degrees, lines up with the top

    bool isLeft;
    uint16_t lane;

    // in the event the start and end angle are the same, hide the arc, not have it be a full
    // circle
    void fixZeroThickness() { setHidden(startAngle == endAngle); }
};

class TextSizer
{
public:
    uint16_t height, x, y = 0;  // can set this directly, will appear next time drawn
    uint16_t width = 0;         // can read this, setting will be in vain, reset next time drawn

    TextSizer(uint16_t len) : len(len) {}
    TextSizer(uint16_t len, uint16_t x, uint16_t y, uint16_t height)
        : height(height),
          x(x),
          y(y),
          len(len)
    {
    }

    /* resizes this text to fit within the given rect, ignoring width because it doesn't cut off
     * the contained text */
    void inputRect(UnfilledRectangle* rect)
    {
        x = rect->x;
        y = rect->y;
        height = rect->height;
        calculateNumbers();  // caller might want to have an updated width
    }

    /* resizes the given rect to bound this text, will include width */
    void outputRect(UnfilledRectangle* rect)
    {
        calculateNumbers();
        rect->x = x;
        rect->y = y;
        rect->width = width;
        rect->height = height;
    }

    /* call this if you set text and height and want an up to date width */
    void calculateNumbers()
    {
        fontSize = height;
        textX = x;
        textY = y + height;
        calculateWidth();
    }

private:
    // need to test/tune, was from ui website
    static constexpr uint16_t WIDTH_OFFSET_MULT = 19;
    static constexpr uint16_t WIDTH_OFFSET_DIV = 47;

    // need polymorphism later to get len because width depends on len
    void calculateWidth()
    {
        width = fontSize * len - fontSize * WIDTH_OFFSET_MULT / WIDTH_OFFSET_DIV;
    }

protected:
    uint16_t fontSize, textX,
        textY = 0;     // can read these, but don't set these, set with setTextNumbers
    uint16_t len = 0;  // for sending integer 123 or text ABC, len would be 3. Not sure about
                       // floats yet, need to test

    uint16_t intLen(int32_t n)
    {
        if (n == 0) return 1;
        if (n < 0) return 1 + intLen(-n);
        return std::floor(std::log10(n) + 1);
    }

    // assumes 1.2 will be shown 1.200; 4 as 4.000; and 1.11111 as 1.111, need to test
    uint16_t floatLen(float n)
    {
        if (n >= 0 && n < 1) return 5;
        if (n < 0) return 1 + floatLen(-n);
        return std::floor(std::log10(n) + 5);
    }

    // assumes null terminated, also strings longer than 30 will say size 30 because only 30 can
    // be send in one message
    uint16_t stringLen(const char* str)
    {
        uint16_t r = strlen(str);
        return r < 30 ? r : 30;
    }

    void setLen(uint16_t newLen)
    {
        len = newLen;
        calculateWidth();
    }
};

class IntegerGraphic : public AtomicGraphicsObject, public TextSizer
{
public:
    IntegerGraphic(int32_t newInteger, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(intLen(newInteger)),
          thickness(rect->thickness),
          integer(newInteger)
    {
        inputRect(rect);
    }

    IntegerGraphic() : IntegerGraphic(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1){};

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

    uint16_t thickness = 0;
    int32_t integer = 0;

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

class FloatGraphic : public AtomicGraphicsObject, public TextSizer
{
public:
    FloatGraphic(float newFloat, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(floatLen(newFloat)),
          thickness(rect->thickness),
          _float(newFloat)
    {
        inputRect(rect);
    }

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

class StringGraphic : public AtomicGraphicsObject, public TextSizer
{
private:
    static constexpr int STRING_SIZE = 31;  // not sure if it should be 30 or 31

public:
    StringGraphic(const char* newString, UnfilledRectangle* rect)
        : AtomicGraphicsObject(rect->color),
          TextSizer(stringLen(newString)),
          thickness(rect->thickness)
    {
        inputRect(rect);
        setString(newString);
    }

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

    void setString(const char* newString)
    {
        // Truncate to the buffer size and always null-terminate; a plain
        // strncpy(dst, src, STRING_SIZE) leaves the buffer unterminated when
        // the source is STRING_SIZE chars or longer.
        strncpy(string, newString, STRING_SIZE - 1);
        string[STRING_SIZE - 1] = '\0';
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
    char string[STRING_SIZE];

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