#pragma once

#include <cmath>

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A 2D vector of floats, used throughout the client display for screen positions and offsets.
 *
 * The referee system's HUD works in screen pixels, so this is deliberately a small
 * value-semantics type with the arithmetic the drawing code needs, rather than the heavier
 * `modm::Vector` used for physical quantities elsewhere.
 */
class Vector2d
{
protected:
    /// The vector's components.
    float x, y;
    /**
     * @param[in] num The value to constrain.
     * @param[in] min The lower bound.
     * @param[in] max The upper bound.
     * @return `num` constrained to the given range.
     */
    float valClamp(float num, float min, float max) { return std::min(std::max(num, min), max); }

public:
    // Constructors
    /**
     * @param[in] x The x component.
     * @param[in] y The y component.
     */
    Vector2d(float x, float y) : x(x), y(y) {}
    ~Vector2d() {}
    /// Constructs the zero vector.
    Vector2d() : x(0.0f), y(0.0f) {}
    /**
     * @param[in] vec The x and y components, in that order.
     */
    Vector2d(float vec[2]) : x(vec[0]), y(vec[1]) {}
    /**
     * @param[in] other The vector to copy.
     */
    Vector2d(const Vector2d& other) : x(other.x), y(other.y) {}

    // Getters
    /// @return The x component.
    float getX() const { return x; }
    /// @return The y component.
    float getY() const { return y; }

    /// Compute angle from positive x-axis
    float angle() const
    {
        if (x == 0 && y == 0) return 0;
        return std::atan2(y, x);
    }

    /// Rotate vector by given angle
    Vector2d rotate(float amt) const
    {
        float mag = magnitude();
        return Vector2d(mag * std::cos(amt + angle()), mag * std::sin(amt + angle()));
    }

    /// Compute magnitude (length) of vector
    float magnitude() const { return std::hypot(x, y); }

    /**
     * @param[in] min Componentwise lower bounds.
     * @param[in] max Componentwise upper bounds.
     * @return This vector with each component constrained to the given range.
     */
    Vector2d clamp(Vector2d min, Vector2d max)
    {
        return Vector2d(valClamp(x, min.x, max.x), valClamp(y, min.y, max.y));
    }

    /**
     * Writes the components into a caller-supplied array.
     *
     * @param[out] array Receives the x and y components, in that order.
     * @return `array`, so the call can be used inline.
     */
    float* toArray(float array[2])
    {
        array[0] = x;
        array[1] = y;
        return array;
    }

    /// Overload + operator (vector addition)
    Vector2d operator+(const Vector2d& other) const { return Vector2d(x + other.x, y + other.y); }

    /// Overload - operator (vector subtraction)
    Vector2d operator-(const Vector2d& other) const { return Vector2d(x - other.x, y - other.y); }

    /// Overload * operator (scalar multiplication)
    Vector2d operator*(float scalar) const { return Vector2d(x * scalar, y * scalar); }

    /// Overload += operator (vector addition and assignment)
    Vector2d& operator+=(const Vector2d& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    /// Overload -= operator (vector subtraction and assignment)
    Vector2d& operator-=(const Vector2d& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    /// Overload *= operator (scalar multiplication and assignment)
    Vector2d& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    /// Overload == operator (vector equality)
    bool operator==(const Vector2d& other) const
    {
        constexpr float EPSILON = 1e-4;  // Small threshold for floating-point comparison
        return (std::fabs(x - other.x) < EPSILON) && (std::fabs(y - other.y) < EPSILON);
    }

    /// Overload assignment operator =
    Vector2d& operator=(const Vector2d& other)
    {
        if (this != &other)
        {  // Prevent self-assignment
            x = other.x;
            y = other.y;
        }
        return *this;
    }
};

}  // namespace src::control::client_display::graphics