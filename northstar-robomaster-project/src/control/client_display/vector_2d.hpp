#pragma once

#include <cmath>

namespace src::control::client_display
{
class Vector2d
{
protected:
    float x, y;
    float valClamp(float num, float min, float max) { return std::min(std::max(num, min), max); }

public:
    // Constructors
    Vector2d(float x, float y) : x(x), y(y) {}
    ~Vector2d() {}
    Vector2d() : x(0.0f), y(0.0f) {}
    Vector2d(float vec[2]) : x(vec[0]), y(vec[1]) {}
    Vector2d(const Vector2d& other) : x(other.x), y(other.y) {}

    // Getters
    float getX() const { return x; }
    float getY() const { return y; }

    // Compute angle from positive x-axis
    float angle() const
    {
        if (x == 0 && y == 0) return 0;
        return std::atan2(y, x);
    }

    // Rotate vector by given angle
    Vector2d rotate(float amt) const
    {
        float mag = magnitude();
        return Vector2d(mag * std::cos(amt + angle()), mag * std::sin(amt + angle()));
    }

    // Compute magnitude (length) of vector
    float magnitude() const { return std::hypot(x, y); }

    Vector2d clamp(Vector2d min, Vector2d max)
    {
        return Vector2d(valClamp(x, min.x, max.x), valClamp(y, min.y, max.y));
    }

    float* toArray(float array[2])
    {
        array[0] = x;
        array[1] = y;
        return array;
    }

    // Overload + operator (vector addition)
    Vector2d operator+(const Vector2d& other) const { return Vector2d(x + other.x, y + other.y); }

    // Overload - operator (vector subtraction)
    Vector2d operator-(const Vector2d& other) const { return Vector2d(x - other.x, y - other.y); }

    // Overload * operator (scalar multiplication)
    Vector2d operator*(float scalar) const { return Vector2d(x * scalar, y * scalar); }

    // Overload += operator (vector addition and assignment)
    Vector2d& operator+=(const Vector2d& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    // Overload -= operator (vector subtraction and assignment)
    Vector2d& operator-=(const Vector2d& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    // Overload *= operator (scalar multiplication and assignment)
    Vector2d& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    // Overload == operator (vector equality)
    bool operator==(const Vector2d& other) const
    {
        constexpr float EPSILON = 1e-4;  // Small threshold for floating-point comparison
        return (std::fabs(x - other.x) < EPSILON) && (std::fabs(y - other.y) < EPSILON);
    }

    // Overload assignment operator =
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

}  // namespace src::control::client_display