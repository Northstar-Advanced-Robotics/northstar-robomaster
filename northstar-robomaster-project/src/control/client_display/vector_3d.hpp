#pragma once

#include <cmath>

#include "vector_2d.hpp"

namespace src::control::client_display
{
class Vector3d
{
protected:
    float x, y, z;
    float valClamp(float num, float min, float max) { return std::min(std::max(num, min), max); }

public:
    // Constructors
    Vector3d(float x, float y, float z) : x(x), y(y), z(z) {}
    ~Vector3d() {}
    Vector3d() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3d(float vec[3]) : x(vec[0]), y(vec[1]), z(vec[2]) {}
    Vector3d(const Vector3d& other) : x(other.x), y(other.y), z(other.z) {}

    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    // Compute angle around the z-axis from positive x-axis to the positive y-axis
    float angleYaw() const
    {
        if (x == 0 && y == 0) return 0;
        return std::atan2(y, x);
    }

    // Rotate vector by given angle around the z-axis, changing x and y
    Vector3d rotateYaw(float amt) const
    {
        float mag = std::hypot(x, y);
        return Vector3d(mag * std::cos(amt + angleYaw()), mag * std::sin(amt + angleYaw()), z);
    }

    // Compute angle around the x-axis from positive y-axis to the positive z-axis
    float anglePitch() const
    {
        if (y == 0 && z == 0) return 0;
        return std::atan2(z, y);
    }

    // Rotate vector by given angle around the x-axis, changing y and z
    Vector3d rotatePitch(float amt) const
    {
        float mag = std::hypot(y, z);
        return Vector3d(x, mag * std::cos(amt + anglePitch()), mag * std::sin(amt + anglePitch()));
    }

    // Compute magnitude (length) of vector
    float magnitude() const { return std::hypot(std::hypot(x, y), z); }

    Vector3d clamp(Vector3d min, Vector3d max)
    {
        return Vector3d(
            valClamp(x, min.x, max.x),
            valClamp(y, min.y, max.y),
            valClamp(z, min.z, max.z));
    }

    float* toArray(float array[3])
    {
        array[0] = x;
        array[1] = y;
        array[2] = z;
        return array;
    }

    // Overload + operator (vector addition)
    Vector3d operator+(const Vector3d& other) const
    {
        return Vector3d(x + other.x, y + other.y, z + other.z);
    }

    // Overload - operator (vector subtraction)
    Vector3d operator-(const Vector3d& other) const
    {
        return Vector3d(x - other.x, y - other.y, z - other.z);
    }

    // Overload * operator (scalar multiplication)
    Vector3d operator*(float scalar) const { return Vector3d(x * scalar, y * scalar, z * scalar); }

    // Overload += operator (vector addition and assignment)
    Vector3d& operator+=(const Vector3d& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    // Overload -= operator (vector subtraction and assignment)
    Vector3d& operator-=(const Vector3d& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    // Overload *= operator (scalar multiplication and assignment)
    Vector3d& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    // Overload == operator (vector equality)
    bool operator==(const Vector3d& other) const
    {
        constexpr float EPSILON = 1e-4;  // Small threshold for floating-point comparison
        return (std::fabs(x - other.x) < EPSILON) && (std::fabs(y - other.y) < EPSILON) &&
               (std::fabs(z - other.z) < EPSILON);
    }

    // Overload assignment operator =
    Vector3d& operator=(const Vector3d& other)
    {
        if (this != &other)
        {  // Prevent self-assignment
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }
};

}  // namespace src::control::client_display