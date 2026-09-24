#pragma once
#include "orientation_2d.hpp"
#include "vector_2d.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A planar position and heading, combining `Vector2d` and `Orientation2d`.
 *
 * Inheriting from both means a pose can be passed anywhere either is expected; `vec` and
 * `orientation` name the two halves explicitly where that reads better. The vector operators keep
 * the heading unchanged, since translating something should not turn it.
 */
class Pose2d : public Vector2d, public Orientation2d
{
public:
    // all the constructors
    /**
     * @param[in] x The x component of the position.
     * @param[in] y The y component of the position.
     * @param[in] r The heading, in radians.
     */
    Pose2d(float x, float y, float r) : Vector2d(x, y), Orientation2d(r) {}

    ~Pose2d() {}

    /// Constructs a pose at the origin with zero heading.
    Pose2d() {}

    /**
     * @param[in] vec The x position, y position, and heading, in that order.
     */
    Pose2d(float vec[3]) : Vector2d(vec), Orientation2d(vec[2]) {}

    /**
     * @param[in] vec The position. The heading is set to zero.
     */
    Pose2d(const Vector2d& vec) : Vector2d(vec), Orientation2d(0.0f) {}

    /**
     * @param[in] other The pose to copy.
     */
    Pose2d(const Pose2d& other) : Vector2d(other.x, other.y), Orientation2d(other.rotation) {}

    // get this as a vec and as an orientation which is useful
    /// @return This pose's position, as a mutable reference to its `Vector2d` base.
    Vector2d& vec() { return *this; }

    /// @return This pose's heading, as a mutable reference to its `Orientation2d` base.
    Orientation2d& orientation() { return *this; }

    /// rotate but for a pose
    Pose2d rotate(float amt)
    {
        return Pose2d(
            magnitude() * std::cos(amt + angle()),
            magnitude() * std::sin(amt + angle()),
            rotation);
    }

    /**
     * @param[in] min Componentwise lower bounds on position and heading.
     * @param[in] max Componentwise upper bounds on position and heading.
     * @return This pose with each component constrained to the given range.
     */
    Pose2d clamp(Pose2d min, Pose2d max)
    {
        return Pose2d(
            valClamp(x, min.x, max.x),
            valClamp(y, min.y, max.y),
            valClamp(rotation, min.rotation, max.rotation));
    }

    /**
     * Writes the pose into a caller-supplied array.
     *
     * @param[out] array Receives the x position, y position, and heading, in that order.
     * @return `array`, so the call can be used inline.
     */
    float* toArray(float array[3])
    {
        array[0] = x;
        array[1] = y;
        array[2] = rotation;
        return array;
    }

    /// Overload + operator (Pose2d addition)
    Pose2d operator+(const Vector2d& other) const
    {
        return Pose2d(x + other.getX(), y + other.getY(), rotation);
    }

    /// Overload - operator (Pose2d subtraction)
    Pose2d operator-(const Vector2d& other) const
    {
        return Pose2d(x - other.getX(), y - other.getY(), rotation);
    }

    /// Overload * operator (scalar multiplication)
    Pose2d operator*(float scalar) const
    {
        return Pose2d(x * scalar, y * scalar, rotation * scalar);
    }

    /// Overload * operator (dot product)
    Pose2d operator*(const Pose2d& other) const
    {
        return Pose2d(x * other.x, y * other.y, rotation * other.rotation);
    }

    /// Overload += operator (Pose2d addition and assignment)
    Pose2d& operator+=(const Vector2d& other)
    {
        x += other.getX();
        y += other.getY();
        return *this;
    }

    /// Overload -= operator (Pose2d subtraction and assignment)
    Pose2d& operator-=(const Vector2d& other)
    {
        x -= other.getX();
        y -= other.getY();
        return *this;
    }

    /// Overload *= operator (scalar multiplication and assignment)
    Pose2d& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        rotation *= scalar;
        return *this;
    }

    /// Overload == operator (Pose2d equality check)
    bool operator==(const Pose2d& other) const
    {
        constexpr float EPSILON = 1e-4;  // Threshold for floating-point comparison
        return (std::fabs(x - other.x) < EPSILON) && (std::fabs(y - other.y) < EPSILON) &&
               (std::fabs(rotation - other.rotation) < EPSILON);
    }

    /// Overload = operator (Pose2d assignment)
    Pose2d& operator=(const Pose2d& other)
    {
        if (this != &other)
        {  // Prevent self-assignment
            x = other.x;
            y = other.y;
            rotation = other.rotation;
        }
        return *this;
    }

    /// Overload = operator (Pose2d assignment)
    Pose2d& operator=(const Vector2d& other)
    {
        if (this != &other)
        {  // Prevent self-assignment
            x = other.getX();
            y = other.getY();
        }
        return *this;
    }
};

}  // namespace src::control::client_display::graphics