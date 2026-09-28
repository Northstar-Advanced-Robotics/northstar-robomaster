#include <cmath>

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * A planar rotation in radians.
 *
 * Kept as its own type rather than a bare float so that `Pose2d` can inherit position and
 * orientation separately, and so the drawing code cannot silently mix an angle up with a distance.
 * The value is not wrapped, so accumulated rotations can exceed a full turn.
 */
class Orientation2d
{
protected:
    /// The rotation, in radians.
    float rotation;

public:
    // Constructors
    /**
     * @param[in] r The rotation, in radians.
     */
    Orientation2d(float r) : rotation(r) {}
    ~Orientation2d() {}
    /// Constructs a zero rotation.
    Orientation2d() : rotation(0.0f) {}

    // Getter
    /// @return The rotation, in radians.
    float getRotation() const { return rotation; }

    /// Overload + operator (adds rotation values)
    Orientation2d operator+(const Orientation2d& other) const
    {
        return Orientation2d(rotation + other.rotation);
    }

    /// Overload - operator (subtracts rotation values)
    Orientation2d operator-(const Orientation2d& other) const
    {
        return Orientation2d(rotation - other.rotation);
    }

    /// Overload * operator (scales the rotation)
    Orientation2d operator*(float scalar) const { return Orientation2d(rotation * scalar); }

    /// Overload += operator (adds and assigns rotation)
    Orientation2d& operator+=(const Orientation2d& other)
    {
        rotation += other.rotation;
        return *this;
    }

    /// Overload -= operator (subtracts and assigns rotation)
    Orientation2d& operator-=(const Orientation2d& other)
    {
        rotation -= other.rotation;
        return *this;
    }

    /// Overload *= operator (scales rotation and assigns)
    Orientation2d& operator*=(float scalar)
    {
        rotation *= scalar;
        return *this;
    }

    /// Overload == operator (checks if two objects have the same rotation)
    bool operator==(const Orientation2d& other) const
    {
        constexpr float EPSILON = 1e-4;  // Small threshold for floating-point comparison
        return std::fabs(rotation - other.rotation) < EPSILON;
    }

    /// Overload assignment operator =
    Orientation2d& operator=(const Orientation2d& other)
    {
        if (this != &other)
        {  // Prevent self-assignment
            rotation = other.rotation;
        }
        return *this;
    }
};

}  // namespace src::control::client_display::graphics