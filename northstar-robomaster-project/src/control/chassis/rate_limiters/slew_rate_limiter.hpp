namespace src::chassis::algorithms
{
/**
 * @ingroup chassis
 *
 * Caps how fast a commanded wheel velocity may change from one control loop iteration to the next.
 *
 * Feeding a large step change straight to the motors draws a current spike and can trip the
 * referee system's power limit, so the setpoint is ramped instead. Starting from rest is the
 * exception: a stationary motor needs a minimum command to break static friction, so the limiter
 * jumps straight to a small non-zero velocity rather than creeping up to it.
 */
class SlewRateLimiter
{
public:
    /**
     * @param[in] rateLimit The largest allowed change in velocity per second, in motor RPM per
     *      second. Divided by the 500Hz control loop rate to get the per-iteration limit.
     * @param[in] maxError The largest tolerated deviation from the requested velocity, in motor
     *      RPM, likewise converted to per-iteration units.
     */
    SlewRateLimiter(float rateLimit, float maxError)
        : rateLimit(rateLimit / 500),
          maxError(maxError / 500)
    {
    }
    /**
     * Steps the commanded velocity toward the requested one by at most the configured rate limit.
     *
     * @param[in] desiredVelocity The velocity being asked for, in motor RPM.
     * @param[in] currentVelocity The velocity currently commanded, in motor RPM.
     * @return The velocity to command this iteration.
     */
    float runLimiter(float desiredVelocity, float currentVelocity);

private:
    /// Largest allowed velocity change per control loop iteration.
    float rateLimit;  // Motor rotations per minute per second
    /// Largest tolerated deviation from the requested velocity, per control loop iteration.
    float maxError;  // Rotations per minute
};

}  // namespace src::chassis::algorithms
