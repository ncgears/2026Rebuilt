package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Constants for the Targeting helper class.
 */
public class TargetingConstants {
    public static final LinearVelocity kMaxSpeedMetersPerSecond = MetersPerSecond.of(2.5); //throttle speed when targeting
    public static final GlobalConstants.TelemetryLevel kTelemetryLevel = GlobalConstants.TelemetryLevel.INFO;
    /** Heading error tolerance for READY state transition during target tracking. */
    public static final double kReadyToleranceDegrees = 1.5;
    /** Minimum absolute turn rate command while targeting in degrees per second. */
    public static final double kMinTurnRateDegreesPerSecond = 37.24;
    /** Minimum heading error required before enforcing min turn rate in degrees. */
    public static final double kMinTurnRateErrorDegrees = 1.6;
}
