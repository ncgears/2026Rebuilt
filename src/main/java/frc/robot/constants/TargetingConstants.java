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
}
