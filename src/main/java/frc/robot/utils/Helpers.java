package frc.robot.utils;

/** Utility helper methods shared across subsystems. */
public final class Helpers {
    /**
     * Creates a non-instantiable helpers utility class.
     */
    private Helpers() {}

    /**
     * Converts revolutions per minute (RPM) to revolutions per second (RPS).
     *
     * @param rpm Speed in revolutions per minute.
     * @return Speed in revolutions per second.
     */
    public static double RPMtoRPS(double rpm) {
        return rpm / 60.0;
    }
}
