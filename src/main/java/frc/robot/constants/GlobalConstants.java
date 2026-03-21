
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
/**
 * Constants that are Global for the robot
 */
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
public class GlobalConstants {
    /**
     * Telemetry verbosity levels used to gate dashboard publishing.
     */
    public enum TelemetryLevel {
        NONE,
        INFO,
        DEBUG
    }

    //Global Constants
    public static final int kFalconMaxRPS = 6350 / 60;
    public static final int kKrakenMaxRPS = 5800 / 60;
    public static final boolean CAMERA_ENABLED = false; //set to false if UsbCamera is removed
    public static final boolean SWERVE_SENSOR_NONCONTINUOUS = false;
    public static final int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
    public static final int kPidIndex = 0;  //Talon PID index for primary loop
    public static final int kPidProfileSlotIndex = 0; //PID Profile gains slot
    public static final int kWheelbaseWidth = 20; //from pivot to pivot of swerve module side to side
    public static final int kWheelbaseLength = 20; //from pivot to pivot of the swerve module front to back
    public static final int kFrameWidth = 29; //outside frame perimeter side to side
    public static final int kFrameLength = 25; //outside frame perimeter front to back
    public static final int kBumperWidth = kFrameWidth + 6; //outside of bumpers side to side //.89m
    public static final int kBumperLength = kFrameLength + 6; //outside of bumpers front to back //.965m
    public static final boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
    public static final int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recubring debug messages
    public static final int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
    public final static boolean tuningMode = true; //Enable tunable numbers

    /**
     * Returns true when configured telemetry level is at least the required level.
     *
     * @param configured Configured level.
     * @param required Required level.
     * @return True when telemetry at the required level should be published.
     */
    public static boolean telemetryAtLeast(TelemetryLevel configured, TelemetryLevel required) {
        return configured.ordinal() >= required.ordinal();
    }
}

