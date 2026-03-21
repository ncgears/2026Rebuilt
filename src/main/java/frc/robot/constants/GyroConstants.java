
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
/**
 * Constants let us quickly define the characteristics of our robot without having to search through code
 */
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
public class GyroConstants {
    public static final int kPigeonID = ID.Pigeon2.gyro;
    public static final String kCANbus = "drivetrain"; 
    public static final GlobalConstants.TelemetryLevel kTelemetryLevel = GlobalConstants.TelemetryLevel.INFO;
    public static final boolean kGyroReversed = false; //true for NAVx
}

