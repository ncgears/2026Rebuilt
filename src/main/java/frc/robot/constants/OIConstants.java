
package frc.robot.constants;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
/**
 * Constants for the Operator Interface
 */
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
public class OIConstants { 
    public static final int JoyDriverID = 0; //ID of Driver Joystick
    public static final int JoyOperID = 1; //ID of Operator Joystick
    public static final int JoyProgID = 2; //ID of Programmer Joystick
    public static final double kMinDeadband = 0.03; //Deadband for analog joystick axis minimum
    public static final double kMaxDeadband = 0.98; //Deadband for analog joystick axis minimum
}


