
package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
// @SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Shooter Subsystem
 */
public class ShooterConstants {
    //Controller Setup
    public static final CANBus canBus = new CANBus("rio");
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final boolean isDisabled = false; //disable shooter system

    public static final double kDefaultRPM = 2500.0;
    public static final double kDefaultFrontRPM = kDefaultRPM;
    public static final double kDefaultBackRPM = kDefaultRPM;

    public static final double kIdleRPM = 0.0; //RPM when idle, to prevent ramp up spike

    public class Front {
        public static final int kMotorID = ID.TalonFX.shooter_front;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kGearRatio = 1.0;
        //FF gains
        public static final double kV = 0.12165; //0.14; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0026923; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kS = 0.25543; // add kS to overcome static friction: adjust first to start moving
        //PID gains
        public static final double kP = 0.000046567; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.0; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }
    public class Back {
        public static final int kMotorID = ID.TalonFX.shooter_back;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kGearRatio = 1.0;
        //FF gains
        public static final double kV = 0.12256; //0.14; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0026607; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kS = 0.23637; // add kS to overcome static friction: adjust first to start moving
        //PID gains
        public static final double kP = 0.000037237; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.0; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }
}
