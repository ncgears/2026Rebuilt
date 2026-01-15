/**Todo: Add constants for the Indexer Subsystem */
package frc.robot.constants;

import frc.robot.utils.PIDGains;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Indexer Subsystem
 */
public class IndexerConstants {

    //Controller Setup
    public static final CANBus canBus = new CANBus("rio");
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final boolean isDisabled = false; //disable climber default command

    public static final int kBeambreakID = ID.DIO.indexer_beambreak;

    public class Feed {
        //Motor Setup
        public static final int kMotorID = ID.TalonFX.indexer_feed;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kFwdPower = 0.8;
        public static final double kRevPower = 0.8;
        //PID Control
        public static final double kS = 0.22; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 32.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.01; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 200; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 0; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }
    public class Belt {
        //Motor Setup
        public static final int kMotorID = ID.TalonFX.indexer_belt;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kFwdPower = 0.8;
        public static final double kRevPower = 0.8;
        //PID Control
        public static final double kS = 0.22; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 32.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.01; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 200; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 0; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }
}
