/**Todo: Add constants for the Indexer Subsystem */
package frc.robot.constants;

import frc.robot.utils.PIDGains;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
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

    public class Indexer {
        public static final int kMotorID = ID.TalonFX.indexer;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        //FF gains
        public static final double kV = 0.11862;
        public static final double kA = 0.0066756;
        public static final double kS = 0.17534;
        //PID gains
        public static final double kP = 0.000058859;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;        
    }
    public class Knuckle {
        public static final int kMotorID = ID.TalonFX.knuckle;
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kReverseRPM = 1500.0;
        //FF gains
        public static final double kV = 0.11756;
        public static final double kA = 0.0025794;
        public static final double kS = 0.23565;
        //PID gains
        public static final double kP = 0.00019926;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }
    public class LiveBottom {
        public static final int kMotorID = ID.TalonFX.livebottom;
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        //Power Levels
        public static final double kForwardPower = 0.25;
        public static final double kReversePower = 0.25;
        //FF gains
        public static final double kV = 0.11756;
        public static final double kA = 0.0025794;
        public static final double kS = 0.23565;
        //PID gains
        public static final double kP = 0.00019926;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
    }

    public class MatrixBreaker {
        public static final int kServoID = ID.PWM.matrixbreaker;
        public static final double kForward = 0.0;
        public static final double kReverse = 1.0;
        public static final double kStop = 0.5;
    }

    public class Shoe {
        public static final int kServoID = ID.PWM.shoe;
        public static final double kForward = 0.0;
        public static final double kReverse = 1.0;
        public static final double kStop = 0.5;
    }
}
