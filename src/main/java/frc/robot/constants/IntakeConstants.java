/**Todo: Add constants for the Intake Subsystem */
package frc.robot.constants;

import frc.robot.utils.PIDGains;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
/**
 * Constants for the Intake Subsystem
 */
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
public class IntakeConstants {

    public class Intake {
        public class Motor1 {
            //IntakeMotor1
            public static final int kMotorID = ID.TalonFX.intake;
            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
        }
        public class Motor2 {
            //IntakeMotor2
            public static final int kMotorID = ID.TalonFX.intake_dos;
            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive; //this should be opposite motor1
        }
        //General
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
        //Commanded speeds
        public static final double kForwardRPM = 6000.0; // Changed from 4000 to 6000 because of gearbox change-Simon
        public static final double kSlowForwardRPM = 700.0;
        public static final double kReverseRPM = 3500.0;
    }
    public class Deploy {
        //Cancoder
        public static final boolean kUseCANcoder = true;
        public static final int kCANcoderID = ID.CANcoder.deploy_cc;
        public static final double kMagnetOffset = -0.031592  ; //Add 0.1 to offset; Adjust magnet to sensor offset for CANcoder
        //Motor
        public static final int kMotorID = ID.TalonFX.deploy;
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        /**
         * Deploy gear ratio from motor rotor rotations to CANcoder rotations.
         * Rotor-to-mechanism is 50:14 and sensor is 5:1 reduction from mechanism.
         */
        /** Rotor rotations per mechanism rotation (18:15 pulleys, 12:1 gearbox) 
         * 15t pulley on gearbox, 18t on shaft
         * 7:1 stage in gearbox
        */
        public static final double kRotorToMechanismRatio = (18.0 / 18.0) * 7.0 / 1;
        /** Sensor rotations per mechanism rotation (7:1 reduction from mechanism). */
        public static final double kSensorToMechanismRatio = 1.0  / 7.0;
        public static final double kRotorToSensorRatio = kRotorToMechanismRatio / kSensorToMechanismRatio;
        //FF gains
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kS = 0.9;
        //PID gains
        public static final double kP = 14.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        //Soft-hold PID gains (slot1)
        public static final double kSoftHoldS = 0.9;
        public static final double kSoftHoldV = 0.0;
        public static final double kSoftHoldA = 0.0;
        public static final double kSoftHoldP = 4.0;
        public static final double kSoftHoldI = 0.0;
        public static final double kSoftHoldD = 0.0;
        /** Maximum absolute position error to stay in soft-hold mode while deployed out. */
        public static final double kSoftHoldEngageToleranceRotations = 0.20;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 30.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        //Software limits (mechanism rotations)
        /** Disable while characterizing manual deploy travel; re-enable for normal operation. */
        public static final boolean kSoftLimitEnable = true;
        public static final double kSoftLimitLow = 0.7;   // CANcoder 0.10 * 7.0
        public static final double kSoftLimitHigh = 4.395; // CANcoder 0.625 * 7.0
        //Motion Magic profile (mechanism rotations)
        public static final double kMotionMagicCruise = 3.5; //2.5
        public static final double kMotionMagicAccel = 13.0; //9.0
        public static final double kMotionMagicJerk = 200.0;
        /** Manual deploy duty-cycle cap for operator stick control (0.0 to 1.0). */
        public static final double kManualDutyCycleMax = 1.0;
        /** During tuning, initialize deploy setpoint to unjam on subsystem init. */
        public static final boolean kInitToUnjamForTuning = false;

        public class Positions {
            public static final double kStow = 0.7;   // CANcoder 0.10 * 7.0
            public static final double kDeployedSafe = 0.7; //Safe to go under truss - Changed from 3.6 for imidiate go
            public static final double kOut = 4.378;  // CANcoder 0.625 * 7.0
            public static final double kUnjam = 2.8;  // CANcoder 0.40 * 7.0
            public static final double kProtect = kStow;
        }
    }
    //Controller Setup
    public static final CANBus canBus = new CANBus("rio");
    public static final GlobalConstants.TelemetryLevel kTelemetryLevel = GlobalConstants.TelemetryLevel.INFO;
    public static final boolean isDisabled = false; //disable climber default command

    public static final int kMotorID = ID.TalonFX.intake;
    public static final boolean kIsInverted = true;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final double kStowPosition = 0;
    public static final double kDeployPower = 0.8;
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

