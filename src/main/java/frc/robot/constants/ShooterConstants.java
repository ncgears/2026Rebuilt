
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
    public static final GlobalConstants.TelemetryLevel kTelemetryLevel = GlobalConstants.TelemetryLevel.INFO;
    public static final boolean isDisabled = false; //disable shooter system
    public static final double kRobotToShooter = 0.25; //Meters from the center of the robot to the shooter

    /* Lab Tested Values 
     * 2300 = Hub Corner
     * 2400 = Hub Corner (Preferred)
     * 3200 = Against Tower
    */
    /* GVSU Practice Field Tested Values Hub Dimensions 47.125" Across Front x 47.25 Across 
     * 60.348" from silver bar to center - 2400RPM Hits Backwall
     * 121.0356" from silver bar to center of hub - 3100RPM
     * Against the tower or 150.473" From Silver Bar to Center Hub -  3400 - 3450 RPM
     */
    public static final double kDefaultRPM = 2400.0; //CHANGE THIS
    public static final double kDefaultFrontRPM = kDefaultRPM;
    public static final double kDefaultBackRPM = kDefaultRPM;
    public static final double kFixedShotBackRPM = 2400.0;
    public static final double kTrenchShotBackRPM = 3400.0;
    public static final double kNeutralZoneDumpBackRPM = 4500.0;
    public static final double kReverseRPM = 1500.0;

    public static final double kIdleRPM = 0.0; //RPM when idle, to prevent ramp up spike
    public static final double kSpinupDelaySeconds = 0.25;

    /** Distance-to-RPM model for the back shooter wheel. */
    public class DistanceModel {
        /**
         * Linear fit from measured points (distance shooter->hub):
         * 1.5367 m -> 2350 RPM, 3.0734 m -> 3100 RPM, 3.8227 m -> 3450 RPM.
         * Formula: backRpm = kBackRpmPerMeter * distanceMeters + kBackRpmIntercept.
         */
        public static final double kBackRpmPerMeter = 485.2098239; //482 - 4/16 was 475
        public static final double kBackRpmIntercept = 1500.206999; //1611
        public static final double kBackRpmMin = 2200.0;
        public static final double kBackRpmMax = 3700.0;
    }

    /** Multipliers derived from the back shooter RPM (master). */
    public class Multipliers {
        public static final double kFrontFromBack = 1.0; //DONT CHANGE
        public static final double kNeutralZoneDumpFrontFromBack = 0.25;
        public static final double kIndexerFromBack = 0.6666666667; //DONT CHANGE
        public static final double kKnuckleFromBack = 0.8888888889; //DONT CHANGE
    }

    public class Front {
        public static final int kMotorID = ID.TalonFX.shooter_front;
        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
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
        public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
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
