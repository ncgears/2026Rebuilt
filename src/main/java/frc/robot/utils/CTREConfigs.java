package frc.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.*; 

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    /**
     * Returns the shared configuration instance.
     *
     * @return Singleton CTREConfigs instance.
     */
    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    //TalonFX
        public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration coralFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration shooterTiltFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
    //TalonFXS
        public final TalonFXSConfiguration intakeFXSConfig = new TalonFXSConfiguration();
        public final TalonFXSConfiguration shooterFrontFXSConfig = new TalonFXSConfiguration();
        public final TalonFXSConfiguration shooterBackFXSConfig = new TalonFXSConfiguration();
    //CANcoder
        public final CANcoderConfiguration coralCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration shooterCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration elevatorCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration climberCCConfig = new CANcoderConfiguration();

    /** Creates and populates CTRE configuration objects for all subsystems. */
    public CTREConfigs() {
        //Intake Configuration
        //CANcoder
        coralCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        coralCCConfig.MagnetSensor.SensorDirection = (CoralConstants.kSensorInverted) ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        coralCCConfig.MagnetSensor.MagnetOffset = CoralConstants.kMagnetOffset;

        Slot0Configs coralSlot0Configs = new Slot0Configs()
            .withKP(CoralConstants.in.kP)
            .withKI(CoralConstants.in.kI)
            .withKD(CoralConstants.in.kD)
            .withKS(CoralConstants.in.kS)
            .withKV(CoralConstants.in.kV)
            .withKA(CoralConstants.in.kA);
        coralFXConfig.Slot0 = coralSlot0Configs;
        Slot1Configs coralSlot1Configs = new Slot1Configs()
            .withKP(CoralConstants.out.kP)
            .withKI(CoralConstants.out.kI)
            .withKD(CoralConstants.out.kD)
            .withKS(CoralConstants.out.kS)
            .withKV(CoralConstants.out.kV)
            .withKA(CoralConstants.out.kA);
        coralFXConfig.Slot1 = coralSlot1Configs;
        //Current Limits
        CurrentLimitsConfigs coralCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CoralConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(CoralConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(CoralConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(CoralConstants.kCurrentLimitEnable);
        coralFXConfig.CurrentLimits = coralCurrentLimitsConfigs;
        //Motion Magic
        // MotionMagicConfigs coralMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicCruiseVelocity(CoralConstants.kMotionMagicCruise)
        //     .withMotionMagicAcceleration(CoralConstants.kMotionMagicAccel)
        //     .withMotionMagicJerk(CoralConstants.kMotionMagicJerk);
        // coralFXConfig.MotionMagic = coralMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs coralSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(CoralConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(CoralConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(CoralConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(CoralConstants.kSoftForwardLimit);
        coralFXConfig.SoftwareLimitSwitch = coralSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs coralHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // coralFXConfig.HardwareLimitSwitch = coralHardwareLimitsConfigs;
        //Encoder
        if(CoralConstants.kUseCANcoder) {
            coralFXConfig.Feedback.FeedbackRemoteSensorID = CoralConstants.kCANcoderID;
            coralFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            coralFXConfig.Feedback.RotorToSensorRatio = CoralConstants.kGearRatio;
            coralFXConfig.Feedback.SensorToMechanismRatio = CoralConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            coralFXConfig.Feedback.SensorToMechanismRatio = CoralConstants.kGearRatio;
        }
        //Neutral and Direction
        coralFXConfig.MotorOutput.NeutralMode = CoralConstants.kNeutralMode;
        coralFXConfig.MotorOutput.Inverted = (CoralConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        coralFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Shooter Configuration
        //CANcoder
        shooterCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        shooterCCConfig.MagnetSensor.SensorDirection = (ShooterConstants.kSensorInverted) ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        shooterCCConfig.MagnetSensor.MagnetOffset = ShooterConstants.kMagnetOffset;

        Slot0Configs shooterSlot0Configs = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kV)
            .withKA(ShooterConstants.kA);
        shooterTiltFXConfig.Slot0 = shooterSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ShooterConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ShooterConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ShooterConstants.kCurrentLimitEnable);
        shooterTiltFXConfig.CurrentLimits = shooterCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ShooterConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ShooterConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ShooterConstants.kMotionMagicJerk);
        shooterTiltFXConfig.MotionMagic = shooterMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs shooterSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(ShooterConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(ShooterConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(ShooterConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(ShooterConstants.kSoftForwardLimit);
        shooterTiltFXConfig.SoftwareLimitSwitch = shooterSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs coralHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // shooterTiltFXConfig.HardwareLimitSwitch = shooterHardwareLimitsConfigs;
        //Encoder
        if(ShooterConstants.kUseCANcoder) {
            shooterTiltFXConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.kCANcoderID;
            shooterTiltFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            shooterTiltFXConfig.Feedback.RotorToSensorRatio = ShooterConstants.kGearRatio;
            shooterTiltFXConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            shooterTiltFXConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kGearRatio;
        }
        //Neutral and Direction
        shooterTiltFXConfig.MotorOutput.NeutralMode = ShooterConstants.wrist.kNeutralMode;
        shooterTiltFXConfig.MotorOutput.Inverted = (ShooterConstants.wrist.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        shooterTiltFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Toros
        Slot0Configs shooterFrontSlot0Configs = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kV)
            .withKA(ShooterConstants.kA);
        shooterFrontFXSConfig.Slot0 = shooterFrontSlot0Configs;
        shooterFrontFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        HardwareLimitSwitchConfigs shooterFrontHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
            // .withReverseLimitEnable(false)
            // .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            // .withReverseLimitAutosetPositionEnable(true)
            // .withReverseLimitAutosetPositionValue(0.0)
            .withForwardLimitEnable(true)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen);
        shooterFrontFXSConfig.HardwareLimitSwitch = shooterFrontHardwareLimitsConfigs;
        //Current Limits
        CurrentLimitsConfigs shooterFrontCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ShooterConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ShooterConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ShooterConstants.kCurrentLimitEnable);
        shooterFrontFXSConfig.CurrentLimits = shooterFrontCurrentLimitsConfigs;
        //Neutral and Direction
        shooterFrontFXSConfig.MotorOutput.NeutralMode = ShooterConstants.left.kNeutralMode;
        shooterFrontFXSConfig.MotorOutput.Inverted = (ShooterConstants.left.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Right Side
        shooterBackFXSConfig.Slot0 = shooterFrontSlot0Configs;
        shooterBackFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        shooterBackFXSConfig.HardwareLimitSwitch = shooterFrontHardwareLimitsConfigs;
        shooterBackFXSConfig.CurrentLimits = shooterFrontCurrentLimitsConfigs;
        shooterBackFXSConfig.MotorOutput.NeutralMode = ShooterConstants.right.kNeutralMode;
        shooterBackFXSConfig.MotorOutput.Inverted = (ShooterConstants.right.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


        //Elevator
        //CANcoder
        elevatorCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        elevatorCCConfig.MagnetSensor.SensorDirection = (ElevatorConstants.kSensorInverted) ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        elevatorCCConfig.MagnetSensor.MagnetOffset = ElevatorConstants.kMagnetOffset;

        Slot0Configs elevatorSlot0Configs = new Slot0Configs()
            .withKP(ElevatorConstants.kP)
            .withKI(ElevatorConstants.kI)
            .withKD(ElevatorConstants.kD)
            .withKS(ElevatorConstants.kS)
            .withKV(ElevatorConstants.kV)
            .withKA(ElevatorConstants.kA)
            .withKG(ElevatorConstants.kG);
        elevatorFXConfig.Slot0 = elevatorSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs elevatorCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ElevatorConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ElevatorConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ElevatorConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ElevatorConstants.kCurrentLimitEnable);
        elevatorFXConfig.CurrentLimits = elevatorCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ElevatorConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ElevatorConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ElevatorConstants.kMotionMagicJerk);
        elevatorFXConfig.MotionMagic = elevatorMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs elevatorSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(ElevatorConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(ElevatorConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(ElevatorConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(ElevatorConstants.kSoftForwardLimit);
        elevatorFXConfig.SoftwareLimitSwitch = elevatorSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs elevatorHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(false)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen); //Add autoset position on forward limit to appropriate number also.
        // elevatorFXConfig.HardwareLimitSwitch = elevatorHardwareLimitsConfigs;
        //Encoder
        if(ElevatorConstants.kUseCANcoder) {
            elevatorFXConfig.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kCANcoderID;
            elevatorFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            elevatorFXConfig.Feedback.RotorToSensorRatio = ElevatorConstants.kGearRatio;
            elevatorFXConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            elevatorFXConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;
        }
        //Neutral and Direction
        elevatorFXConfig.MotorOutput.NeutralMode = ElevatorConstants.kNeutralMode;
        elevatorFXConfig.MotorOutput.Inverted = (ElevatorConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        elevatorFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        //Climber
        //CANcoder
        climberCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        climberCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        climberCCConfig.MagnetSensor.MagnetOffset = ClimberConstants.kMagnetOffset;

        Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withKP(ClimberConstants.kP)
            .withKI(ClimberConstants.kI)
            .withKD(ClimberConstants.kD)
            .withKS(ClimberConstants.kS)
            .withKV(ClimberConstants.kV)
            .withKA(ClimberConstants.kA);
        climberFXConfig.Slot0 = climberSlot0Configs;
        //Current Limits
        CurrentLimitsConfigs climberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ClimberConstants.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ClimberConstants.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ClimberConstants.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ClimberConstants.kCurrentLimitEnable);
        climberFXConfig.CurrentLimits = climberCurrentLimitsConfigs;
        //Motion Magic
        MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ClimberConstants.kMotionMagicCruise)
            .withMotionMagicAcceleration(ClimberConstants.kMotionMagicAccel)
            .withMotionMagicJerk(ClimberConstants.kMotionMagicJerk);
        climberFXConfig.MotionMagic = climberMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs climberSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(ClimberConstants.kSoftReverseLimitEnable)
            .withReverseSoftLimitThreshold(ClimberConstants.kSoftReverseLimit)
            .withForwardSoftLimitEnable(ClimberConstants.kSoftForwardLimitEnable)
            .withForwardSoftLimitThreshold(ClimberConstants.kSoftForwardLimit);
        climberFXConfig.SoftwareLimitSwitch = climberSoftwareLimitSwitchConfigs;
        HardwareLimitSwitchConfigs climberHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        //     .withReverseLimitEnable(false)
        //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        //     .withReverseLimitAutosetPositionEnable(true)
        //     .withReverseLimitAutosetPositionValue(0.0)
            .withForwardLimitEnable(true)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed);
        climberFXConfig.HardwareLimitSwitch = climberHardwareLimitsConfigs;
        //Encoder
        if(ClimberConstants.kUseCANcoder) {
            climberFXConfig.Feedback.FeedbackRemoteSensorID = ClimberConstants.kCANcoderID;
            climberFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            climberFXConfig.Feedback.RotorToSensorRatio = ClimberConstants.kGearRatio;
            climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        } else {
            climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kGearRatio;
        }
        //Neutral and Direction
        climberFXConfig.MotorOutput.NeutralMode = ClimberConstants.kNeutralMode;
        climberFXConfig.MotorOutput.Inverted = (ClimberConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        climberFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

    }

    /**
     * Retries applying a configuration until it succeeds or attempts are exhausted.
     *
     * @param toApply Supplier that applies a configuration and returns a status code.
     */
    public void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = 5;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}
