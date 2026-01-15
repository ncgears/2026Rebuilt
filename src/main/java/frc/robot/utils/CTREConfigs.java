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
        public final TalonFXConfiguration indexerFeedFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration indexerBeltFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration shooterTiltFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration elevatorFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
    //TalonFXS
        public final TalonFXSConfiguration intakeFXSConfig = new TalonFXSConfiguration();
        public final TalonFXSConfiguration shooterFrontFXSConfig = new TalonFXSConfiguration();
        public final TalonFXSConfiguration shooterBackFXSConfig = new TalonFXSConfiguration();
    //CANcoder
        public final CANcoderConfiguration indexerFeedCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration shooterCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration elevatorCCConfig = new CANcoderConfiguration();
        public final CANcoderConfiguration climberCCConfig = new CANcoderConfiguration();

    /** Creates and populates CTRE configuration objects for all subsystems. */
    public CTREConfigs() {
        //Indexer Configuration
        Slot0Configs indexerFeedSlot0Configs = new Slot0Configs()
            .withKP(IndexerConstants.Feed.kP)
            .withKI(IndexerConstants.Feed.kI)
            .withKD(IndexerConstants.Feed.kD)
            .withKS(IndexerConstants.Feed.kS)
            .withKV(IndexerConstants.Feed.kV)
            .withKA(IndexerConstants.Feed.kA);
        indexerFeedFXConfig.Slot0 = indexerFeedSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs indexerFeedCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IndexerConstants.Feed.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IndexerConstants.Feed.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IndexerConstants.Feed.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IndexerConstants.Feed.kCurrentLimitEnable);
        indexerFeedFXConfig.CurrentLimits = indexerFeedCurrentLimitsConfigs;
        //Motion Magic
        // MotionMagicConfigs indexerFeedMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicCruiseVelocity(IndexerConstants.kMotionMagicCruise)
        //     .withMotionMagicAcceleration(IndexerConstants.kMotionMagicAccel)
        //     .withMotionMagicJerk(IndexerConstants.kMotionMagicJerk);
        // indexerFeedFXConfig.MotionMagic = indexerFeedMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs indexerFeedSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        indexerFeedFXConfig.SoftwareLimitSwitch = indexerFeedSoftwareLimitSwitchConfigs;
        //Neutral and Direction
        indexerFeedFXConfig.MotorOutput.NeutralMode = IndexerConstants.Feed.kNeutralMode;
        indexerFeedFXConfig.MotorOutput.Inverted = (IndexerConstants.Feed.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        indexerFeedFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

        Slot0Configs indexerBeltSlot0Configs = new Slot0Configs()
            .withKP(IndexerConstants.Belt.kP)
            .withKI(IndexerConstants.Belt.kI)
            .withKD(IndexerConstants.Belt.kD)
            .withKS(IndexerConstants.Belt.kS)
            .withKV(IndexerConstants.Belt.kV)
            .withKA(IndexerConstants.Belt.kA);
        indexerBeltFXConfig.Slot0 = indexerBeltSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs indexerBeltCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IndexerConstants.Belt.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IndexerConstants.Belt.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IndexerConstants.Belt.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IndexerConstants.Belt.kCurrentLimitEnable);
        indexerBeltFXConfig.CurrentLimits = indexerBeltCurrentLimitsConfigs;
        //Motion Magic
        // MotionMagicConfigs indexerBeltMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicCruiseVelocity(IndexerConstants.kMotionMagicCruise)
        //     .withMotionMagicAcceleration(IndexerConstants.kMotionMagicAccel)
        //     .withMotionMagicJerk(IndexerConstants.kMotionMagicJerk);
        // indexerBeltFXConfig.MotionMagic = indexerBeltMotionMagicConfigs;
        //Mechanical Limits
        SoftwareLimitSwitchConfigs indexerBeltSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        indexerBeltFXConfig.SoftwareLimitSwitch = indexerBeltSoftwareLimitSwitchConfigs;
        //Neutral and Direction
        indexerBeltFXConfig.MotorOutput.NeutralMode = IndexerConstants.Belt.kNeutralMode;
        indexerBeltFXConfig.MotorOutput.Inverted = (IndexerConstants.Belt.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        //Audio
        indexerBeltFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

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
        // HardwareLimitSwitchConfigs indexerFeedHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
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
