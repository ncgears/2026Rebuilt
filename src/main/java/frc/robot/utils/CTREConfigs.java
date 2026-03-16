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
        public final TalonFXConfiguration shooterFrontFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration shooterBackFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration indexerFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration knuckleFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration climberFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration deployFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration liveBottomFXConfig = new TalonFXConfiguration();
        public final TalonFXConfiguration intakeFXConfig = new TalonFXConfiguration();
    //CANcoder
        public final CANcoderConfiguration deployCCConfig = new CANcoderConfiguration();

    /** Creates and populates CTRE configuration objects for all subsystems. */
    public CTREConfigs() {
        //#region Shooter Front
        //Shooter Front Configuration
        Slot0Configs shooterFrontSlot0Configs = new Slot0Configs()
            .withKP(ShooterConstants.Front.kP)
            .withKI(ShooterConstants.Front.kI)
            .withKD(ShooterConstants.Front.kD)
            .withKS(ShooterConstants.Front.kS)
            .withKV(ShooterConstants.Front.kV)
            .withKA(ShooterConstants.Front.kA);
        shooterFrontFXConfig.Slot0 = shooterFrontSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs shooterFrontCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.Front.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ShooterConstants.Front.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ShooterConstants.Front.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ShooterConstants.Front.kCurrentLimitEnable);
        shooterFrontFXConfig.CurrentLimits = shooterFrontCurrentLimitsConfigs;
        //Neutral and Direction
        shooterFrontFXConfig.MotorOutput.NeutralMode = ShooterConstants.Front.kNeutralMode;
        shooterFrontFXConfig.MotorOutput.Inverted = ShooterConstants.Front.kInverted;
        //Audio
        shooterFrontFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Shooter Back
        //Shooter Back Configuration
        Slot0Configs shooterBackSlot0Configs = new Slot0Configs()
            .withKP(ShooterConstants.Back.kP)
            .withKI(ShooterConstants.Back.kI)
            .withKD(ShooterConstants.Back.kD)
            .withKS(ShooterConstants.Back.kS)
            .withKV(ShooterConstants.Back.kV)
            .withKA(ShooterConstants.Back.kA);
        shooterBackFXConfig.Slot0 = shooterBackSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs shooterBackCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.Back.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ShooterConstants.Back.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ShooterConstants.Back.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ShooterConstants.Back.kCurrentLimitEnable);
        shooterBackFXConfig.CurrentLimits = shooterBackCurrentLimitsConfigs;
        //Neutral and Direction
        shooterBackFXConfig.MotorOutput.NeutralMode = ShooterConstants.Back.kNeutralMode;
        shooterBackFXConfig.MotorOutput.Inverted = ShooterConstants.Back.kInverted;
        //Audio
        shooterBackFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Indexer
        //Indexer Configuration
        Slot0Configs indexerSlot0Configs = new Slot0Configs()
            .withKP(IndexerConstants.Indexer.kP)
            .withKI(IndexerConstants.Indexer.kI)
            .withKD(IndexerConstants.Indexer.kD)
            .withKS(IndexerConstants.Indexer.kS)
            .withKV(IndexerConstants.Indexer.kV)
            .withKA(IndexerConstants.Indexer.kA);
       indexerFXConfig.Slot0 = indexerSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs indexerCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IndexerConstants.Indexer.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IndexerConstants.Indexer.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IndexerConstants.Indexer.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IndexerConstants.Indexer.kCurrentLimitEnable);
        indexerFXConfig.CurrentLimits = indexerCurrentLimitsConfigs;
        //Neutral and Direction
        indexerFXConfig.MotorOutput.NeutralMode = IndexerConstants.Indexer.kNeutralMode;
        indexerFXConfig.MotorOutput.Inverted = IndexerConstants.Indexer.kInverted;
        //Audio
        indexerFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Knuckle
        Slot0Configs knuckleSlot0Configs = new Slot0Configs()
            .withKP(IndexerConstants.Knuckle.kP)
            .withKI(IndexerConstants.Knuckle.kI)
            .withKD(IndexerConstants.Knuckle.kD)
            .withKS(IndexerConstants.Knuckle.kS)
            .withKV(IndexerConstants.Knuckle.kV)
            .withKA(IndexerConstants.Knuckle.kA);
       knuckleFXConfig.Slot0 = knuckleSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs knuckleCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IndexerConstants.Knuckle.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IndexerConstants.Knuckle.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IndexerConstants.Knuckle.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IndexerConstants.Knuckle.kCurrentLimitEnable);
        knuckleFXConfig.CurrentLimits = knuckleCurrentLimitsConfigs;
        //Neutral and Direction
        knuckleFXConfig.MotorOutput.NeutralMode = IndexerConstants.Knuckle.kNeutralMode;
        knuckleFXConfig.MotorOutput.Inverted = IndexerConstants.Knuckle.kInverted;
        //Audio
        knuckleFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region LiveBottom
        Slot0Configs liveBottomSlot0Configs = new Slot0Configs()
            .withKP(IndexerConstants.LiveBottom.kP)
            .withKI(IndexerConstants.LiveBottom.kI)
            .withKD(IndexerConstants.LiveBottom.kD)
            .withKS(IndexerConstants.LiveBottom.kS)
            .withKV(IndexerConstants.LiveBottom.kV)
            .withKA(IndexerConstants.LiveBottom.kA);
       liveBottomFXConfig.Slot0 = liveBottomSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs liveBottomCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IndexerConstants.LiveBottom.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IndexerConstants.LiveBottom.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IndexerConstants.LiveBottom.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IndexerConstants.LiveBottom.kCurrentLimitEnable);
        liveBottomFXConfig.CurrentLimits = liveBottomCurrentLimitsConfigs;
        //Neutral and Direction
        liveBottomFXConfig.MotorOutput.NeutralMode = IndexerConstants.LiveBottom.kNeutralMode;
        liveBottomFXConfig.MotorOutput.Inverted = IndexerConstants.LiveBottom.kInverted;
        //Audio
        liveBottomFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Intake
        Slot0Configs intakeSlot0Configs = new Slot0Configs()
            .withKP(IntakeConstants.Intake.kP)
            .withKI(IntakeConstants.Intake.kI)
            .withKD(IntakeConstants.Intake.kD)
            .withKS(IntakeConstants.Intake.kS)
            .withKV(IntakeConstants.Intake.kV)
            .withKA(IntakeConstants.Intake.kA);
        intakeFXConfig.Slot0 = intakeSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.Intake.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IntakeConstants.Intake.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IntakeConstants.Intake.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IntakeConstants.Intake.kCurrentLimitEnable);
        intakeFXConfig.CurrentLimits = intakeCurrentLimitsConfigs;
        //Neutral and Direction
        intakeFXConfig.MotorOutput.NeutralMode = IntakeConstants.Intake.kNeutralMode;
        intakeFXConfig.MotorOutput.Inverted = IntakeConstants.Intake.kInverted;
        //Audio
        intakeFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Deploy
        //CANcoder
        deployCCConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        deployCCConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        deployCCConfig.MagnetSensor.MagnetOffset = IntakeConstants.Deploy.kMagnetOffset;

        Slot0Configs deploySlot0Configs = new Slot0Configs()
            .withKP(IntakeConstants.Deploy.kP)
            .withKI(IntakeConstants.Deploy.kI)
            .withKD(IntakeConstants.Deploy.kD)
            .withKS(IntakeConstants.Deploy.kS)
            .withKV(IntakeConstants.Deploy.kV)
            .withKA(IntakeConstants.Deploy.kA);
       deployFXConfig.Slot0 = deploySlot0Configs;
        MotionMagicConfigs deployMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(IntakeConstants.Deploy.kMotionMagicCruise)
            .withMotionMagicAcceleration(IntakeConstants.Deploy.kMotionMagicAccel)
            .withMotionMagicJerk(IntakeConstants.Deploy.kMotionMagicJerk);
        deployFXConfig.MotionMagic = deployMotionMagicConfigs;

        //Current Limits
        CurrentLimitsConfigs deployCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.Deploy.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(IntakeConstants.Deploy.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(IntakeConstants.Deploy.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(IntakeConstants.Deploy.kCurrentLimitEnable);
        deployFXConfig.CurrentLimits = deployCurrentLimitsConfigs;
        //Neutral and Direction
        deployFXConfig.MotorOutput.NeutralMode = IntakeConstants.Deploy.kNeutralMode;
        deployFXConfig.MotorOutput.Inverted = IntakeConstants.Deploy.kInverted;
        //Software Limits
        SoftwareLimitSwitchConfigs deploySoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(IntakeConstants.Deploy.kSoftLimitEnable)
            .withReverseSoftLimitThreshold(IntakeConstants.Deploy.kSoftLimitLow)
            .withForwardSoftLimitEnable(IntakeConstants.Deploy.kSoftLimitEnable)
            .withForwardSoftLimitThreshold(IntakeConstants.Deploy.kSoftLimitHigh);
        deployFXConfig.SoftwareLimitSwitch = deploySoftwareLimitSwitchConfigs;
        //Encoder
        deployFXConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.Deploy.kCANcoderID;
        deployFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        deployFXConfig.Feedback.RotorToSensorRatio = IntakeConstants.Deploy.kRotorToSensorRatio;
        deployFXConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Deploy.kSensorToMechanismRatio;
        //Audio
        deployFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        //#region Climber
        Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withKP(ClimberConstants.Climber.kP)
            .withKI(ClimberConstants.Climber.kI)
            .withKD(ClimberConstants.Climber.kD)
            .withKS(ClimberConstants.Climber.kS)
            .withKV(ClimberConstants.Climber.kV)
            .withKA(ClimberConstants.Climber.kA);
       climberFXConfig.Slot0 = climberSlot0Configs;

        //Current Limits
        CurrentLimitsConfigs climberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ClimberConstants.Climber.kCurrentLimitAmps)
            // .withSupplyCurrentLowerLimit(ClimberConstants.Climber.kCurrentLimitThresholdAmps)
            // .withSupplyCurrentLowerTime(ClimberConstants.Climber.kCurrentLimitThresholdSecs)
            .withSupplyCurrentLimitEnable(ClimberConstants.Climber.kCurrentLimitEnable);
        climberFXConfig.CurrentLimits = climberCurrentLimitsConfigs;
        //Neutral and Direction
        climberFXConfig.MotorOutput.NeutralMode = ClimberConstants.Climber.kNeutralMode;
        climberFXConfig.MotorOutput.Inverted = ClimberConstants.Climber.kInverted;
        //Audio
        climberFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);
        //#endregion

        // //Climber
        // Slot0Configs climberSlot0Configs = new Slot0Configs()
        //     .withKP(ClimberConstants.kP)
        //     .withKI(ClimberConstants.kI)
        //     .withKD(ClimberConstants.kD)
        //     .withKS(ClimberConstants.kS)
        //     .withKV(ClimberConstants.kV)
        //     .withKA(ClimberConstants.kA);
        // climberFXConfig.Slot0 = climberSlot0Configs;
        // //Current Limits
        // CurrentLimitsConfigs climberCurrentLimitsConfigs = new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(ClimberConstants.kCurrentLimitAmps)
        //     // .withSupplyCurrentLowerLimit(ClimberConstants.kCurrentLimitThresholdAmps)
        //     // .withSupplyCurrentLowerTime(ClimberConstants.kCurrentLimitThresholdSecs)
        //     .withSupplyCurrentLimitEnable(ClimberConstants.kCurrentLimitEnable);
        // climberFXConfig.CurrentLimits = climberCurrentLimitsConfigs;
        // //Motion Magic
        // MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicCruiseVelocity(ClimberConstants.kMotionMagicCruise)
        //     .withMotionMagicAcceleration(ClimberConstants.kMotionMagicAccel)
        //     .withMotionMagicJerk(ClimberConstants.kMotionMagicJerk);
        // climberFXConfig.MotionMagic = climberMotionMagicConfigs;
        // //Mechanical Limits
        // SoftwareLimitSwitchConfigs climberSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        //     .withReverseSoftLimitEnable(ClimberConstants.kSoftReverseLimitEnable)
        //     .withReverseSoftLimitThreshold(ClimberConstants.kSoftReverseLimit)
        //     .withForwardSoftLimitEnable(ClimberConstants.kSoftForwardLimitEnable)
        //     .withForwardSoftLimitThreshold(ClimberConstants.kSoftForwardLimit);
        // climberFXConfig.SoftwareLimitSwitch = climberSoftwareLimitSwitchConfigs;
        // HardwareLimitSwitchConfigs climberHardwareLimitsConfigs = new HardwareLimitSwitchConfigs()
        // //     .withReverseLimitEnable(false)
        // //     .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        // //     .withReverseLimitAutosetPositionEnable(true)
        // //     .withReverseLimitAutosetPositionValue(0.0)
        //     .withForwardLimitEnable(true)
        //     .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed);
        // climberFXConfig.HardwareLimitSwitch = climberHardwareLimitsConfigs;
        // //Encoder
        // if(ClimberConstants.kUseCANcoder) {
        //     climberFXConfig.Feedback.FeedbackRemoteSensorID = ClimberConstants.kCANcoderID;
        //     climberFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        //     climberFXConfig.Feedback.RotorToSensorRatio = ClimberConstants.kGearRatio;
        //     climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kSensorGearRatio; //CANcoder is the same as mechanism
        // } else {
        //     climberFXConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kGearRatio;
        // }
        // //Neutral and Direction
        // climberFXConfig.MotorOutput.NeutralMode = ClimberConstants.kNeutralMode;
        // climberFXConfig.MotorOutput.Inverted = (ClimberConstants.kIsInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // //Audio
        // climberFXConfig.Audio = new AudioConfigs().withAllowMusicDurDisable(true);

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
