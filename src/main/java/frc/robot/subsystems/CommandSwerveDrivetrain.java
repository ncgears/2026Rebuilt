package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.NCDebug;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@SuppressWarnings({"unused"})
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private static final String kTrackedTargetFieldObjectName = "Tracked Target";
    private static final double kTrackedTargetRingRadiusMeters = 0.20;
    private static final int kTrackedTargetRingPoints = 24;
    private boolean m_dashboardRegistered = false;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    /** Field visualization sendable published to dashboards. */
    public Field2d field = new Field2d();
    
    private boolean m_suppressFrontVision = false;
    private boolean m_suppressShooterVision = false;
    private boolean m_suppressBackVision = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following 
     * These are used for PathPlanner
    */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Swerve request to apply during field-centric path following 
     * These are used for Choreo
    */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(7,0, 0);
    private final PIDController m_pathYController = new PIDController(7, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(8, 0, 0);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
    }

    /** Initializes drivetrain state after construction. */
    public void init() {
        NCDebug.Debug.debug("Drivetrain: Initialized");
    }

    /**
     * Publishes drivetrain telemetry to SmartDashboard.
     */
    public void updateDashboards() {
        if (!GlobalConstants.telemetryAtLeast(SwerveConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
        // INFO level telemetry goes here
        if (!m_dashboardRegistered) {
            SmartDashboard.putData("Subsystems/Drivetrain/Field", getField());
            m_dashboardRegistered = true;
        }
        SmartDashboard.putBoolean("Subsystems/Drivetrain/HeadingLock", getHeadingLocked());
        SmartDashboard.putString("Subsystems/Drivetrain/HeadingLockColor", getHeadingLockedColor());
        SmartDashboard.putBoolean("Subsystems/Drivetrain/Vision/FrontSuppressed", isFrontVisionSuppressed());
        SmartDashboard.putBoolean("Subsystems/Drivetrain/Vision/ShooterSuppressed", isShooterVisionSuppressed());
        SmartDashboard.putBoolean("Subsystems/Drivetrain/Vision/BackSuppressed", isBackVisionSuppressed());
        SmartDashboard.putNumber("Subsystems/Drivetrain/TargetHeading", NCDebug.General.roundDouble(getTargetHeading(), 4));
        SmartDashboard.putNumber("Subsystems/Drivetrain/CurrentHeading", NCDebug.General.roundDouble(getBotHeading().getDegrees(), 4));
        SmartDashboard.putNumber("Subsystems/Drivetrain/HeadingError", NCDebug.General.roundDouble(getHeadingError().getDegrees(), 4));

        if (!GlobalConstants.telemetryAtLeast(SwerveConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
        // DEBUG level telemetry goes here
        SmartDashboard.putNumber("Subsystems/Drivetrain/PoseX", NCDebug.General.roundDouble(getBotPose().getX(), 4));
        SmartDashboard.putNumber("Subsystems/Drivetrain/PoseY", NCDebug.General.roundDouble(getBotPose().getY(), 4));
    }

    /**
     * Returns the current robot pose.
     *
     * @return Robot pose.
     */
    public Pose2d getBotPose() {
        return getState().Pose;
    }

    /**
     * Returns the robot heading from the drivetrain state.
     *
     * @return Robot heading.
     */
    public Rotation2d getBotHeading() {
        return getBotPose().getRotation();
    }

    /**
     * Returns the heading error between target and current heading.
     *
     * @return Heading error.
     */
    public Rotation2d getHeadingError() {
        if (!getHeadingLocked()) return Rotation2d.kZero;
        return getBotHeading().minus(RobotContainer.m_targetDirection);
    }

    /**
     * Sets whether front-camera vision corrections are suppressed.
     *
     * @param suppress True to suppress front vision.
     */
    public void setSuppressFrontVision(boolean suppress) {
        m_suppressFrontVision = suppress;
        NCDebug.Debug.debug((m_suppressFrontVision) ? "Drive: Front Vision Suppressed" : "Drive: Front Vision Unsuppressed");
    }

    /**
     * Creates a command to suppress front vision.
     *
     * @return Command that suppresses front vision.
     */
    public Command suppressFrontVisionC() {
        return runOnce(() -> setSuppressFrontVision(true));
    }

    /**
     * Creates a command to unsuppress front vision.
     *
     * @return Command that unsuppresses front vision.
     */
    public Command unsuppressFrontVisionC() {
        return runOnce(() -> setSuppressFrontVision(false));
    }

    /**
     * Sets whether shooter-camera vision corrections are suppressed.
     *
     * @param suppress True to suppress shooter vision.
     */
    public void setSuppressShooterVision(boolean suppress) {
        m_suppressShooterVision = suppress;
        NCDebug.Debug.debug((m_suppressShooterVision) ? "Drive: Shooter Vision Suppressed" : "Drive: Shooter Vision Unsuppressed");
    }

    /**
     * Creates a command to suppress shooter vision.
     *
     * @return Command that suppresses shooter vision.
     */
    public Command suppressShooterVisionC() {
        return runOnce(() -> setSuppressShooterVision(true));
    }

    /**
     * Creates a command to unsuppress shooter vision.
     *
     * @return Command that unsuppresses shooter vision.
     */
    public Command unsuppressShooterVisionC() {
        return runOnce(() -> setSuppressShooterVision(false));
    }

    /**
     * Sets whether back-camera vision corrections are suppressed.
     *
     * @param suppress True to suppress back vision.
     */
    public void setSuppressBackVision(boolean suppress) {
        m_suppressBackVision = suppress;
        NCDebug.Debug.debug((m_suppressBackVision) ? "Drive: Back Vision Suppressed" : "Drive: Back Vision Unsuppressed");
    }

    /**
     * Creates a command to suppress back vision.
     *
     * @return Command that suppresses back vision.
     */
    public Command suppressBackVisionC() {
        return runOnce(() -> setSuppressBackVision(true));
    }

    /**
     * Creates a command to unsuppress back vision.
     *
     * @return Command that unsuppresses back vision.
     */
    public Command unsuppressBackVisionC() {
        return runOnce(() -> setSuppressBackVision(false));
    }

    /** Automatically suppresses vision based on robot speed if enabled. */
    public void autoSuppressVision() {
        if(VisionConstants.kUseAutoSuppress) {
            // ChassisSpeeds speeds = getState().ChassisSpeeds;
            // //if the speed is over threshold, suppress vision measurements from being added to pose
            // m_suppressFrontVision = (
            //     Math.sqrt(
            //         Math.pow(speeds.vxMetersPerSecond,2) + 
            //         Math.pow(speeds.vyMetersPerSecond,2)
            //     ) >= VisionConstants.kAutosuppressSpeedMetersPerSecond);
        }
    }

    /**
     * Returns whether front vision is suppressed.
     *
     * @return True when suppressed.
     */
    public boolean isFrontVisionSuppressed() { return m_suppressFrontVision; }

    /**
     * Returns whether shooter vision is suppressed.
     *
     * @return True when suppressed.
     */
    public boolean isShooterVisionSuppressed() { return m_suppressShooterVision; }

    /**
     * Returns whether back vision is suppressed.
     *
     * @return True when suppressed.
     */
    public boolean isBackVisionSuppressed() { return m_suppressBackVision; }

    /**
     * Returns whether heading lock is active.
     *
     * @return True when heading lock is active.
     */
    public boolean getHeadingLocked() { return RobotContainer.m_targetLock; }

    /**
     * Returns the dashboard color for heading lock status.
     *
     * @return Hex color string.
     */
    public String getHeadingLockedColor() {
        return (getHeadingLocked()) ? DashboardConstants.Colors.GREEN : DashboardConstants.Colors.RED;
    }

    /**
     * Returns the target heading in degrees.
     *
     * @return Target heading in degrees.
     */
    public double getTargetHeading() { return RobotContainer.m_targetDirection.getDegrees(); }

    /**
     * Returns whether target tracking is active.
     *
     * @return True when tracking a target.
     */
    public boolean isTrackingTarget() { return RobotContainer.targeting.getTracking(); }

    /**
     * Returns the tracking target heading in degrees.
     *
     * @return Target heading in degrees.
     */
    public double getTrackingTargetHeading() {
        return Rotation2d.fromDegrees(RobotContainer.targeting.getTrackingTargetBearing()).getDegrees();
    }

    /**
     * Returns the Field2d instance for visualization.
     *
     * @return Field2d instance.
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Returns the alliance-adjusted field pose of the currently selected tracking target.
     *
     * @return Target pose in field coordinates.
     */
    private Pose2d getTrackedTargetPose() {
        var trackingTarget = RobotContainer.targeting.getTrackingTarget();
        return RobotContainer.isAllianceRed()
            ? trackingTarget.getRotatedPose().toPose2d()
            : trackingTarget.getPose().toPose2d();
    }

    /**
     * Updates the tracked target overlay on Field2d as a ring around the selected target.
     * The ring is published as a trajectory so it renders as a continuous loop.
     */
    private void updateTrackedTargetFieldObject() {
        if (!RobotContainer.targeting.getTracking()) {
            field.getObject(kTrackedTargetFieldObjectName).setPoses();
            return;
        }

        Pose2d targetPose = getTrackedTargetPose();
        Pose2d[] ringPoses = new Pose2d[kTrackedTargetRingPoints + 1];
        for (int i = 0; i <= kTrackedTargetRingPoints; i++) {
            double angleRadians = 2.0 * Math.PI * i / kTrackedTargetRingPoints;
            double x = targetPose.getX() + (kTrackedTargetRingRadiusMeters * Math.cos(angleRadians));
            double y = targetPose.getY() + (kTrackedTargetRingRadiusMeters * Math.sin(angleRadians));
            ringPoses[i] = new Pose2d(x, y, Rotation2d.fromRadians(angleRadians));
        }
        field.getObject(kTrackedTargetFieldObjectName).setPoses(ringPoses);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }    

    /**
     * Returns an array of TalonFX motors available for the orchestra
     * @return Array TalonFX[] of TalonFX devices
     */
    public TalonFX[] getMotors() {
        ArrayList<TalonFX> motors = new ArrayList<>();
        for (var module: getModules()) {
            motors.add(module.getDriveMotor());
            motors.add(module.getSteerMotor());
        }
        return motors.toArray(new TalonFX[motors.size()]);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply.
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * Returns the current swerve module positions.
     *
     * @return Array of module positions.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return getState().ModulePositions;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /** Runs periodic updates and applies operator perspective and vision corrections. */
    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        if (!m_suppressFrontVision) {
            RobotContainer.vision.correctFrontPoseWithVision();
        }
        if (!m_suppressShooterVision) {
            RobotContainer.vision.correctShooterPoseWithVision();
        }
        if (!m_suppressBackVision) {
            RobotContainer.vision.correctBackPoseWithVision();
        }
        updateTrackedTargetFieldObject();
        field.setRobotPose(this.getState().Pose);
        updateDashboards();
    }

    /**
     * Creates a command to reset the gyro and seed field-centric heading.
     *
     * @return Command that resets the gyro.
     */
    public Command resetGyroC() {
        return runOnce(() -> {
            seedFieldCentric();
            NCDebug.Debug.debug("Drive: Reset Gyro");
        });
    }

    /** Starts the faster simulation update loop. */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /** Add a fake vision reading to test vision corrections */
    public Command addFakeVisionReadingC() {
        return runOnce(() -> {
            addVisionMeasurement(new Pose2d(3,3,Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
            NCDebug.Debug.debug("Drive: Added fake vision measurement");
        });
    }
}

