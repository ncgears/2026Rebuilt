// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.classes.Gyro;
import frc.robot.classes.Lighting;
import frc.robot.classes.Lighting.Colors;
import frc.robot.classes.Targeting.Targets;
import frc.robot.classes.NCOrchestra;
import frc.robot.classes.Targeting;
import frc.robot.classes.Vision;
import frc.robot.constants.*;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.InputAxis;
import frc.robot.utils.NCDebug;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.*;

public class RobotContainer {
    //#region Declarations
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final Lighting lighting = Lighting.getInstance();
    public static final Gyro gyro = Gyro.getInstance();
    public static final Vision vision = Vision.getInstance();
    private final NCOrchestra orchestra = NCOrchestra.getInstance();
    // public static final DriveSubsystem drive = DriveSubsystem.getInstance(); //must be after gyro
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //must be after gyro
    public static final Targeting targeting = Targeting.getInstance(); //must be after drive
    public static final PowerDistribution power = new PowerDistribution(1,ModuleType.kRev);
    // public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    public static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    
    public static Optional<Alliance> m_alliance = Optional.empty();

    private AutoFactory autoFactory;
    private AutoRoutines autoRoutines;
        
    private AutoChooser autoChooser = new AutoChooser("000: Do Nothing");
    //Sendables definitions
    private SendableChooser<Command> m_auto_chooser = new SendableChooser<>();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(SwerveConstants.kMaxAngularRate).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);
    /* Helpers for breaking and wheel strait testing */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /* Setup the drive methods for normal driving, robot strafing, and snap driving */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors    
    private final SwerveRequest.FieldCentric targetingDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.035)
        .withRotationalDeadband(0.0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotdrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(SnapDriveConstants.kP, SnapDriveConstants.kI, SnapDriveConstants.kD)
        .withDeadband(MaxSpeed * 0.035)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // .withDriveRequestType(DriveRequestType.Velocity);
    public static Rotation2d m_targetDirection = new Rotation2d();
    public static Boolean m_trackingOverride = false;
    public static Boolean m_targetLock = false;

    /* Setup the joysticks */
    private final CommandXboxController dj = new CommandXboxController(OIConstants.JoyDriverID);
    private final CommandXboxController oj = new CommandXboxController(OIConstants.JoyOperID);
    // private final CommandStadiaController dj = new CommandStadiaController(OIConstants.JoyDriverID);
    // private final CommandStadiaController oj = new CommandStadiaController(OIConstants.JoyOperID);
    private final CommandStadiaController pj = new CommandStadiaController(OIConstants.JoyProgID);
    //#endregion Declarations

    /** Creates the robot container and configures subsystems and bindings. */
    public RobotContainer() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        snapDrive.HeadingController = new PhoenixPIDController(SnapDriveConstants.kP, SnapDriveConstants.kI, SnapDriveConstants.kD);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
        snapDrive.HeadingController.setTolerance(Math.toRadians(SnapDriveConstants.kToleranceDegrees));
        double joystickDeadband = (Robot.isSimulation()) ? OIConstants.kMinDeadband * 2.0 : OIConstants.kMinDeadband;

        final InputAxis m_fieldX = new InputAxis("Forward", dj::getLeftY)
            .withDeadband(joystickDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_fieldY = new InputAxis("Strafe", dj::getLeftX)
            .withDeadband(joystickDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_rotate = new InputAxis("Rotate", dj::getRightX)
            .withDeadband(joystickDeadband)
            .withInvert(true);
        final InputAxis m_climbAxis = new InputAxis("Climber", oj::getRightY)
            .withDeadband(joystickDeadband)
            .withMultiplier(ClimberConstants.kClimbPower)
            .withSquaring(true)
            .withInvert(true);
        final InputAxis m_deployAxis = new InputAxis("DeployManual", oj::getLeftY)
            .withDeadband(joystickDeadband)
            .withMultiplier(IntakeConstants.Deploy.kManualDutyCycleMax)
            .withInvert(true)
            .withSquaring(true);
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose,
            drivetrain::resetPose,
            drivetrain::followPath,
            true,
            drivetrain
        );
        autoRoutines = new AutoRoutines(autoFactory);
        
        initOrchestra();
        configureBindings();
        buildDashboards();

        //#region Default Commands
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double fieldX = m_fieldX.getAsDouble();
                double fieldY = m_fieldY.getAsDouble();
                double rot = m_rotate.getAsDouble();
                boolean targetingActive = targeting.getTracking() && !m_trackingOverride;
                boolean requestingTranslation = Math.abs(fieldX) > 0.0 || Math.abs(fieldY) > 0.0;
                double requestedVX = fieldX * MaxSpeed;
                double requestedVY = fieldY * MaxSpeed;
                if (targetingActive) {
                    double targetingMaxSpeed = TargetingConstants.kMaxSpeedMetersPerSecond.in(MetersPerSecond);
                    double requestedSpeed = Math.hypot(requestedVX, requestedVY);
                    if (requestedSpeed > targetingMaxSpeed && requestedSpeed > 1e-9) {
                        double scale = targetingMaxSpeed / requestedSpeed;
                        requestedVX *= scale;
                        requestedVY *= scale;
                    }
                }
                if (targetingActive) {
                    Rotation2d targetBearing = Rotation2d.fromDegrees(targeting.getBearingOfTarget(targeting.getTrackingTarget()));
                    Rotation2d perspective = (isAllianceRed()) ? Rotation2d.k180deg : Rotation2d.kZero;
                    m_targetDirection = targetBearing.minus(perspective);
                    targeting.setTrackingReady();
                } else {
                    if(Math.abs(rot) > 0.0) {
                        if(m_targetLock) NCDebug.Debug.debug("Drive: Heading Unlocked");
                        m_targetLock = false;
                    } else {
                        if(!m_targetLock && requestingTranslation) {
                            m_targetDirection = drivetrain.getBotHeading();
                            // NCDebug.Debug.debug("Drive: Heading Locked to "+drivetrain.getBotHeading().getDegrees());
                            m_targetLock = true;
                        }
                    }
                    m_targetLock = false; //disable!
                }
                if (targetingActive) {
                    double currentHeadingRadians = drivetrain.getBotHeading().getRadians();
                    double targetHeadingRadians = m_targetDirection.getRadians();
                    double headingErrorRadians = MathUtil.angleModulus(targetHeadingRadians - currentHeadingRadians);
                    double commandedOmega = snapDrive.HeadingController.calculate(
                        currentHeadingRadians,
                        targetHeadingRadians,
                        Utils.getCurrentTimeSeconds()
                    );
                    double minTurnRate = Math.toRadians(TargetingConstants.kMinTurnRateDegreesPerSecond);
                    double minTurnErrorRadians = Math.toRadians(TargetingConstants.kMinTurnRateErrorDegrees);
                    if (Math.abs(headingErrorRadians) > minTurnErrorRadians && Math.abs(commandedOmega) < minTurnRate) {
                        commandedOmega = Math.copySign(minTurnRate, headingErrorRadians);
                    }
                    commandedOmega = MathUtil.clamp(commandedOmega, -MaxAngularRate, MaxAngularRate);
                    return targetingDrive.withVelocityX(requestedVX) // Drive forward with negative Y (forward)
                        .withVelocityY(requestedVY) // Drive left with negative X (left)
                        .withRotationalRate(commandedOmega);
                } else if (m_targetLock || targeting.getTracking()) { //specific facing angle
                    // NCDebug.Debug.debug("drive with facing angle");
                    return snapDrive.withVelocityX(requestedVX) // Drive forward with negative Y (forward)
                        .withVelocityY(requestedVY) // Drive left with negative X (left)
                        .withTargetDirection(m_targetDirection);
                } else {
                    // NCDebug.Debug.debug("drive with unlocked");
                    return drive.withVelocityX(requestedVX) // Drive forward with negative Y (forward)
                        .withVelocityY(requestedVY) // Drive left with negative X (left)
                        .withRotationalRate(rot * MaxAngularRate); // Drive counterclockwise with negative X (left)
                        // .withRotationalRate(m_rotate.getAsDouble() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                }
            })
        );
        intake.setDefaultCommand(
            intake.deployManualC(m_deployAxis)
        );
        //#endregion Default Commands
      
    }
    
    /** Initializes the CTRE orchestra with motors used for music playback. */
    private void initOrchestra() {
        /**
         * This adds instruments to the orchestra. 
         * It is recommended there is at least 6-8 TalonFX devices in the orchestra.
         * Falcon500 are much louder than Krakens
         * 
         * This depends on each subsystem having a getMotors() method that returns an array of the TalonFX devices
         */
        if(AudioConstants.isEnabled) {
            ArrayList<TalonFX> instrumentsAll = new ArrayList<>();
            // The following subsystems return an array of motors (TalonFX[])
            for (TalonFX motor: drivetrain.getMotors()) {
                instrumentsAll.add(motor);
            }
            // for (TalonFX motor: shooter.getMotors()) {
            //   instrumentsAll.add(motor);
            // }
            TalonFX[] instruments = instrumentsAll.toArray(new TalonFX[instrumentsAll.size()]);
            orchestra.apply(instruments);
        }
    }

    /**
     * This performs robot reset initializations when the disabled() trigger fires.
     */
    private void resetRobot() {
        lighting.init();
        targeting.init();
        m_alliance = DriverStation.getAlliance();
        drivetrain.init();
        // climber.init();
        intake.init();
        intake.setDeployStow();
        indexer.init();
        shooter.init();
    }

    /** Sets subsystems to a safe neutral state while disabled. */
    public void neutralRobot() {
        // climber.neutralCommand().ignoringDisable(true);
        intake.neutralCommand().ignoringDisable(true);
        indexer.neutralCommand().ignoringDisable(true);
        shooter.neutralCommand().ignoringDisable(true);
    }

    /**
     * Monitors alliance color while disabled and reseeds targeting pose when it changes.
     * This avoids unnecessary reseeding while enabled, where vision normally refines pose.
     */
    public void monitorAllianceChangeWhileDisabled() {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && !currentAlliance.equals(m_alliance)) {
            NCDebug.Debug.debug("Robot: Alliance changed while disabled to " + currentAlliance.get() + ", reseeding pose");
            m_alliance = currentAlliance;
            targeting.init();
        } else if (currentAlliance.isEmpty() && m_alliance.isPresent()) {
            m_alliance = Optional.empty();
        }
    }

    /**
     * Returns true if the alliance is red, otherwise false for blue.
     *
     * @return True when on red alliance.
     */
    public static boolean isAllianceRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
    
    /** Binds controller inputs to commands and triggers. */
    private void configureBindings() {
        //#region RobotMode Triggers
        // bind to the disabled() trigger which happens any time the robot is disabled
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(this::resetRobot).ignoringDisable(true)
            .alongWith(lighting.danceParty()).until(RobotModeTriggers.disabled().negate()).ignoringDisable(true)
            // .alongWith(elevator.ElevatorStopC().ignoringDisable(true)).ignoringDisable(true)
            .alongWith(new InstantCommand(() -> {m_targetLock = false;})).ignoringDisable(true)
        );
        // bind to autonomous(), teleop(), and test() so this runs any time the robot is enabled
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).or(RobotModeTriggers.test()).onTrue(
            new InstantCommand(orchestra::stop).ignoringDisable(true)
            .andThen(lighting.setColorCommand(Colors.OFF)).ignoringDisable(true)
            .andThen(new InstantCommand(intake::setDeployOut))
            // .andThen(coral.CoralPositionC(CoralSubsystem.Position.SCORE))
            // .andThen(new InstantCommand(() -> elevator.gotoTargetPosition()))
        );
        //#endregion

        //#region Trigger Actions
        if(!ClimberConstants.isDisabled) {
            /**
             * This monitors the hasCage trigger and immediately starts climbing until the climbComplete trigger, then goes to holding mode
             * Once the climbComplete trigger fires, the climber stops after 2 seconds
             */
            // climber.hasCage.and(climber.climbComplete.negate()).onTrue(
            //   climber.climberMoveC(() -> ClimberConstants.kClimbPower)
            //   .alongWith(lighting.climbStartColor())
            // );
            // climber.climbComplete.onTrue(
            //   climber.climberStopC()
            //   .alongWith(lighting.climbDoneColor())
            // );
        }
        //#endregion Trigger Actions

        //#region Driver Joystick
        if(AudioConstants.isEnabled) {
            /** Manage Music - Song list
            *  Brawl-Theme.chrp
            *  Megalovania.chrp
            *  Rickroll.chrp
            *  Still-Alive.chrp
            */
            // dj.stadia().onTrue(new InstantCommand(() -> {
            //     if(orchestra.isPlaying()) {
            //         orchestra.stop();
            //     } else {
            //         orchestra.withMusic("Still-Alive.chrp").play();
            //     }
            // }).ignoringDisable(true));
        }

        //POV left and right are robot-centric strafing
        dj.povLeft().whileTrue(drivetrain.applyRequest(() -> 
                robotdrive.withVelocityX(0).withVelocityY(SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeLeft"); })
            )
        );
        dj.povRight().whileTrue(drivetrain.applyRequest(() -> 
            robotdrive.withVelocityX(0).withVelocityY(-SwerveConstants.kAlignStrafeSpeed)
            ).alongWith(
                new InstantCommand(() -> { NCDebug.Debug.debug("Debug: StrafeRight"); })
            )
        );

        dj.leftTrigger()
            .onTrue(
                new InstantCommand(() -> { m_trackingOverride = true; m_targetLock = false; })
                    .andThen(targeting.trackingOverrideStartC())
            )
            .onFalse(
                new InstantCommand(() -> { m_trackingOverride = false; })
                    .andThen(targeting.trackingOverrideStopC())
            );

        dj.start().onTrue(drivetrain.resetGyroC());
        // dj.back().onTrue(drivetrain.addFakeVisionReadingC());

        //hold A to apply brake
        dj.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //hold B to point the wheels forward
        dj.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-dj.getLeftY(),-dj.getLeftX()))
        ));

        // dj.rightTrigger().whileTrue(drivetrain.applyRequest(() -> {
        //         double fieldX = m_fieldX.getAsDouble();
        //         double fieldY = m_fieldY.getAsDouble();
        //         double rot = m_rotate.getAsDouble();
        //         boolean targetingActive = targeting.getTracking() && !m_trackingOverride;
        //         boolean requestingTranslation = Math.abs(fieldX) > 0.0 || Math.abs(fieldY) > 0.0;
        //         double requestedVX = fieldX * MaxSpeed;
        //         double requestedVY = fieldY * MaxSpeed;
        //         robotdrive.withVelocityX(requestedVX).withVelocityY(requestedVY)
        //             .withRotationalRate(rot * MaxAngularRate); // Drive counterclockwise with negative X (left)
        //         })
        // );

        // Right Trigger bindings for testing with a single controller
        // dj.rightTrigger().onTrue(
        //     targeting.setTrackingHubC()
        //     .andThen(targeting.trackingStartC())
        // ).onFalse(
        //     targeting.trackingStopC()
        // );

        // dj.rightTrigger().whileTrue(
        //     Commands.sequence(
        //         targeting.setTrackingHubC(),
        //         targeting.trackingStartC(),
        //         Commands.deadline(
        //             wait(ShooterConstants.kSpinupDelaySeconds),
        //             Commands.run(
        //                 () -> shooter.startShooter(targeting.getDistanceOfTarget(targeting.getTrackingTarget())),
        //                 shooter
        //             )
        //         ),
        //         Commands.run(
        //             () -> {
        //                 shooter.startShooter(targeting.getDistanceOfTarget(targeting.getTrackingTarget()));
        //                 indexer.startIndexer();
        //             },
        //             shooter,
        //             indexer
        //         )
        //     )
        // ).onFalse(
        //     targeting.trackingStopC()
        //     .andThen(
        //         Commands.parallel(
        //             shooter.neutralCommand(),
        //             indexer.neutralCommand()
        //         )
        //     )
        // );


        // reset the field-centric heading on hamburger button press
        dj.start().onTrue(drivetrain.resetGyroC());
        //#endregion Driver Joystick

        //#region Operator Joystick
          /*
           * lt - tracking
           * +dpad left (left pocket)
           * +dpad right (right pocket)
           * rt - fire
           * lb - shift modifier
           * rb - intake
           * a - momentary undeploy
           * x - reverse indexer+knuckle
           * y - deploy
           * lb+y - stow intake
           * b - 
           * start - climber up  
           * back - shooter reverse
           * left y - manual deploy duty
           * rstick -
           * dpad up -
           * dpad down -
           */

        /** OJ Left Trigger - Tracking mode while held */
        oj.leftTrigger().onTrue(
            targeting.setTrackingHubC()
            .andThen(targeting.trackingStartC())
        ).onFalse(
            targeting.trackingStopC()
        );
        oj.povLeft().or(oj.povUpLeft()).or(oj.povDownLeft()).onTrue(
            targeting.setTrackingPocketLeftC()
            .andThen(targeting.trackingStartC())
        ).onFalse(
            targeting.trackingStopC()
        );
        oj.povRight().or(oj.povUpRight()).or(oj.povDownRight()).onTrue(
            targeting.setTrackingPocketRightC()
            .andThen(targeting.trackingStartC())
        ).onFalse(
            targeting.trackingStopC()
        );

        /** OJ Right Trigger - Scoring sequence - continuously update shooter from distance and feed after spinup */
        oj.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.deadline(
                    wait(ShooterConstants.kSpinupDelaySeconds),
                    Commands.run(
                        () -> shooter.startShooter(targeting.getDistanceOfTarget(targeting.getTrackingTarget())),
                        shooter
                    )
                ),
                Commands.run(
                    () -> {
                        shooter.startShooter(targeting.getDistanceOfTarget(targeting.getTrackingTarget()));
                        indexer.startIndexer();
                    },
                    shooter,
                    indexer
                )
            )
        ).onFalse(
            Commands.parallel(
                shooter.neutralCommand(),
                indexer.neutralCommand()
            )
        );

        /** OJ Ellipses (while held) - Run shooter in reverse for unjam/clear. */
        oj.back().whileTrue(
            shooter.setShooterSpeedC(-ShooterConstants.kReverseRPM)
            .andThen(indexer.setKnuckleSpeedC(-IndexerConstants.Knuckle.kReverseRPM))
        ).onFalse(
            Commands.parallel(
                shooter.neutralCommand(),
                indexer.neutralCommand()
            )
        );

        /** OJ Right Bumper (without Left Bumper) - Run intake forward while held.
         * Uses slow speed when shooter is actively shooting to avoid command contention. */
        oj.leftBumper().negate().and(oj.rightBumper()).whileTrue(
            intake.setIntakeForwardAdaptiveC(shooter::isShooting)
        );

        /** OJ Left Bumper + Right Bumper - Run intake reverse while held */
        oj.leftBumper().and(oj.rightBumper()).onTrue(
            intake.setIntakeReverseC()
        );

        /** OJ Right Bumper released - Stop intake */
        oj.rightBumper().onFalse(
            intake.setIntakeStopC()
        );

        /** OJ Y (without Left Bumper) - Set deploy to OUT position. */
        oj.leftBumper().negate().and(oj.y()).onTrue(
            intake.setDeployOutC()
        );

        /** OJ Left Bumper + Y - Set deploy to STOW position. */
        oj.leftBumper().and(oj.y()).onTrue(
            intake.setDeployStowC()
        );

        /** OJ A (while held) - Set deploy to UNJAM, then return to OUT on release. */
        oj.a().onTrue(
            intake.setDeployUnjamC()
        ).onFalse(
            intake.setDeployOutC()
        );

        /** OJ X (while held) - Indexer UNJAM in reverse, then stop on release. */
        oj.x().onTrue(
            indexer.setIndexerUnjamC()
        ).onFalse(
            indexer.neutralCommand()
        );
        //#endregion Operator Joystick

        //#region Programmer Joystick
        // Run SysId routines when holding ellipses/google and X/Y.
        // Note that each routine should be run exactly once in a single log.
        var m_mechanism = drivetrain; //drivetrain, elevator, coral, algae, climber
        // pj.ellipses().and(pj.a()).whileTrue(m_mechanism.runSysIdCommand());
        
        //seperately, but would need to use logic to see if we are atLimit
        pj.leftBumper().onTrue(Commands.runOnce(SignalLogger::start)); 
        pj.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        pj.ellipses().and(pj.y()).whileTrue(m_mechanism.sysIdDynamic(Direction.kForward));
        pj.ellipses().and(pj.x()).whileTrue(m_mechanism.sysIdDynamic(Direction.kReverse));
        pj.google().and(pj.y()).whileTrue(m_mechanism.sysIdQuasistatic(Direction.kForward));
        pj.google().and(pj.x()).whileTrue(m_mechanism.sysIdQuasistatic(Direction.kReverse));
        //#endregion Programmer Joystick

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    /**
     * Use this to pass the named command to the main Robot class.
     * @return command
     */
    public Command getAutonomousCommand() {
        if(AutonConstants.isDisabled) {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+m_auto_chooser.getSelected().getName());
            return m_auto_chooser.getSelected();
        } else {
            NCDebug.Debug.debug("Robot: Selected auton routine is "+autoChooser.selectedCommand().getName());
            return autoChooser.selectedCommand();
        }
    }

    //#region Dashboard
    //From here down is all used for building the shuffleboard
    /** Builds all driver/system dashboards. */
    public void buildDashboards(){
        //List of Widgets: https://github.com/Gold872/elastic-dashboard/wiki/Widgets-List-&-Properties-Reference
        buildAutonChooser();
        gyro.buildDashboards();
    }    
    
    /** Builds and populates the autonomous chooser. */
    public void buildAutonChooser() {
        //This builds the auton chooser, giving driver friendly names to the commands from above
        if(AutonConstants.isDisabled) {
            m_auto_chooser.setDefaultOption("00: None (Auto Disabled)", Commands.none());
            SmartDashboard.putData("Autonomous Chooser", m_auto_chooser);
        } else {
            if(AutonConstants.kUseChoreo) {
                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);
                autoChooser.addRoutine("101: sTL-Direct-1Pass", autoRoutines::sTLDirect1Pass);
                autoChooser.addRoutine("102: sTL-Direct-2Pass", autoRoutines::sTLDirect2Pass);
                autoChooser.addRoutine("201: sTR-Direct-1Pass", autoRoutines::sTRDirect1Pass);
                // autoChooser.addRoutine("202: sTR-Direct-2Pass", autoRoutines::sTRDirect2Pass);
                autoChooser.addRoutine("999: Test Run", autoRoutines::testRun);

                ShuffleboardTab systemTab = Shuffleboard.getTab("Auton");
                systemTab.add("Autonomous Chooser", autoChooser);

                //I feel dirty using shuffleboard for this, but below is not working
                // SmartDashboard.putData("Autonomous Chooser", autoChooser);
                // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
            }
        }
    }

    //#endregion Dashboard

    //#region Convenience
    //This command is a shortcut for WaitCommand
    /**
     * Convenience wrapper for {@link WaitCommand}.
     *
     * @param seconds Time to wait.
     * @return Wait command.
     */
    private Command wait(double seconds) {
      return new WaitCommand(seconds);
    }
    //This command does nothing
    /**
     * Returns a no-op command.
     *
     * @return No-op command.
     */
    private Command noop() {
      return Commands.none();
    }
    //#endregion
}
