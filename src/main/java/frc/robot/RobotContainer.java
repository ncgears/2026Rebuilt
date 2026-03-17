// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Map;
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
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    
    public static Optional<Alliance> m_alliance;

    private AutoFactory autoFactory;
    private AutoRoutines autoRoutines;
        
    private final AutoChooser autoChooser = new AutoChooser("000: Do Nothing");
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
    private final SwerveRequest.RobotCentric robotdrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(7.0,0.0,0.0)
        .withDeadband(MaxSpeed * 0.035)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // .withDriveRequestType(DriveRequestType.Velocity);
    public static Rotation2d m_targetDirection = new Rotation2d();
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

        snapDrive.HeadingController = new PhoenixPIDController(5,0,0);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI,Math.PI);

        final InputAxis m_fieldX = new InputAxis("Forward", dj::getLeftY)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_fieldY = new InputAxis("Strafe", dj::getLeftX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true)
            .withSquaring(false);
        final InputAxis m_rotate = new InputAxis("Rotate", dj::getRightX)
            .withDeadband(OIConstants.kMinDeadband)
            .withInvert(true);
        final InputAxis m_climbAxis = new InputAxis("Climber", oj::getRightY)
            .withDeadband(OIConstants.kMinDeadband)
            .withMultiplier(ClimberConstants.kClimbPower)
            .withSquaring(true)
            .withInvert(true);
        final InputAxis m_deployAxis = new InputAxis("DeployManual", oj::getLeftY)
            .withDeadband(OIConstants.kMinDeadband)
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
              double rot = m_rotate.getAsDouble();
              if(Math.abs(rot) > 0) { //turning
                if(m_targetLock) NCDebug.Debug.debug("Drive: Heading Unlocked");
                m_targetLock = false;
              }
              if(m_targetLock) { //specific facing angle
                // NCDebug.Debug.debug("drive with facing angle");
                return snapDrive.withVelocityX(m_fieldX.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(m_fieldY.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
                .withTargetDirection(m_targetDirection);
              } else {
                // NCDebug.Debug.debug("drive with unlocked");
                return drive.withVelocityX(m_fieldX.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(m_fieldY.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
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
        drivetrain.init();
        // climber.init();
        intake.init();
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
        // bind to the autonomous() and teleop() trigger which happens any time the robot is enabled in either of those modes
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(
            new InstantCommand(orchestra::stop).ignoringDisable(true)
            .andThen(lighting.setColorCommand(Colors.OFF)).ignoringDisable(true)
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

        // dj.frame().onTrue((Commands.runOnce(drivetrain::zeroGyro)));
        // dj.stadia().onTrue(Commands.runOnce(drivetrain::addFakeVisionReading));

        //hold A to apply brake
        dj.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //hold B to point the wheels forward
        dj.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-dj.getLeftY(),-dj.getLeftX()))
        ));

        //target tracking/alignment buttons
        /** DJ NO Left Trigger...
         * + Left Bumper - Align to Reef Front Left face
         * + Right Bumper - Align to Reef Front Center face
         * + Right Trigger - Align to Reef Front Right face
         * DJ YES Left Trigger...
         * + Left Bumper - Align to Reef Rear Left face
         * + Right Bumper - Align to Reef Rear Center face
         * + Right Trigger - Align to Reef Rear Right face
         */
        dj.leftTrigger().negate().and(dj.leftBumper()).onTrue(updateTargetC(Targets.REEF_FRONT_LEFT_C));
        dj.leftTrigger().negate().and(dj.rightBumper()).onTrue(updateTargetC(Targets.REEF_FRONT_CENTER_C));
        dj.leftTrigger().negate().and(dj.rightTrigger()).onTrue(updateTargetC(Targets.REEF_FRONT_RIGHT_C));
        dj.leftTrigger().and(dj.leftBumper()).onTrue(updateTargetC(Targets.REEF_BACK_LEFT_C));
        dj.leftTrigger().and(dj.rightBumper()).onTrue(updateTargetC(Targets.REEF_BACK_CENTER_C));
        dj.leftTrigger().and(dj.rightTrigger()).onTrue(updateTargetC(Targets.REEF_BACK_RIGHT_C));
        /** DJ Left Trigger (While Held) - Right Stick is facing angle instead of turn */
        dj.leftTrigger().whileTrue(
          drivetrain.applyRequest(() -> {
            // Use right joystick raw for heading
            double headingX = -dj.getRightY(); // joystick -Y is forward, +X heading
            double headingY = -dj.getRightX(); // joystick -X is left, +Y heading
            if ((MathUtil.applyDeadband(headingX, 0.25) != 0.0 || MathUtil.applyDeadband(headingY,0.25) != 0.0) && Math.hypot(headingX, headingY) > 0.1) {
              m_targetDirection = new Rotation2d(headingX, headingY);
              // NCDebug.Debug.debug("Drive: Heading Locked to "+NCDebug.General.roundDouble(m_targetDirection.getDegrees(),2));
            } else {
              // m_targetDirection = Rotation2d.kZero;
              if(!m_targetLock) {
                m_targetDirection = drivetrain.getBotHeading();
                // NCDebug.Debug.debug("Drive: Heading Locked to "+drivetrain.getBotHeading().getDegrees());
              }
              m_targetLock = true;
            }
            return snapDrive.withVelocityX(-dj.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-dj.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withTargetDirection(m_targetDirection);
          })
        );
        
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
           * hamburger - climber up  
           * ellipses - shooter reverse
           * left y - manual deploy duty
           * rstick -
           * dpad up -
           * dpad down -
           */

        /** OJ Left Trigger - Tracking mode while held */
        oj.leftTrigger().onTrue(
            noop()
            // TODO: Start tracking mode.
            // TODO: Set default tracking target to HUB while LT is held.
        ).onFalse(
            noop()
            // TODO: Stop tracking mode and restore normal targeting behavior.
        );

        /** OJ Left Trigger + POV Left - Set tracking target to left pocket */
        oj.leftTrigger().and(oj.povLeft()).onTrue(
            noop()
            // TODO: Set tracking target to LEFT_POCKET.
        );

        /** OJ Left Trigger + POV Right - Set tracking target to right pocket */
        oj.leftTrigger().and(oj.povRight()).onTrue(
            noop()
            // TODO: Set tracking target to RIGHT_POCKET.
        );

        /** OJ Right Trigger - Scoring sequence - spin up shooter, wait, spin up indexer */
        oj.rightTrigger().onTrue(
            shooter.startShooterC()
                .andThen(wait(ShooterConstants.kSpinupDelaySeconds))
                .andThen(indexer.startIndexerC())
        )
        .onFalse(
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

        /** OJ X - L1 Position (currently stow?) */
        oj.x().onTrue(
            noop()
            // coral.CoralPositionC(CoralSubsystem.Position.SCORE)
            // .andThen(wait(0.4))
            // .andThen(
            //     elevator.ElevatorPositionC(ElevatorSubsystem.Position.L1)
            // )
            // .until(elevator::isAtTarget)
            // .andThen(coral.CoralStopC())
        );
        /** OJ A (while held) - Set deploy to UNJAM, then return to OUT on release. */
        oj.a().onTrue(
            intake.setDeployUnjamC()
        ).onFalse(
            intake.setDeployOutC()
        );
        /** OJ B - L3 Scoring Position */
        oj.b().onTrue(
            noop()
            // elevator.ElevatorPositionC(ElevatorSubsystem.Position.L3)
            // .andThen(wait(CoralConstants.kWaitDelay))
            // .andThen(coral.CoralPositionC(CoralSubsystem.Position.OUT))
        );
        /** OJ X - L4 Scoring Position */
        oj.x().onTrue(
            noop()
            // elevator.ElevatorPositionC(ElevatorSubsystem.Position.LINEUP)
            // .andThen(algae.setAlgaePositionC(AlgaeSubsystem.Position.STOW))
        );

        // ALGAE STUFF
        //Temp for testing
        // oj.povRight().onTrue(algae.startToroC(false)).onFalse(algae.stopToroC()); //intake
        // oj.povLeft().onTrue(algae.startToroC(true)).onFalse(algae.stopToroC()); //outtake
        // oj.povUp().onTrue(algae.setAlgaePositionC(AlgaeSubsystem.Position.UP)); //wrist up
        // oj.povDown().onTrue(algae.setAlgaePositionC(AlgaeSubsystem.Position.FLOOR)
        //     .andThen(algae.startToroC(false))
        // ).onFalse(algae.stopToroC()); //wrist down
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
        buildDriverTab();
        buildSystemTab();
        // buildTab("Swerve"); //placeholder
        // buildPowerTab();
        // buildDebugTab();
        gyro.buildDashboards();
    }    
    
    /** Builds and populates the autonomous chooser. */
    public void buildAutonChooser() {
        //This builds the auton chooser, giving driver friendly names to the commands from above
        if(AutonConstants.isDisabled) {
            m_auto_chooser.setDefaultOption("00: None (Auto Disabled)", Commands.none());
        } else {
            if(AutonConstants.kUseChoreo) {
                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);
                autoChooser.addRoutine("001: sLL-Move Off Line", autoRoutines::sLLmoveOffLine);
                autoChooser.addRoutine("002: sRR-Move Off Line", autoRoutines::sRRmoveOffLine);
                autoChooser.addRoutine("003: sC-Move Off Line", autoRoutines::sCmoveOffLine);
                autoChooser.addRoutine("100: sC Score Coral Get Low Algae", autoRoutines::sCScoreAlgae);
                autoChooser.addRoutine("201: sLC-Left 2.5C 1A", autoRoutines::left4Coral);
                autoChooser.addRoutine("202: sC-L4 Coral Left 2 Algae",autoRoutines::sCL4Coral2Algae);
                autoChooser.addRoutine("204: sLC-Left 3C", autoRoutines::left3Coral);
                autoChooser.addRoutine("204_SL: sLC-Left 3C", autoRoutines::left3Coral_sl);
                autoChooser.addRoutine("205: sLL Straight-Left 3C", autoRoutines::left3CoralStraight);
                autoChooser.addRoutine("206: sLL Straight-Left 3C Left", autoRoutines::left3CoralStraightLeft);
                autoChooser.addRoutine("301: sRC-Right 4C", autoRoutines::right4Coral);
                autoChooser.addRoutine("304: sRC-Right 3C", autoRoutines::right3Coral);
                autoChooser.addRoutine("305: sRR Straight-Right 3C", autoRoutines::right3CoralStraight);
                autoChooser.addRoutine("901: Left Algae Double", autoRoutines::leftAlgaeDouble);
                autoChooser.addRoutine("999: Test Run", autoRoutines::testRun);
            }
        }
    }

    /** Builds the driver Shuffleboard tab. */
    private void buildDriverTab(){
        ShuffleboardTab driverTab = buildTab("Driver");
        // Match Time - Cannot be programmatically placed, but we put it here for informative reasons
        driverTab.add("Match Time", "")
            .withPosition(0,2)
            .withSize(8,3)
            .withProperties(Map.of("time_display_mode","Minutes and Seconds","red_start_time",15,"yellow_start_time",30)) //mode: "Seconds Only" or "Minutes and Seconds"
            .withWidget("Match Time");
        // Auton Chooser
        // if(AutonConstants.isDisabled) {
        //     driverTab.add("Autonomous Chooser", m_auto_chooser)
        //         .withPosition(0, 5)
        //         .withSize(8, 2)
        //         .withProperties(Map.of("sort_options",true))
        //         .withWidget("ComboBox Chooser");
        // } else {
        //     // SmartDashboard.putData("Autonomous Chooser", autoChooser);
        //     driverTab.add("Autonomous Chooser", autoChooser)
        //         .withPosition(0, 5)
        //         .withSize(8, 2)
        //         .withProperties(Map.of("sort_options",true))
        //         .withWidget("ComboBox Chooser");
        // }
            // FMS Info - Cannot be programmatically placed, but we put it here for informative reasons
            // driverTab.add("FMS Info", "")
            //   .withPosition(0,5)
            //   .withSize(8,2)
            //   .withWidget("FMSInfo");
            // Alerts
            // driverTab.add("Alerts", "")
            //   .withPosition(8,0)
            //   .withSize(11,7)
            //   .withWidget("Alerts");
        // Camera
        // driverTab.add("Camera", Robot.camera)
        //     .withPosition(18,0)
        //     .withSize(6,5)
        //     // .withProperties(Map.of("Glyph","CAMERA_RETRO","Show Glyph",true,"Show crosshair",true,"Crosshair color","#CCCCCC","Show controls",false))
        //     .withWidget("Camera Stream");
    }

    @SuppressWarnings({"unused"})
    /** Builds the system Shuffleboard tab. */
    private void buildSystemTab(){
        ShuffleboardTab systemTab = buildTab("System");
        var chooser = (AutonConstants.isDisabled) ? m_auto_chooser : autoChooser;
        systemTab.add("Autonomous Chooser", chooser)
          .withPosition(12, 6)
          .withSize(8, 2)
          .withProperties(Map.of("sort_options",true))
          .withWidget("ComboBox Chooser");
    }

    @SuppressWarnings({"unused"})
    /** Builds the debug Shuffleboard tab. */
    private void buildDebugTab(){
        ShuffleboardTab debugTab = buildTab("Debug");
    }

    @SuppressWarnings({"unused"})
    /** Builds the power Shuffleboard tab. */
    private void buildPowerTab(){
        ShuffleboardTab powerTab = buildTab("Power");
        powerTab.add("Power", power)
            .withPosition(0, 0);
            // .withSize(1, 1);
    }

    /**
     * Returns a Shuffleboard tab by name.
     *
     * @param tabname Tab name.
     * @return Shuffleboard tab.
     */
    private ShuffleboardTab buildTab(String tabname) {
        return Shuffleboard.getTab(tabname);
    }
    //#endregion Dashboard

    //#region Convenience
    /**
     * Updates the target heading and enables target lock.
     *
     * @param target Target to face.
     */
    private void updateTarget(Targets target) {
      Rotation2d angle = targeting.getAngleOfTarget(target);
      NCDebug.Debug.debug("Update target to "+target.toString()+" ("+NCDebug.General.roundDouble(angle.getDegrees(),2)+" deg)");
      m_targetDirection = angle;
      m_targetLock = true;
    }

    /**
     * Creates a command to update the target heading.
     *
     * @param target Target to face.
     * @return Command that updates the target.
     */
    private Command updateTargetC(Targets target) {
      return new InstantCommand(() -> updateTarget(target)); 
    }    
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
