package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.classes.Lighting;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.NCDebug;

/**
 * Builds and wires all autonomous routines and helper command fragments.
 */
public class AutoRoutines {
    private final AutoFactory m_factory;

    /**
     * Creates the auto routines helper and binds global events.
     *
     * @param factory AutoFactory used to create routines.
     */
    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        ConfigureGlobalBindings();
    }

    /**
     * Creates the truss left direct to fuel intake and score 1 pass.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sTLDirect1Pass() { //101
      final AutoRoutine routine = m_factory.newRoutine("sTLDirect1Pass");
      final AutoTrajectory path1 = routine.trajectory("LeftFirstPass");
    
      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(DeployIntake())
          .andThen(Commands.idle().until(RobotContainer.intake::getDeployed))
          .andThen(runPath(path1))
          .andThen(StartShooter())
          .andThen(Commands.idle().withTimeout(4.25))
          .andThen(StopShooter())
          .andThen(StopIntake())
      );
      return routine;
    }

    /**
     * Creates the truss left direct to fuel intake and score 1 pass.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sTLDirect2Pass() { //102
      final AutoRoutine routine = m_factory.newRoutine("sTLDirect2Pass");
      final AutoTrajectory path1 = routine.trajectory("LeftFirstPass");
      final AutoTrajectory path2 = routine.trajectory("LeftSecondPass");
    
      path1.recentlyDone().onTrue(
        noop()
        .andThen(StartShooter())
        .andThen(Commands.idle().withTimeout(4.25))
        .andThen(StopShooter())
        .andThen(runPath(path2))
      );

      path2.done().onTrue(
        noop()
        .andThen(StartShooter())
        .andThen(Commands.idle().withTimeout(4.25))
        .andThen(StopShooter())
        .andThen(StopIntake())
        .andThen(log("Routine Complete!"))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(DeployIntake())
          .andThen(Commands.idle().until(RobotContainer.intake::getDeployed))
          .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates the truss right direct to fuel intake and score 1 pass.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sTRDirect1Pass() {  //202
      final AutoRoutine routine = m_factory.newRoutine("sTRDirect1Pass");
      final AutoTrajectory path1 = routine.trajectory("RightFirstPass");
    
      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(DeployIntake())
          .andThen(Commands.idle().until(RobotContainer.intake::getDeployed))
          .andThen(runPath(path1))
          .andThen(StartShooter())
          .andThen(Commands.idle().withTimeout(4.25))
          .andThen(StopShooter())
          .andThen(StopIntake())
      );
      return routine;
    }

        /**
     * Creates the truss right direct to fuel intake and score 2 pass.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sTRDirect2Pass() { //102
      final AutoRoutine routine = m_factory.newRoutine("sTRDirect2Pass");
      final AutoTrajectory path1 = routine.trajectory("RightFirstPass");
      final AutoTrajectory path2 = routine.trajectory("RightSecondPass");
    
      path1.recentlyDone().onTrue(
        noop()
        .andThen(StartShooter())
        .andThen(Commands.idle().withTimeout(4.25))
        .andThen(StopShooter())
        .andThen(runPath(path2))
      );

      path2.done().onTrue(
        noop()
        .andThen(StartShooter())
        .andThen(Commands.idle().withTimeout(4.25))
        .andThen(StopShooter())
        .andThen(StopIntake())
        .andThen(log("Routine Complete!"))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(DeployIntake())
          .andThen(Commands.idle().until(RobotContainer.intake::getDeployed))
          .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates a test routine for quick path validation.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine testRun() { //999
      final AutoRoutine routine = m_factory.newRoutine("TestRun");
      final AutoTrajectory path1 = routine.trajectory("T1");
      final AutoTrajectory path2 = routine.trajectory("T2");

      path1.recentlyDone().onTrue(
        noop()
        .andThen(runPath(path2))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(DeployIntake())
          .andThen(Commands.idle().until(RobotContainer.intake::getDeployed))
          .andThen(runPath(path1))
      );

      return routine;
    }

    
    //#region Global Bindings
    /** This method binds event names to their commands */
    private void ConfigureGlobalBindings() {
      m_factory
        .bind("DeployIntake",log("EVENT(DeployIntake)").andThen(DeployIntake()))
        .bind("StartShooter",log("EVENT(StartShooter)").andThen(StartShooter()))
        .bind("StopShooter",log("EVENT(StopShooter)").andThen(StopShooter()))
        .bind("StartIntake",log("EVENT(StartIntake)").andThen(StartIntake()))
        .bind("StopIntake",log("EVENT(StopIntake)").andThen(StopIntake()))
        .bind("SeekFuel",log("EVENT(SeekFule)").andThen(SeekingFuel()))
        .bind("SeekTarget",log("EVENT(SeekTarget)").andThen(SeekingTarget()))
        .bind("SeekNone",log("EVENT(SeekNone)").andThen(SeekingNone()))
        ;
    }
    //#endregion Global Bindings

    //#region AutoCommands
    /**
     * Creates a command to deploy the intake.
     *
     * @return Command sequence.
     */
    private Command DeployIntake() {
      return RobotContainer.intake.setDeployOutC();
    }

    /**
     * Creates a command to start the shooter.
     *
     * @return Command sequence.
     */
    private Command StartShooter() {
      return Commands.sequence(
                RobotContainer.targeting.setTrackingHubC(),
                Commands.deadline(
                    wait(ShooterConstants.kSpinupDelaySeconds),
                    Commands.run(
                        () -> RobotContainer.shooter.startShooter(RobotContainer.targeting.getDistanceOfTarget(RobotContainer.targeting.getTrackingTarget())),
                        RobotContainer.shooter
                    )
                ),
                Commands.run(
                    () -> {
                        RobotContainer.shooter.startShooter(RobotContainer.targeting.getDistanceOfTarget(RobotContainer.targeting.getTrackingTarget()));
                        RobotContainer.indexer.startIndexer();
                    },
                    RobotContainer.shooter,
                    RobotContainer.indexer
                )
            );
      }

    /**
     * Creates a command to stop the shooter.
     *
     * @return Command sequence.
     */
    private Command StopShooter() {
      return Commands.parallel(
                RobotContainer.shooter.neutralCommand(),
                RobotContainer.indexer.neutralCommand()
            );
    }

        /**
     * Creates a command to start the shooter.
     *
     * @return Command sequence.
     */
    private Command StartIntake() {
      return RobotContainer.intake.setIntakeForwardAdaptiveC(RobotContainer.shooter::isShooting);
    }

    /**
     * Creates a command to stop the shooter.
     *
     * @return Command sequence.
     */
    private Command StopIntake() {
      return RobotContainer.intake.setIntakeStopC();
    }
    
    /**
     * Sets lighting to indicate fuel-seeking.
     *
     * @return Command to set lighting color.
     */
    private Command SeekingFuel() {
      return RobotContainer.lighting.setColorCommand(Lighting.Colors.YELLOW);
    }
    /**
     * Sets lighting to indicate targeting.
     *
     * @return Command to set lighting color.
     */
    private Command SeekingTarget() {
      return RobotContainer.lighting.setColorCommand(Lighting.Colors.GREEN);
    }
    /**
     * Clears lighting indication.
     *
     * @return Command to set lighting off.
     */
    private Command SeekingNone() {
      return RobotContainer.lighting.setColorCommand(Lighting.Colors.OFF);
    }
    //#endregion AutoCommands

    //#region Convenience
    /**
     * Seeds the drivetrain pose from the given path.
     *
     * @param path Auto trajectory to use for pose seeding.
     */
    private void seedPose(AutoTrajectory path) {
      try {
        RobotContainer.targeting.resetPose(path.getInitialPose().get());
      } catch(Exception e) {
        log("EXCEPTION! Bad Path Name?");
      };
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
    //This command runs the specified path
    /**
     * Logs and spawns a trajectory command.
     *
     * @param path Trajectory to run.
     * @return Command to execute the path.
     */
    private Command runPath(AutoTrajectory path) {
      return
        log("PATH("+path.getRawTrajectory().name()+")")
        .andThen(path.spawnCmd());

    }
    /**
     * Logs a message with the auton prefix.
     *
     * @param msg Message to log.
     * @return Command that logs the message.
     */
    private Command log(String msg) {
      return NCDebug.Debug.debugC("Auton: "+msg);
    }
    //#endregion
}
