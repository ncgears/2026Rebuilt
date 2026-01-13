package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.classes.Lighting;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.NCDebug;

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
     * Creates the center move-off-line routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sCmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sCMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sC-MoveOffLine");

      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates a center score-algae routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sCScoreAlgae() {
      final AutoRoutine routine = m_factory.newRoutine("sCScoreAlgae");
      final AutoTrajectory path1 = routine.trajectory("sC-rBC");

      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates the left move-off-line routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sLLmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sLLMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sLL-MoveOffLine");
    
      path1.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates the right move-off-line routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sRRmoveOffLine() {
      final AutoRoutine routine = m_factory.newRoutine("sRRMoveOffLine");
      final AutoTrajectory path1 = routine.trajectory("sRR-MoveOffLine");
    
      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
            .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates the left algae double routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine leftAlgaeDouble() {
      final AutoRoutine routine = m_factory.newRoutine("LeftAlgaeDouble");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_c");
      final AutoTrajectory path2 = routine.trajectory("rBL_c-bC");
      final AutoTrajectory path3 = routine.trajectory("bC-aL-bC");
    
      path1.done().onTrue(runPath(path2));
      path2.done().onTrue(runPath(path3));
      path3.done().onTrue(log("Routine Complete!"));

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingAlgae())
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
        ScoreCoral()
        .andThen(wait(0.2))
        .andThen(runPath(path2))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the left four-coral routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine left4Coral() { //201
      final AutoRoutine routine = m_factory.newRoutine("Left4Coral");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_r");
      final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_c");
      // final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      // final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingAlgae())
        .andThen(runPath(path5))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the center L4 coral then two algae routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine sCL4Coral2Algae() { //202
      final AutoRoutine routine = m_factory.newRoutine("sCL4Coral2Algae");
      final AutoTrajectory path1 = routine.trajectory("sCb-rBC_r");
      final AutoTrajectory path2 = routine.trajectory("rBC_r-rBC_c");
      final AutoTrajectory path3 = routine.trajectory("rBC_c-bC");
      final AutoTrajectory path4 = routine.trajectory("bC-rBL_c");
      // final AutoTrajectory path5 = routine.trajectory("rBL_c-waitLeft");
      final AutoTrajectory path5 = routine.trajectory("rBL_c-bC");
      // final AutoTrajectory path6 = routine.trajectory("bC-hL");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(wait(0.2))
        .andThen(SeekingAlgae())
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP))
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreAlgae()
        .andThen(wait(0.3))
        .andThen(SeekingAlgae())
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingNone())
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP))
        // .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreAlgae()
        .andThen(wait(0.3))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );
      return routine;
    }

    /**
     * Creates the left three-coral routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine left3Coral() { //204
      final AutoRoutine routine = m_factory.newRoutine("Left3Coral");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_r");
      final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      // final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the left three-coral routine with slow segments.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine left3Coral_sl() { //204_SL
      final AutoRoutine routine = m_factory.newRoutine("Left3Coral");
      final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_r");
      final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l_sl");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_r_sl");
      final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      // final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the left three-coral straight routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine left3CoralStraight() { //205
      final AutoRoutine routine = m_factory.newRoutine("Left3CoralStraight");
      final AutoTrajectory path1 = routine.trajectory("sLLb-rBL_r_str");
      final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      // final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the left three-coral straight-left routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine left3CoralStraightLeft() { //206
      final AutoRoutine routine = m_factory.newRoutine("Left3CoralStraightLeft");
      final AutoTrajectory path1 = routine.trajectory("sLLb-rBL_l_str");
      final AutoTrajectory path2 = routine.trajectory("rBL_l-hL");
      final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
      // final AutoTrajectory path7 = routine.trajectory("hL-rFC_l");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the right four-coral routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine right4Coral() { //301
      final AutoRoutine routine = m_factory.newRoutine("Right4Coral");
      final AutoTrajectory path1 = routine.trajectory("sRCb-rBR_l"); //sLCb-rBL_r
      final AutoTrajectory path2 = routine.trajectory("rBR_l-hR"); //rBL_r-hL
      final AutoTrajectory path3 = routine.trajectory("hR-rFR_r"); //hL-rFL_l
      final AutoTrajectory path4 = routine.trajectory("rFR_r-hR"); //rFL_l-hL
      final AutoTrajectory path5 = routine.trajectory("hR-rFR_l"); //hL-rFL_r
      final AutoTrajectory path6 = routine.trajectory("rFR_l-hR"); //rFL_r-hL
      // final AutoTrajectory path7 = routine.trajectory("hR-rFC_r"); //hL-rFC_l

      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(wait(0.2))
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(wait(0.2))
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(wait(0.2))
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the right three-coral routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine right3Coral() { //304
      final AutoRoutine routine = m_factory.newRoutine("Right3Coral");
      final AutoTrajectory path1 = routine.trajectory("sRCb-rBR_l");
      final AutoTrajectory path2 = routine.trajectory("rBR_l-hR");
      final AutoTrajectory path3 = routine.trajectory("hR-rFR_r");
      final AutoTrajectory path4 = routine.trajectory("rFR_r-hR");
      final AutoTrajectory path5 = routine.trajectory("hR-rFR_l");
      final AutoTrajectory path6 = routine.trajectory("rFR_l-hR");

      // final AutoTrajectory path1 = routine.trajectory("sLCb-rBL_r");
      // final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      // final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      // final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      // final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      // final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");
    
      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    /**
     * Creates the right three-coral straight routine.
     *
     * @return Auto routine instance.
     */
    public AutoRoutine right3CoralStraight() { //305
      final AutoRoutine routine = m_factory.newRoutine("Right3CoralStraight");
      final AutoTrajectory path1 = routine.trajectory("sRRb-rBR_l_str");
      final AutoTrajectory path2 = routine.trajectory("rBR_l-hR");
      final AutoTrajectory path3 = routine.trajectory("hR-rFR_r");
      final AutoTrajectory path4 = routine.trajectory("rFR_r-hR");
      final AutoTrajectory path5 = routine.trajectory("hR-rFR_l");
      final AutoTrajectory path6 = routine.trajectory("rFR_l-hR");

      // final AutoTrajectory path1 = routine.trajectory("sLLb-rBL_r_str");
      // final AutoTrajectory path2 = routine.trajectory("rBL_r-hL");
      // final AutoTrajectory path3 = routine.trajectory("hL-rFL_l");
      // final AutoTrajectory path4 = routine.trajectory("rFL_l-hL");
      // final AutoTrajectory path5 = routine.trajectory("hL-rFL_r");
      // final AutoTrajectory path6 = routine.trajectory("rFL_r-hL");

      path1.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path2))
      );
      path2.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path3))
      );
      path3.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path4))
      );
      path4.recentlyDone().onTrue(
        wait(0.7)
        .andThen(SeekingCoral())
        .andThen(runPath(path5))
      );
      path5.recentlyDone().onTrue(
        ScoreCoral()
        .andThen(runPath(path6))
      );

      seedPose(path1);
      routine.active().onTrue(
          path1.resetOdometry()
          .andThen(SeekingCoral())
          .andThen(runPath(path1))
      );

      return routine;
    }

    //#region Global Bindings
    /** This method binds event names to their commands */
    private void ConfigureGlobalBindings() {
      m_factory
        .bind("readyL2",log("EVENT(readyL4)").andThen(ReadyL2()))
        .bind("readyL3",log("EVENT(readyL4)").andThen(ReadyL3()))
        .bind("readyL4",log("EVENT(readyL4)").andThen(ReadyL4()))
        .bind("transit",log("EVENT(transit)").andThen(Transit()))
        .bind("intakeCoral",log("EVENT(intakeCoral)").andThen(IntakeCoral()))
        .bind("scoreCoral",log("EVENT(scoreCoral)").andThen(ScoreCoral()))
        .bind("readyBarge",log("EVENT(readyBarge)").andThen(ReadyBarge()))
        .bind("intakeLow",log("EVENT(intakeLow)").andThen(IntakeAlgaeLow()))
        .bind("intakeHigh",log("EVENT(intakeHigh)").andThen(IntakeAlgaeHigh()))
        .bind("intakeSpike",log("EVENT(intakeSpike)").andThen(IntakeAlgaeSpike()))
        .bind("scoreAlgae",log("EVENT(scoreAlgae)").andThen(ScoreAlgae()));
    }
    //#endregion Global Bindings

    //#region AutoCommands
    /**
     * Creates a command to move to L2 and prep coral.
     *
     * @return Command sequence.
     */
    private Command ReadyL2() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L2)
        // .until(RobotContainer.elevator.atTarget)
      .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.OUT));
  }
    /**
     * Creates a command to move to L3 and prep coral.
     *
     * @return Command sequence.
     */
    private Command ReadyL3() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L3)
        // .until(RobotContainer.elevator.atTarget)
      .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.OUT));
  }
    /**
     * Creates a command to move to L4 and prep coral.
     *
     * @return Command sequence.
     */
    private Command ReadyL4() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L4)
        // .until(RobotContainer.elevator.atTarget)
      .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.OUT));
    }
    /**
     * Creates a command sequence to score coral.
     *
     * @return Command sequence.
     */
    private Command ScoreCoral() {
      return 
        // Commands.waitUntil(RobotContainer.elevator.atTarget)
        wait(0.1)
        .andThen(RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.L4SCORE))
          // .until(RobotContainer.elevator.atTarget)
        .andThen(wait(0.4))
        .andThen(SeekingNone())
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.SCORE));
    }
    /**
     * Creates a command sequence to intake coral.
     *
     * @return Command sequence.
     */
    private Command IntakeCoral() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.HP)
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.OUT))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    /**
     * Creates a command to move to the barge scoring position.
     *
     * @return Command sequence.
     */
    private Command ReadyBarge() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.BARGE)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    /**
     * Creates a command sequence to intake low algae.
     *
     * @return Command sequence.
     */
    private Command IntakeAlgaeLow() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAELOW)
          // .until(RobotContainer.elevator.atTarget)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
        .andThen(RobotContainer.algae.startToroC(false));
        // .andThen(wait(0.5))
        // .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    /**
     * Creates a command sequence to intake high algae.
     *
     * @return Command sequence.
     */
    private Command IntakeAlgaeHigh() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.ALGAEHIGH)
          // .until(RobotContainer.elevator.atTarget)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF))
        .andThen(RobotContainer.algae.startToroC(false));
    }
    /**
     * Creates a command sequence to intake algae and return up.
     *
     * @return Command sequence.
     */
    private Command IntakeAlgae() {
      return RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.REEF)
      .andThen(wait(1.0))
      .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    /**
     * Creates a command sequence to intake algae from the spike.
     *
     * @return Command sequence.
     */
    private Command IntakeAlgaeSpike() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.FLOOR)
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.FLOOR))
        .andThen(RobotContainer.algae.startToroC(false));
    }
    /**
     * Creates a command sequence to score algae.
     *
     * @return Command sequence.
     */
    private Command ScoreAlgae() {
      return RobotContainer.algae.startToroC(true)
      .andThen(wait(0.2))
      .andThen(SeekingNone())
      .andThen(RobotContainer.algae.stopToroC());
    }
    /**
     * Creates a command sequence for transit pose.
     *
     * @return Command sequence.
     */
    private Command Transit() {
      return RobotContainer.elevator.ElevatorPositionC(ElevatorSubsystem.Position.HP)
        .andThen(RobotContainer.coral.CoralPositionC(CoralSubsystem.Position.SCORE))
        .andThen(RobotContainer.algae.setAlgaePositionC(AlgaeSubsystem.Position.UP));
    }
    /**
     * Sets lighting to indicate coral-seeking.
     *
     * @return Command to set lighting color.
     */
    private Command SeekingCoral() {
      return RobotContainer.lighting.setColorCommand(Lighting.Colors.WHITE);
    }
    /**
     * Sets lighting to indicate algae-seeking.
     *
     * @return Command to set lighting color.
     */
    private Command SeekingAlgae() {
      return RobotContainer.lighting.setColorCommand(Lighting.Colors.TEAL);
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
