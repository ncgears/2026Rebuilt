
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.utils.NCDebug;

/**
 * This subsystem handles managing the Indexer subsystem.
 * It is responsible for doing some stuff.
 */
public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;
  // #region Declarations
  // Declare public and private variables
  private DigitalInput m_beambreak = new DigitalInput(IndexerConstants.kBeambreakID);

  public enum State {
    REV(DashboardConstants.Colors.RED),
    FWD(DashboardConstants.Colors.GREEN),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    State(String color) {
      this.color = color;
    }
    /**
     * Returns the dashboard color for this state.
     *
     * @return Hex color string.
     */
    public String getColor() {
      return this.color;
    }
  }

  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();
  // private CANcoder m_encoder;
  private TalonFX m_feedmotor1, m_beltmotor1;
  private State m_curState = State.STOP;
  // #endregion Declarations

  // #region Triggers
  // Trigger definitions

  /**
   * Returns true when the indexer feed is full
   */
  public final Trigger isFull = new Trigger(this::getIsFull);
  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the IndexerSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return IndexerSubsystem instance
   */
  public static IndexerSubsystem getInstance() {
    if (instance == null)
      instance = new IndexerSubsystem();
    return instance;
  }

  /** Creates the Indexer subsystem and initializes state. */
  public IndexerSubsystem() {
    // initialize values for private and public variables, etc.
    m_beltmotor1 = new TalonFX(IndexerConstants.Belt.kMotorID, IndexerConstants.canBus);
    m_feedmotor1 = new TalonFX(IndexerConstants.Feed.kMotorID, IndexerConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_feedmotor1.getConfigurator().apply(RobotContainer.ctreConfigs.indexerFeedFXConfig));
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_beltmotor1.getConfigurator().apply(RobotContainer.ctreConfigs.indexerBeltFXConfig));

    init();
    createDashboards();
  }

  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    m_curState = State.STOP;
    NCDebug.Debug.debug("Indexer: Initialized");
  }

  /** Runs periodically for the Indexer subsystem. */
  @Override
  public void periodic() {
  }
  // #endregion Setup

  // #region Commands
  /** Creates Shuffleboard widgets for the climber. */
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Climber", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(6, 7);

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout indexerList = systemTab.getLayout("Indexer", BuiltInLayouts.kList)
      .withSize(4, 6)
      .withPosition(16, 0)
      .withProperties(Map.of("Label position", "LEFT"));
    indexerList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    indexerList.addBoolean("Is Full", this::getIsFull);
    indexerList.addString("State", this::getStateName);

    if (IndexerConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgindexerList = debugTab.getLayout("Indexer", BuiltInLayouts.kList)
        .withSize(4, 11)
        .withPosition(4, 0)
        .withProperties(Map.of("Label position", "LEFT"));
    }
  }

  /**
   * neutralCommand is used to reset this system into a safe state when disabled. 
   * It is called when the robot is disabled to reset counters, states, etc.
   *
   * @return command that does nothing when scheduled
   */
  public Command neutralCommand() {
    m_curState = State.STOP;
    return Commands.none();
  }

  // #region Dashboard
  // Methods for creating and updating dashboards
  // #endregion Dashboard

  // #region Getters
  // Methods for getting data for subsystem

  /**
   * Returns the current indexer state.
   *
   * @return Current state.
   */
  public State getState() {
    return m_curState;
  }

  /**
   * Returns the current indexer state name.
   *
   * @return State name.
   */
  public String getStateName() {
    return m_curState.toString();
  }

  /**
   * Returns the current state color.
   *
   * @return Hex color string.
   */
  public String getStateColor() {
    return m_curState.getColor();
  }

  /**
   * Returns true if the indexer feed path has a detected fuel.
   *
   * @return True when a the indexer feed path has a detected fuel.
   */
  public boolean getIsFull() {
    return getIndexerBeambreak();
  }

  /**
   * Returns the state of the beam break.
   *
   * @return True when the switch is triggered.
   */
  private boolean getIndexerBeambreak() {
    return !m_beambreak.get();
  }
  // #endregion Getters

  // #region Setters
  // Methods for setting data for subsystem
  // #endregion Setters

  // #region Limits
  // Methods for detecting limit conditions
  // #endregion Limits

  // #region Controls
  // Methods for controlling the subsystem
  // #endregion Controls

  // #region SysID Functions
  // Routines for characterization
  // private final VoltageOut m_voltReq = new VoltageOut(0.0);
  // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
  // new SysIdRoutine.Config(
  // null, //default ramp rate 1V/s
  // Volts.of(4), //reduce dynamic step voltage to 4 to prevent brownout
  // null, //default timeout 10s
  // (state) -> SignalLogger.writeString("SysId_State", state.toString())
  // ),
  // new SysIdRoutine.Mechanism(
  // (volts) -> m_motor1.setControl(m_voltReq.withOutput(volts.in(Volts))),
  // null,
  // this
  // )
  // );
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return m_sysIdRoutine.quasistatic(direction);
  // }
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return m_sysIdRoutine.dynamic(direction);
  // }
  // public Command runSysIdCommand() {
  // return Commands.sequence(
  // sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(this::atLimit),
  // sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(this::atLimit),
  // sysIdDynamic(SysIdRoutine.Direction.kForward).until(this::atLimit),
  // sysIdDynamic(SysIdRoutine.Direction.kReverse).until(this::atLimit)
  // );
  // }
  // #endregion SysID Functions
}
