
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.utils.Helpers;
import frc.robot.utils.NCDebug;

/**
 * This subsystem handles managing the Indexer subsystem.
 * It is responsible for doing some stuff.
 */
public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;
  // #region Declarations
  // Declare public and private variables

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
  private final VelocityVoltage m_indexerVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_knuckleVelocityRequest = new VelocityVoltage(0.0);
  // private CANcoder m_encoder;
  private TalonFX m_indexerMotor, m_knuckleMotor;
  private State m_curIndexerState = State.STOP;
  private State m_curKnuckleState = State.STOP;
  private double m_indexerCommandedSpeedRpm = 0.0;
  private double m_knuckleCommandedSpeedRpm = 0.0;
  // #endregion Declarations

  // #region Triggers
  // Trigger definitions

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
    m_indexerMotor = new TalonFX(IndexerConstants.Indexer.kMotorID, IndexerConstants.canBus);
    m_knuckleMotor = new TalonFX(IndexerConstants.Knuckle.kMotorID, IndexerConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_indexerMotor.getConfigurator().apply(RobotContainer.ctreConfigs.indexerFXConfig));
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_knuckleMotor.getConfigurator().apply(RobotContainer.ctreConfigs.knuckleFXConfig));

    init();
    createDashboards();
  }

  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    m_curIndexerState = State.STOP;
    m_curKnuckleState = State.STOP;
    m_indexerCommandedSpeedRpm = 0.0;
    m_knuckleCommandedSpeedRpm = 0.0;
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
    driverTab.addString("Climber", this::getIndexStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(6, 7);

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout indexerList = systemTab.getLayout("Indexer", BuiltInLayouts.kList)
      .withSize(4, 6)
      .withPosition(16, 0)
      .withProperties(Map.of("Label position", "LEFT"));
    indexerList.addString("Status", this::getIndexStateColor)
      .withWidget("Single Color View");
    indexerList.addString("State", this::getIndexStateName);

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
   * @return Command that sets indexer and knuckle motors to coast neutral mode.
   */
  public Command neutralCommand() {
    return indexerNeutralC();
  }

  /**
   * Creates a command to set both indexer motors to the same speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for indexer and knuckle motors.
   * @return Command that updates both motor speed setpoints.
   */
  public Command setIndexerSpeedC(double rpm) {
    return runOnce(() -> setIndexerSpeedRPM(rpm));
  }

  /**
   * Creates a command to set independent speed setpoints in RPM.
   *
   * @param indexerRpm Target speed in RPM for the indexer motor.
   * @param knuckleRpm Target speed in RPM for the knuckle motor.
   * @return Command that updates indexer and knuckle speed setpoints.
   */
  public Command setIndexerSpeedC(double indexerRpm, double knuckleRpm) {
    return runOnce(() -> setIndexerSpeedRPM(indexerRpm, knuckleRpm));
  }

  /**
   * Creates a command to set the knuckle motor speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for the knuckle motor.
   * @return Command that updates the knuckle motor speed setpoint.
   */
  public Command setKnuckleSpeedC(double rpm) {
    return runOnce(() -> setKnuckleSpeedRPM(rpm));
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
  public State getIndexState() {
    return m_curIndexerState;
  }

  /**
   * Returns the current indexer state name.
   *
   * @return State name.
   */
  public String getIndexStateName() {
    return m_curIndexerState.toString();
  }

  /**
   * Returns the current state color.
   *
   * @return Hex color string.
   */
  public String getIndexStateColor() {
    return m_curIndexerState.getColor();
  }

  /**
   * Returns the current knuckle state.
   *
   * @return Current knuckle state.
   */
  public State getKnuckleState() {
    return m_curKnuckleState;
  }

  /**
   * Returns the current knuckle state name.
   *
   * @return Knuckle state name.
   */
  public String getKnuckleStateName() {
    return m_curKnuckleState.toString();
  }

  /**
   * Returns the current knuckle state color.
   *
   * @return Knuckle state color as a hex string.
   */
  public String getKnuckleStateColor() {
    return m_curKnuckleState.getColor();
  }

  /**
   * Returns the commanded indexer speed setpoint in RPM.
   *
   * @return Commanded indexer speed in RPM.
   */
  public double getIndexerCommandedSpeedRPM() {
    return m_indexerCommandedSpeedRpm;
  }

  /**
   * Returns the commanded knuckle speed setpoint in RPM.
   *
   * @return Commanded knuckle speed in RPM.
   */
  public double getKnuckleCommandedSpeedRPM() {
    return m_knuckleCommandedSpeedRpm;
  }

  /**
   * Returns the current measured indexer speed in RPM.
   *
   * @return Current indexer speed in RPM.
   */
  public double getIndexerCurrentSpeedRPM() {
    return m_indexerMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Returns the current measured knuckle speed in RPM.
   *
   * @return Current knuckle speed in RPM.
   */
  public double getKnuckleCurrentSpeedRPM() {
    return m_knuckleMotor.getVelocity().getValueAsDouble() * 60.0;
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

  /**
   * Sets indexer and knuckle motors to coast neutral mode.
   */
  public void indexerNeutral() {
    m_indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_knuckleMotor.setNeutralMode(NeutralModeValue.Coast);
    m_curIndexerState = State.STOP;
    m_curKnuckleState = State.STOP;
    m_indexerCommandedSpeedRpm = 0.0;
    m_knuckleCommandedSpeedRpm = 0.0;
    NCDebug.Debug.debug("Indexer: Switch to Coast");
  }

  /**
   * Sets both indexer motors to the same speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for indexer and knuckle motors.
   */
  public void setIndexerSpeedRPM(double rpm) {
    setIndexerSpeedRPM(rpm, rpm);
  }

  /**
   * Sets independent speed setpoints in RPM.
   *
   * @param indexerRpm Target speed in RPM for the indexer motor.
   * @param knuckleRpm Target speed in RPM for the knuckle motor.
   */
  public void setIndexerSpeedRPM(double indexerRpm, double knuckleRpm) {
    double indexerRps = Helpers.RPMtoRPS(indexerRpm);
    double knuckleRps = Helpers.RPMtoRPS(knuckleRpm);
    m_indexerMotor.setControl(m_indexerVelocityRequest.withVelocity(indexerRps));
    m_knuckleMotor.setControl(m_knuckleVelocityRequest.withVelocity(knuckleRps));
    m_indexerCommandedSpeedRpm = indexerRpm;
    m_knuckleCommandedSpeedRpm = knuckleRpm;
    m_curIndexerState = stateFromRPM(indexerRpm);
    m_curKnuckleState = stateFromRPM(knuckleRpm);
    NCDebug.Debug.debug("Indexer: Set speed " + indexerRpm + "RPM indexer, " + knuckleRpm + "RPM knuckle");
  }

  /**
   * Sets the knuckle motor speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for the knuckle motor.
   */
  public void setKnuckleSpeedRPM(double rpm) {
    double rps = Helpers.RPMtoRPS(rpm);
    m_knuckleMotor.setControl(m_knuckleVelocityRequest.withVelocity(rps));
    m_knuckleCommandedSpeedRpm = rpm;
    m_curKnuckleState = stateFromRPM(rpm);
    NCDebug.Debug.debug("Indexer: Set speed " + rpm + "RPM knuckle");
  }

  /**
   * Converts a commanded RPM value into a directional state.
   *
   * @param rpm Commanded motor speed in RPM.
   * @return {@link State#FWD} for positive RPM, {@link State#REV} for negative RPM,
   *         otherwise {@link State#STOP}.
   */
  private State stateFromRPM(double rpm) {
    if (rpm > 0.0) {
      return State.FWD;
    } else if (rpm < 0.0) {
      return State.REV;
    }
    return State.STOP;
  }

  /**
   * Creates a command to set indexer and knuckle motors to coast neutral mode.
   *
   * @return Command that updates both indexer motor neutral modes.
   */
  public Command indexerNeutralC() {
    return runOnce(this::indexerNeutral);
  }
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
