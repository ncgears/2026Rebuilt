
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
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
  private TalonFX m_indexerMotor, m_knuckleMotor, m_liveBottomMotor;
  private Servo m_matrixBreakerServo, m_shoeServo;
  private State m_curIndexerState = State.STOP;
  private State m_curKnuckleState = State.STOP;
  private State m_curLiveBottomState = State.STOP;
  private State m_curMatrixBreakerState = State.STOP;
  private State m_curShoeState = State.STOP;
  private double m_indexerCommandedSpeedRpm = 0.0;
  private double m_knuckleCommandedSpeedRpm = 0.0;
  private double m_liveBottomCommandedPower = 0.0;
  private double m_matrixBreakerCommandedOutput = IndexerConstants.MatrixBreaker.kStop;
  private double m_shoeCommandedOutput = IndexerConstants.Shoe.kStop;
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
    m_liveBottomMotor = new TalonFX(IndexerConstants.LiveBottom.kMotorID, IndexerConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_indexerMotor.getConfigurator().apply(RobotContainer.ctreConfigs.indexerFXConfig));
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_knuckleMotor.getConfigurator().apply(RobotContainer.ctreConfigs.knuckleFXConfig));
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_liveBottomMotor.getConfigurator().apply(RobotContainer.ctreConfigs.liveBottomFXConfig));
    m_matrixBreakerServo = new Servo(IndexerConstants.MatrixBreaker.kServoID);
    m_shoeServo = new Servo(IndexerConstants.Shoe.kServoID);

    init();
  }

  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    m_curIndexerState = State.STOP;
    m_curKnuckleState = State.STOP;
    m_curLiveBottomState = State.STOP;
    m_curMatrixBreakerState = State.STOP;
    m_curShoeState = State.STOP;
    m_indexerCommandedSpeedRpm = 0.0;
    m_knuckleCommandedSpeedRpm = 0.0;
    m_liveBottomCommandedPower = 0.0;
    m_matrixBreakerCommandedOutput = IndexerConstants.MatrixBreaker.kStop;
    m_shoeCommandedOutput = IndexerConstants.Shoe.kStop;
    m_liveBottomMotor.setControl(m_DutyCycle.withOutput(0.0));
    m_matrixBreakerServo.set(IndexerConstants.MatrixBreaker.kStop);
    m_shoeServo.set(IndexerConstants.Shoe.kStop);
    NCDebug.Debug.debug("Indexer: Initialized");
  }

  /** Runs periodically for the Indexer subsystem. */
  @Override
  public void periodic() {
    updateDashboards();
  }
  // #endregion Setup

  // #region Commands
  /**
   * neutralCommand is used to reset this system into a safe state when disabled. 
   * It is called when the robot is disabled to reset counters, states, etc.
   *
   * @return Command that neutralizes indexer, knuckle, live bottom, matrix breaker, and shoe.
   */
  public Command neutralCommand() {
    return runOnce(this::indexerSystemNeutral);
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

  /**
   * Creates a command to set the live bottom motor output power.
   *
   * @param power Percent output from -1.0 to 1.0.
   * @return Command that updates live bottom motor output.
   */
  public Command setLiveBottomPowerC(double power) {
    return runOnce(() -> setLiveBottomPower(power));
  }

  /**
   * Creates a command to run live bottom forward using configured power.
   *
   * @return Command that sets live bottom forward output.
   */
  public Command liveBottomForwardC() {
    return runOnce(this::liveBottomForward);
  }

  /**
   * Creates a command to run live bottom in reverse using configured power.
   *
   * @return Command that sets live bottom reverse output.
   */
  public Command liveBottomReverseC() {
    return runOnce(this::liveBottomReverse);
  }

  /**
   * Creates a command to stop live bottom output.
   *
   * @return Command that stops live bottom output.
   */
  public Command liveBottomStopC() {
    return runOnce(this::liveBottomStop);
  }

  /**
   * Creates a command to run the matrix breaker servo forward.
   *
   * @return Command that runs matrix breaker forward.
   */
  public Command matrixBreakerForwardC() {
    return runOnce(this::matrixBreakerForward);
  }

  /**
   * Creates a command to run the matrix breaker servo in reverse.
   *
   * @return Command that runs matrix breaker in reverse.
   */
  public Command matrixBreakerReverseC() {
    return runOnce(this::matrixBreakerReverse);
  }

  /**
   * Creates a command to stop the matrix breaker servo.
   *
   * @return Command that stops matrix breaker output.
   */
  public Command matrixBreakerStopC() {
    return runOnce(this::matrixBreakerStop);
  }

  /**
   * Creates a command to run the shoe servo forward.
   *
   * @return Command that runs shoe forward.
   */
  public Command shoeForwardC() {
    return runOnce(this::shoeForward);
  }

  /**
   * Creates a command to run the shoe servo in reverse.
   *
   * @return Command that runs shoe in reverse.
   */
  public Command shoeReverseC() {
    return runOnce(this::shoeReverse);
  }

  /**
   * Creates a command to stop the shoe servo.
   *
   * @return Command that stops shoe output.
   */
  public Command shoeStopC() {
    return runOnce(this::shoeStop);
  }

  /**
   * Creates a command to start indexer feed mechanisms using derived RPM setpoints
   * from the configured default back shooter RPM.
   *
   * @return Command that starts indexer, knuckle, live bottom, shoe, and matrix breaker.
   */
  public Command startIndexerC() {
    return runOnce(this::startIndexer);
  }

  /**
   * Creates a command to start indexer feed mechanisms using RPM setpoints
   * derived from a supplied back shooter RPM.
   *
   * @param backRpm Back shooter RPM (master).
   * @return Command that starts indexer, knuckle, live bottom, shoe, and matrix breaker.
   */
  public Command startIndexerC(double backRpm) {
    return runOnce(() -> startIndexer(backRpm));
  }

  // #region Dashboard
  // Methods for creating and updating dashboards
  /**
   * Publishes indexer telemetry to SmartDashboard.
   */
  public void updateDashboards() {
    if (!GlobalConstants.telemetryAtLeast(IndexerConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
    // INFO level telemetry goes here

    SmartDashboard.putString("Subsystems/Indexer/Indexer/State", getIndexStateName());
    SmartDashboard.putString("Subsystems/Indexer/Indexer/StateColor", getIndexStateColor());
    SmartDashboard.putNumber("Subsystems/Indexer/Indexer/RequestedSpeed", getIndexerCommandedSpeedRPM());
    SmartDashboard.putNumber("Subsystems/Indexer/Indexer/CurrentSpeed", getIndexerCurrentSpeedRPM());

    SmartDashboard.putString("Subsystems/Indexer/Knuckle/State", getKnuckleStateName());
    SmartDashboard.putString("Subsystems/Indexer/Knuckle/StateColor", getKnuckleStateColor());
    SmartDashboard.putNumber("Subsystems/Indexer/Knuckle/RequestedSpeed", getKnuckleCommandedSpeedRPM());
    SmartDashboard.putNumber("Subsystems/Indexer/Knuckle/CurrentSpeed", getKnuckleCurrentSpeedRPM());

    SmartDashboard.putString("Subsystems/Indexer/LiveBottom/State", getLiveBottomStateName());
    SmartDashboard.putString("Subsystems/Indexer/LiveBottom/StateColor", getLiveBottomStateColor());
    SmartDashboard.putNumber("Subsystems/Indexer/LiveBottom/RequestedPower", getLiveBottomCommandedPower());
    SmartDashboard.putNumber("Subsystems/Indexer/LiveBottom/CurrentSpeed", getLiveBottomCurrentSpeedRPM());

    SmartDashboard.putString("Subsystems/Indexer/MatrixBreaker/State", getMatrixBreakerStateName());
    SmartDashboard.putString("Subsystems/Indexer/MatrixBreaker/StateColor", getMatrixBreakerStateColor());
    SmartDashboard.putNumber("Subsystems/Indexer/MatrixBreaker/RequestedOutput", getMatrixBreakerCommandedOutput());
    SmartDashboard.putNumber("Subsystems/Indexer/MatrixBreaker/CurrentOutput", getMatrixBreakerOutput());

    SmartDashboard.putString("Subsystems/Indexer/Shoe/State", getShoeStateName());
    SmartDashboard.putString("Subsystems/Indexer/Shoe/StateColor", getShoeStateColor());
    SmartDashboard.putNumber("Subsystems/Indexer/Shoe/RequestedOutput", getShoeCommandedOutput());
    SmartDashboard.putNumber("Subsystems/Indexer/Shoe/CurrentOutput", getShoeOutput());

    if (!GlobalConstants.telemetryAtLeast(IndexerConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
    // DEBUG level telemetry goes here
  }
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
   * Returns the current live bottom state.
   *
   * @return Current live bottom state.
   */
  public State getLiveBottomState() {
    return m_curLiveBottomState;
  }

  /**
   * Returns the current live bottom state name.
   *
   * @return Live bottom state name.
   */
  public String getLiveBottomStateName() {
    return m_curLiveBottomState.toString();
  }

  /**
   * Returns the current live bottom state color.
   *
   * @return Live bottom state color as a hex string.
   */
  public String getLiveBottomStateColor() {
    return m_curLiveBottomState.getColor();
  }

  /**
   * Returns the current matrix breaker state.
   *
   * @return Current matrix breaker state.
   */
  public State getMatrixBreakerState() {
    return m_curMatrixBreakerState;
  }

  /**
   * Returns the current matrix breaker state name.
   *
   * @return Matrix breaker state name.
   */
  public String getMatrixBreakerStateName() {
    return m_curMatrixBreakerState.toString();
  }

  /**
   * Returns the current matrix breaker state color.
   *
   * @return Matrix breaker state color as a hex string.
   */
  public String getMatrixBreakerStateColor() {
    return m_curMatrixBreakerState.getColor();
  }

  /**
   * Returns the current shoe state.
   *
   * @return Current shoe state.
   */
  public State getShoeState() {
    return m_curShoeState;
  }

  /**
   * Returns the current shoe state name.
   *
   * @return Shoe state name.
   */
  public String getShoeStateName() {
    return m_curShoeState.toString();
  }

  /**
   * Returns the current shoe state color.
   *
   * @return Shoe state color as a hex string.
   */
  public String getShoeStateColor() {
    return m_curShoeState.getColor();
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
   * Returns the commanded live bottom output power.
   *
   * @return Commanded live bottom power from -1.0 to 1.0.
   */
  public double getLiveBottomCommandedPower() {
    return m_liveBottomCommandedPower;
  }

  /**
   * Returns the commanded matrix breaker servo output value.
   *
   * @return Servo output value from 0.0 to 1.0.
   */
  public double getMatrixBreakerCommandedOutput() {
    return m_matrixBreakerCommandedOutput;
  }

  /**
   * Returns the commanded shoe servo output value.
   *
   * @return Servo output value from 0.0 to 1.0.
   */
  public double getShoeCommandedOutput() {
    return m_shoeCommandedOutput;
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

  /**
   * Returns the current measured live bottom speed in RPM.
   *
   * @return Current live bottom speed in RPM.
   */
  public double getLiveBottomCurrentSpeedRPM() {
    return m_liveBottomMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Returns the current matrix breaker servo output value.
   *
   * @return Servo output value from 0.0 to 1.0.
   */
  public double getMatrixBreakerOutput() {
    return m_matrixBreakerServo.get();
  }

  /**
   * Returns the current shoe servo output value.
   *
   * @return Servo output value from 0.0 to 1.0.
   */
  public double getShoeOutput() {
    return m_shoeServo.get();
  }

  /**
   * Returns the derived indexer RPM from the shooter master commanded RPM.
   *
   * @return Derived indexer RPM.
   */
  public double getDerivedIndexerCommandedSpeedRPM() {
    return calculateIndexerRPMFromBackRPM(RobotContainer.shooter.getCommandedMasterRPM());
  }

  /**
   * Returns the derived knuckle RPM from the shooter master commanded RPM.
   *
   * @return Derived knuckle RPM.
   */
  public double getDerivedKnuckleCommandedSpeedRPM() {
    return calculateKnuckleRPMFromBackRPM(RobotContainer.shooter.getCommandedMasterRPM());
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
   * Sets all indexer-related mechanisms to their neutral safe state.
   */
  public void indexerSystemNeutral() {
    indexerNeutral();
    liveBottomStop();
    matrixBreakerStop();
    m_liveBottomMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Commands indexer and knuckle motors to zero velocity and sets coast neutral mode.
   */
  public void indexerNeutral() {
    m_indexerMotor.setControl(m_indexerVelocityRequest.withVelocity(0.0));
    m_knuckleMotor.setControl(m_knuckleVelocityRequest.withVelocity(0.0));
    // m_indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    // m_knuckleMotor.setNeutralMode(NeutralModeValue.Coast);
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
    long roundedIndexerRpm = Math.round(indexerRpm);
    long roundedKnuckleRpm = Math.round(knuckleRpm);
    double roundedIndexerRpmAsDouble = roundedIndexerRpm;
    double roundedKnuckleRpmAsDouble = roundedKnuckleRpm;
    double indexerRps = Helpers.RPMtoRPS(roundedIndexerRpmAsDouble);
    double knuckleRps = Helpers.RPMtoRPS(roundedKnuckleRpmAsDouble);
    m_indexerMotor.setControl(m_indexerVelocityRequest.withVelocity(indexerRps));
    m_knuckleMotor.setControl(m_knuckleVelocityRequest.withVelocity(knuckleRps));
    m_indexerCommandedSpeedRpm = roundedIndexerRpmAsDouble;
    m_knuckleCommandedSpeedRpm = roundedKnuckleRpmAsDouble;
    m_curIndexerState = stateFromSignedValue(roundedIndexerRpmAsDouble);
    m_curKnuckleState = stateFromSignedValue(roundedKnuckleRpmAsDouble);
    NCDebug.Debug.debug("Indexer: Set speed " + roundedIndexerRpm + "rpm indexer, " + roundedKnuckleRpm + "rpm knuckle");
  }

  /**
   * Sets the knuckle motor speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for the knuckle motor.
   */
  public void setKnuckleSpeedRPM(double rpm) {
    long roundedRpm = Math.round(rpm);
    double roundedRpmAsDouble = roundedRpm;
    double rps = Helpers.RPMtoRPS(roundedRpmAsDouble);
    m_knuckleMotor.setControl(m_knuckleVelocityRequest.withVelocity(rps));
    m_knuckleCommandedSpeedRpm = roundedRpmAsDouble;
    m_curKnuckleState = stateFromSignedValue(roundedRpmAsDouble);
    NCDebug.Debug.debug("Indexer: Set speed " + roundedRpm + "rpm knuckle");
  }

  /**
   * Sets live bottom output power and keeps shoe direction synchronized.
   *
   * @param power Percent output from -1.0 to 1.0.
   */
  public void setLiveBottomPower(double power) {
    double limitedPower = Math.max(-1.0, Math.min(1.0, power));
    m_liveBottomMotor.setControl(m_DutyCycle.withOutput(limitedPower));
    syncShoeToLiveBottomPower(limitedPower);
    m_liveBottomCommandedPower = limitedPower;
    m_curLiveBottomState = stateFromSignedValue(limitedPower);
  }

  /**
   * Runs live bottom forward using configured forward power.
   * Shoe direction is synchronized with live bottom.
   */
  public void liveBottomForward() {
    setLiveBottomPower(IndexerConstants.LiveBottom.kForwardPower);
    NCDebug.Debug.debug("Indexer: LiveBottom Forward");
  }

  /**
   * Runs live bottom in reverse using configured reverse power.
   * Shoe direction is synchronized with live bottom.
   */
  public void liveBottomReverse() {
    setLiveBottomPower(-IndexerConstants.LiveBottom.kReversePower);
    NCDebug.Debug.debug("Indexer: LiveBottom Reverse");
  }

  /**
   * Stops live bottom output.
   * Shoe output is synchronized and also stopped.
   */
  public void liveBottomStop() {
    setLiveBottomPower(0.0);
    NCDebug.Debug.debug("Indexer: LiveBottom Stop");
  }

  /**
   * Runs the matrix breaker servo forward.
   */
  public void matrixBreakerForward() {
    m_matrixBreakerServo.set(IndexerConstants.MatrixBreaker.kForward);
    m_matrixBreakerCommandedOutput = IndexerConstants.MatrixBreaker.kForward;
    m_curMatrixBreakerState = State.FWD;
    NCDebug.Debug.debug("Indexer: MatrixBreaker Forward");
  }

  /**
   * Runs the matrix breaker servo in reverse.
   */
  public void matrixBreakerReverse() {
    m_matrixBreakerServo.set(IndexerConstants.MatrixBreaker.kReverse);
    m_matrixBreakerCommandedOutput = IndexerConstants.MatrixBreaker.kReverse;
    m_curMatrixBreakerState = State.REV;
    NCDebug.Debug.debug("Indexer: MatrixBreaker Reverse");
  }

  /**
   * Stops the matrix breaker servo output.
   */
  public void matrixBreakerStop() {
    m_matrixBreakerServo.set(IndexerConstants.MatrixBreaker.kStop);
    m_matrixBreakerCommandedOutput = IndexerConstants.MatrixBreaker.kStop;
    m_curMatrixBreakerState = State.STOP;
    NCDebug.Debug.debug("Indexer: MatrixBreaker Stop");
  }

  /**
   * Runs the shoe servo forward.
   */
  public void shoeForward() {
    m_shoeServo.set(IndexerConstants.Shoe.kForward);
    m_shoeCommandedOutput = IndexerConstants.Shoe.kForward;
    m_curShoeState = State.FWD;
    NCDebug.Debug.debug("Indexer: Shoe Forward");
  }

  /**
   * Runs the shoe servo in reverse.
   */
  public void shoeReverse() {
    m_shoeServo.set(IndexerConstants.Shoe.kReverse);
    m_shoeCommandedOutput = IndexerConstants.Shoe.kReverse;
    m_curShoeState = State.REV;
    NCDebug.Debug.debug("Indexer: Shoe Reverse");
  }

  /**
   * Stops the shoe servo output.
   */
  public void shoeStop() {
    m_shoeServo.set(IndexerConstants.Shoe.kStop);
    m_shoeCommandedOutput = IndexerConstants.Shoe.kStop;
    m_curShoeState = State.STOP;
    NCDebug.Debug.debug("Indexer: Shoe Stop");
  }

  /**
   * Synchronizes shoe direction with live bottom direction.
   *
   * @param liveBottomPower Signed live bottom output power.
   */
  private void syncShoeToLiveBottomPower(double liveBottomPower) {
    if (liveBottomPower > 0.0) {
      shoeForward();
    } else if (liveBottomPower < 0.0) {
      shoeReverse();
    } else {
      shoeStop();
    }
  }

  /**
   * Starts indexer feed mechanisms using setpoints derived from the configured
   * currently commanded back shooter RPM.
   * Also starts live bottom, shoe, and matrix breaker forward.
   */
  public void startIndexer() {
    double backRpm = RobotContainer.shooter.getCommandedMasterRPM();
    startIndexer(backRpm);
  }

  /**
   * Starts indexer feed mechanisms using setpoints derived from a supplied
   * back shooter RPM.
   * Also starts live bottom, shoe, and matrix breaker forward.
   *
   * @param backRpm Back shooter RPM (master).
   */
  public void startIndexer(double backRpm) {
    double indexerRpm = calculateIndexerRPMFromBackRPM(backRpm);
    double knuckleRpm = calculateKnuckleRPMFromBackRPM(backRpm);
    setIndexerSpeedRPM(indexerRpm, knuckleRpm);
    liveBottomForward();
    matrixBreakerForward();
  }

  /**
   * Calculates indexer RPM derived from back shooter RPM.
   *
   * @param backShooterRpm Back shooter RPM (master).
   * @return Indexer RPM.
   */
  public double calculateIndexerRPMFromBackRPM(double backShooterRpm) {
    return backShooterRpm * ShooterConstants.Multipliers.kIndexerFromBack;
  }

  /**
   * Calculates knuckle RPM derived from back shooter RPM.
   *
   * @param backShooterRpm Back shooter RPM (master).
   * @return Knuckle RPM.
   */
  public double calculateKnuckleRPMFromBackRPM(double backShooterRpm) {
    return backShooterRpm * ShooterConstants.Multipliers.kKnuckleFromBack;
  }

  /**
   * Converts a signed command value into a directional state.
   *
   * @param value Signed command value.
   * @return {@link State#FWD} for positive values, {@link State#REV} for negative values,
   *         otherwise {@link State#STOP}.
   */
  private State stateFromSignedValue(double value) {
    if (value > 0.0) {
      return State.FWD;
    } else if (value < 0.0) {
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
