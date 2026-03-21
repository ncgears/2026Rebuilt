
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Climber.
 * It is responsible for extending and retracting the elevator/climber.
 */
public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;
  // private and public variables defined here

  private CANdi candi = new CANdi(ClimberConstants.kCANdiID);
  
  /** Operating states for the climber subsystem. */
  public enum State {
    /** Climber moving upward. */
    UP(DashboardConstants.Colors.ORANGE),
    /** Climber moving downward. */
    DOWN(DashboardConstants.Colors.RED),
    /** Climber holding position. */
    HOLD(DashboardConstants.Colors.GREEN),
    /** Climber stopped. */
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
  private TalonFX m_motor1;
  private State m_curState = State.STOP;

  /**
   * Returns true when the cage switch is engaged
   */
  public final Trigger hasCage = new Trigger(this::getStartSwitch);
  /**
   * Returns true when the climber has reached its limit
   */
  public final Trigger climbComplete = new Trigger(this::getClimbComplete);

  /**
   * Returns the instance of the ClimberSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return ClimberSubsystem instance
   */
  public static ClimberSubsystem getInstance() {
    if (instance == null)
      instance = new ClimberSubsystem();
    return instance;
  }

  /** Creates the climber subsystem and configures hardware. */
  public ClimberSubsystem() {
    m_motor1 = new TalonFX(ClimberConstants.kMotorID, ClimberConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.climberFXConfig));

    init();
  }

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    climberStop();
    NCDebug.Debug.debug("Climber: Initialized");
  }

  /** Runs periodic updates for the climber subsystem. */
  @Override
  public void periodic() {
    updateDashboards();
  }


  // #region Commands
  /**
   * neutralCommand is used to reset this system into a safe state when disabled. 
   * It is called when the robot is disabled to reset counters, states, etc.
   *
   * @return command that does nothing when scheduled
   */
  public Command neutralCommand() {
    return Commands.none();
  }

  /**
   * Publishes climber telemetry to SmartDashboard.
   */
  public void updateDashboards() {
    if (!GlobalConstants.telemetryAtLeast(ClimberConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
    // INFO level telemetry goes here

    SmartDashboard.putString("Subsystems/Climber/State", getStateName());
    SmartDashboard.putString("Subsystems/Climber/StateColor", getStateColor());
    SmartDashboard.putBoolean("Subsystems/Climber/Complete", getClimbComplete());
    SmartDashboard.putNumber("Subsystems/Climber/Position", NCDebug.General.roundDouble(getPosition().in(Units.Rotations), 7));
    SmartDashboard.putBoolean("Subsystems/Climber/StartSwitch", getStartSwitch());
    SmartDashboard.putBoolean("Subsystems/Climber/EndSwitch", getEndSwitch());

    if (!GlobalConstants.telemetryAtLeast(ClimberConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
    // DEBUG level telemetry goes here
  }

  /**
   * Returns the current climber state.
   *
   * @return Current state.
   */
  public State getState() {
    return m_curState;
  }

  /**
   * Returns the current state name.
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
   * Returns true if the climb is complete.
   *
   * @return True when climb is complete.
   */
  public boolean getClimbComplete() {
    return getClimbSwitch();
  }

  /**
   * Returns true if the climber is at its limit.
   *
   * @return True when at limit.
   */
  public boolean atLimit() {
    return getClimbComplete();
  }

  /**
   * Returns the first cage switch state.
   *
   * @return True when the switch is triggered.
   */
  private boolean getStartSwitch() {
    return candi.getS1Closed().isAllGood();
  }

  /**
   * Returns the second cage switch state.
   *
   * @return True when the switch is triggered.
   */
  private boolean getEndSwitch() {
    return candi.getS2Closed().isAllGood();
  }

  /**
   * Returns the climb-complete switch state.
   *
   * @return True when the climb switch is triggered.
   */
  private boolean getClimbSwitch() {
    if(Robot.isSimulation()) {
      return true;
    }
    return (m_motor1.getForwardLimit().getValue() == ForwardLimitValue.Open);
  }

  /**
   * Returns the motor position in rotations.
   *
   * @return Motor position.
   */
  public Angle getPosition() {
    return m_motor1.getPosition().getValue();
  }

  /**
   * Moves the climber using percent output.
   *
   * @param power Output power (-1 to 1).
   */
  public void climberMove(double power) {
    if (power > 0) {
      if (m_curState != State.UP) {
        NCDebug.Debug.debug("Climber: Up (" + power + ")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if (power < 0) {
      if (m_curState != State.DOWN) {
        NCDebug.Debug.debug("Climber: Down (" + power + ")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { // 0 power
      if (m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        NCDebug.Debug.debug("Climber: Hold");
      }
    }
  }

  /**
   * Creates a command to move the climber.
   *
   * @param power Power supplier.
   * @return Command that drives the climber.
   */
  public Command climberMoveC(DoubleSupplier power) {
    return runOnce(() -> climberMove(power.getAsDouble()));
  }

  /**
   * Creates a command to stop the climber.
   *
   * @return Command that stops the climber.
   */
  public Command climberStopC() {
    return runOnce(() -> climberStop());
  }

  /**
   * Creates a command to hold the climber.
   *
   * @return Command that holds the climber.
   */
  public Command climberHoldC() {
    return runOnce(() -> climberHold());
  }

  /** Commands the climber to move up. */
  public void climberUp() {
    m_curState = State.UP;
    climberMove(ClimberConstants.kClimbPower);
    NCDebug.Debug.debug("Climber: Up");
  }

  /** Commands the climber to move down. */
  public void climberDown() {
    m_curState = State.DOWN;
    climberMove(-ClimberConstants.kClimbPower);
    NCDebug.Debug.debug("Climber: Down");
  }

  /** Commands the climber to hold position. */
  public void climberHold() {
    m_motor1.setControl(m_brake);
    if (m_curState != State.HOLD) {
      m_curState = State.HOLD;
      NCDebug.Debug.debug("Climber: Hold");
    }
  }

  /** Stops climber output and sets neutral. */
  public void climberStop() {
    m_motor1.setControl(m_neutral);
    if (m_curState != State.STOP) {
      m_curState = State.STOP;
      NCDebug.Debug.debug("Climber: Stop");
    }
  }

  /** Sets the motor neutral mode to coast. */
  public void setCoast() {
    m_motor1.setNeutralMode(NeutralModeValue.Coast);
    NCDebug.Debug.debug("Climber: Switch to Coast");
  }

  /** Sets the motor neutral mode to brake. */
  public void setBrake() {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    NCDebug.Debug.debug("Climber: Switch to Brake");
  }

  // #region SysID Functions
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, // default ramp rate 1V/s
      Volts.of(4), // reduce dynamic step voltage to 4 to prevent brownout
      null, // default timeout 10s
      (state) -> SignalLogger.writeString("SysId_State", state.toString())),
    new SysIdRoutine.Mechanism(
      (volts) -> m_motor1.setControl(m_voltReq.withOutput(volts.in(Volts))),
      null,
      this));

  /**
   * Runs the SysId quasistatic test.
   *
   * @param direction Direction to run.
   * @return Command for the test.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Runs the SysId dynamic test.
   *
   * @param direction Direction to run.
   * @return Command for the test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Runs the full SysId command sequence.
   *
   * @return Command sequence.
   */
  public Command runSysIdCommand() {
    return Commands.sequence(
      sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(this::atLimit),
      sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(this::atLimit),
      sysIdDynamic(SysIdRoutine.Direction.kForward).until(this::atLimit),
      sysIdDynamic(SysIdRoutine.Direction.kReverse).until(this::atLimit));
  }
  // #endregion

}
