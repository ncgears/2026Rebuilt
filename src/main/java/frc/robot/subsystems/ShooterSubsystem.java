
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Helpers;
import frc.robot.utils.NCDebug;

/**
 * This subsystem handles managing the Shooter subsystem.
 * It is responsible for doing some stuff.
 */
public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;
  // #region Declarations
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

  private TalonFX m_shooterFrontMotor, m_shooterBackMotor;
  private final VelocityVoltage m_frontVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_backVelocityRequest = new VelocityVoltage(0.0);
  private State m_curFrontState = State.STOP;
  private State m_curBackState = State.STOP;
  private double m_frontCommandedSpeedRpm = 0.0;
  private double m_backCommandedSpeedRpm = 0.0;
  // #endregion Declarations

  // #region Triggers
  // Trigger definitions
  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the ShooterSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return ShooterSubsystem instance
   */
  public static ShooterSubsystem getInstance() {
    if (instance == null)
      instance = new ShooterSubsystem();
    return instance;
  }

  /** Creates the Shooter subsystem and initializes state. */
  public ShooterSubsystem() {
    // initialize values for private and public variables, etc.
    m_shooterFrontMotor = new TalonFX(ShooterConstants.Front.kMotorID, ShooterConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_shooterFrontMotor.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFrontFXConfig));

    m_shooterBackMotor = new TalonFX(ShooterConstants.Back.kMotorID, ShooterConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_shooterBackMotor.getConfigurator().apply(RobotContainer.ctreConfigs.shooterBackFXConfig));

    init();
  }

  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    m_curFrontState = State.STOP;
    m_curBackState = State.STOP;
    m_frontCommandedSpeedRpm = 0.0;
    m_backCommandedSpeedRpm = 0.0;
    NCDebug.Debug.debug("Shooter: Initialized");
  }

  /** Runs periodically for the Shooter subsystem. */
  @Override
  public void periodic() {
  }
  // #endregion Setup

  // #region Commands
  /**
   * neutralCommand is used to reset this system into a safe state when disabled. 
   * It is called when the robot is disabled to reset counters, states, etc.
   *
   * @return Command that sets both shooter motors to coast neutral mode.
   */
  public Command neutralCommand() {
    return shooterNeutralC();
  }

  /**
   * Creates a command to set both shooter wheels to the same speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for both shooter motors.
   * @return Command that updates both shooter speed setpoints.
   */
  public Command setShooterSpeedC(double rpm) {
    return runOnce(() -> setShooterSpeedRPM(rpm));
  }

  /**
   * Creates a command to set independent shooter wheel speed setpoints in RPM.
   *
   * @param frontRpm Target speed in RPM for the front shooter motor.
   * @param backRpm Target speed in RPM for the back shooter motor.
   * @return Command that updates front and back shooter speed setpoints.
   */
  public Command setShooterSpeedC(double frontRpm, double backRpm) {
    return runOnce(() -> setShooterSpeedRPM(frontRpm, backRpm));
  }

  // #region Dashboard
  // Methods for creating and updating dashboards
  // #endregion Dashboard

  // #region Getters
  // Methods for getting data for subsystem

  /**
   * Returns the current front shooter state.
   *
   * @return Current front state.
   */
  public State getState() {
    return m_curFrontState;
  }

  /**
   * Returns the current front shooter state.
   *
   * @return Current front state.
   */
  public State getFrontState() {
    return m_curFrontState;
  }

  /**
   * Returns the current back shooter state.
   *
   * @return Current back state.
   */
  public State getBackState() {
    return m_curBackState;
  }

  /**
   * Returns the current front shooter state name.
   *
   * @return Front state name.
   */
  public String getFrontStateName() {
    return m_curFrontState.toString();
  }

  /**
   * Returns the current back shooter state name.
   *
   * @return Back state name.
   */
  public String getBackStateName() {
    return m_curBackState.toString();
  }

  /**
   * Returns the current front shooter state color.
   *
   * @return Front state color as hex string.
   */
  public String getFrontStateColor() {
    return m_curFrontState.getColor();
  }

  /**
   * Returns the current back shooter state color.
   *
   * @return Back state color as hex string.
   */
  public String getBackStateColor() {
    return m_curBackState.getColor();
  }

  /**
   * Returns the front and back shooter state names.
   *
   * @return State names as "FRONT_STATE/BACK_STATE".
   */
  public String getStateName() {
    return m_curFrontState.toString() + "/" + m_curBackState.toString();
  }

  /**
   * Returns the state color summary across front and back shooter states.
   *
   * @return Shared state color when both match, otherwise orange.
   */
  public String getStateColor() {
    if (m_curFrontState == m_curBackState) {
      return m_curFrontState.getColor();
    }
    return DashboardConstants.Colors.ORANGE;
  }

  /**
   * Returns the commanded front shooter speed setpoint in RPM.
   *
   * @return Commanded front shooter speed in RPM.
   */
  public double getFrontCommandedSpeedRPM() {
    return m_frontCommandedSpeedRpm;
  }

  /**
   * Returns the commanded back shooter speed setpoint in RPM.
   *
   * @return Commanded back shooter speed in RPM.
   */
  public double getBackCommandedSpeedRPM() {
    return m_backCommandedSpeedRpm;
  }

  /**
   * Returns the current measured front shooter speed in RPM.
   *
   * @return Current front shooter speed in RPM.
   */
  public double getFrontCurrentSpeedRPM() {
    return m_shooterFrontMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Returns the current measured back shooter speed in RPM.
   *
   * @return Current back shooter speed in RPM.
   */
  public double getBackCurrentSpeedRPM() {
    return m_shooterBackMotor.getVelocity().getValueAsDouble() * 60.0;
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
   * Sets both shooter motors to coast neutral mode.
   */
  public void shooterNeutral() {
    m_shooterFrontMotor.setNeutralMode(NeutralModeValue.Coast);
    m_shooterBackMotor.setNeutralMode(NeutralModeValue.Coast);
    m_curFrontState = State.STOP;
    m_curBackState = State.STOP;
    m_frontCommandedSpeedRpm = 0.0;
    m_backCommandedSpeedRpm = 0.0;
    NCDebug.Debug.debug("Shooter: Switch to Coast");
  }

  /**
   * Sets both shooter wheels to the same speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for both shooter motors.
   */
  public void setShooterSpeedRPM(double rpm) {
    setShooterSpeedRPM(rpm, rpm);
  }

  /**
   * Sets independent shooter wheel speed setpoints in RPM.
   *
   * @param frontRpm Target speed in RPM for the front shooter motor.
   * @param backRpm Target speed in RPM for the back shooter motor.
   */
  public void setShooterSpeedRPM(double frontRpm, double backRpm) {
    double frontRps = Helpers.RPMtoRPS(frontRpm);
    double backRps = Helpers.RPMtoRPS(backRpm);
    m_shooterFrontMotor.setControl(m_frontVelocityRequest.withVelocity(frontRps));
    m_shooterBackMotor.setControl(m_backVelocityRequest.withVelocity(backRps));
    m_frontCommandedSpeedRpm = frontRpm;
    m_backCommandedSpeedRpm = backRpm;
    m_curFrontState = stateFromRPM(frontRpm);
    m_curBackState = stateFromRPM(backRpm);
    NCDebug.Debug.debug("Shooter: Set speed " + frontRpm + "RPM front, " + backRpm + "RPM back");
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
   * Creates a command to set both shooter motors to coast neutral mode.
   *
   * @return Command that updates both shooter motor neutral modes.
   */
  public Command shooterNeutralC() {
    return runOnce(this::shooterNeutral);
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
