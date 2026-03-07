
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
  private State m_curState = State.STOP;
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
    m_curState = State.STOP;
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
   * Returns the current shooter state.
   *
   * @return Current state.
   */
  public State getState() {
    return m_curState;
  }

  /**
   * Returns the current shooter state name.
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
    m_curState = State.STOP;
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
    if (frontRpm > 0.0) {
      m_curState = State.FWD;
    } else if (frontRpm < 0.0) {
      m_curState = State.REV;
    } else if (backRpm > 0.0) {
      m_curState = State.FWD;
    } else if (backRpm < 0.0) {
      m_curState = State.REV;
    } else {
      m_curState = State.STOP;
    }
    NCDebug.Debug.debug("Shooter: Set speed " + frontRpm + "RPM front, " + backRpm + "RPM back");
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
