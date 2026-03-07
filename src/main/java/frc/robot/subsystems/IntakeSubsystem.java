
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.DashboardConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.Helpers;
import frc.robot.utils.NCDebug;

/**
 * This subsystem handles managing the Intake subsystem.
 * It is responsible for doing some stuff.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;
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

  public enum DeployPosition {
    STOW(IntakeConstants.Deploy.Positions.kStow),
    OUT(IntakeConstants.Deploy.Positions.kOut),
    PROTECT(IntakeConstants.Deploy.Positions.kProtect);

    private final double position;

    DeployPosition(double position) {
      this.position = position;
    }

    /**
     * Returns the deploy target position in rotations.
     *
     * @return Deploy position in rotations.
     */
    public double getPosition() {
      return this.position;
    }
  }

  private CANcoder m_deployEncoder;
  private TalonFX m_deployMotor, m_intakeMotor;
  private Servo m_shoeServo;
  private final VelocityVoltage m_intakeVelocityRequest = new VelocityVoltage(0.0);
  private final PositionVoltage m_deployPositionRequest = new PositionVoltage(0.0);
  private State m_curIntakeState = State.STOP;
  private State m_curDeployState = State.STOP;
  private double m_intakeCommandedSpeedRpm = 0.0;
  private double m_deployCommandedPositionRotations = 0.0;
  // #endregion Declarations

  // #region Triggers
  // Trigger definitions
  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the IntakeSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return IntakeSubsystem instance
   */
  public static IntakeSubsystem getInstance() {
    if (instance == null)
      instance = new IntakeSubsystem();
    return instance;
  }

  /** Creates the Intake subsystem and initializes state. */
  public IntakeSubsystem() {
    // initialize values for private and public variables, etc.
    m_deployEncoder = new CANcoder(IntakeConstants.Deploy.kCANcoderID, IntakeConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_deployEncoder.getConfigurator().apply(RobotContainer.ctreConfigs.deployCCConfig));

    m_deployMotor = new TalonFX(IntakeConstants.Deploy.kMotorID, IntakeConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_deployMotor.getConfigurator().apply(RobotContainer.ctreConfigs.deployFXConfig));

    m_intakeMotor = new TalonFX(IntakeConstants.Intake.kMotorID, IntakeConstants.canBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_intakeMotor.getConfigurator().apply(RobotContainer.ctreConfigs.intakeFXConfig));

    m_shoeServo = new Servo(IntakeConstants.Shoe.kServoID);

    init();
  }

  /**
   * The init method resets and operational state of the subsystem
   */
  public void init() {
    // set initial stuff, etc.
    m_curIntakeState = State.STOP;
    m_curDeployState = State.STOP;
    m_intakeCommandedSpeedRpm = 0.0;
    seedDeployMotorPositionFromCANcoder();
    m_deployCommandedPositionRotations = getDeployPositionRotations();
    NCDebug.Debug.debug("Intake: Initialized");
  }

  /** Runs periodically for the Intake subsystem. */
  @Override
  public void periodic() {
  }
  // #endregion Setup

  // #region Commands
  /**
   * neutralCommand is used to reset this system into a safe state when disabled. 
   * It is called when the robot is disabled to reset counters, states, etc.
   *
   * @return Command that sets intake and deploy motors to coast neutral mode.
   */
  public Command neutralCommand() {
    return intakeNeutralC();
  }

  // #region Dashboard
  // Methods for creating and updating dashboards
  // #endregion Dashboard

  // #region Getters
  // Methods for getting data for subsystem

  /**
   * Returns the current intake motor state.
   *
   * @return Current intake state.
   */
  public State getIntakeState() {
    return m_curIntakeState;
  }

  /**
   * Returns the current intake motor state name.
   *
   * @return Intake state name.
   */
  public String getIntakeStateName() {
    return m_curIntakeState.toString();
  }

  /**
   * Returns the current intake motor state color.
   *
   * @return Intake state color as hex string.
   */
  public String getIntakeStateColor() {
    return m_curIntakeState.getColor();
  }

  /**
   * Returns the current deploy motor state.
   *
   * @return Current deploy state.
   */
  public State getDeployState() {
    return m_curDeployState;
  }

  /**
   * Returns the current deploy motor state name.
   *
   * @return Deploy state name.
   */
  public String getDeployStateName() {
    return m_curDeployState.toString();
  }

  /**
   * Returns the current deploy motor state color.
   *
   * @return Deploy state color as hex string.
   */
  public String getDeployStateColor() {
    return m_curDeployState.getColor();
  }

  /**
   * Returns the commanded intake speed setpoint in RPM.
   *
   * @return Commanded intake speed in RPM.
   */
  public double getIntakeCommandedSpeedRPM() {
    return m_intakeCommandedSpeedRpm;
  }

  /**
   * Returns the commanded deploy position setpoint in rotations.
   *
   * @return Commanded deploy position in rotations.
   */
  public double getDeployCommandedPositionRotations() {
    return m_deployCommandedPositionRotations;
  }

  /**
   * Returns the current measured intake motor speed in RPM.
   *
   * @return Intake motor speed in RPM.
   */
  public double getIntakeCurrentSpeedRPM() {
    return m_intakeMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Returns the current measured deploy motor speed in RPM.
   *
   * @return Deploy motor speed in RPM.
   */
  public double getDeployCurrentSpeedRPM() {
    return m_deployMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Returns the current deploy motor position in rotations.
   *
   * @return Deploy position in rotations.
   */
  public double getDeployPositionRotations() {
    return m_deployMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the deploy CANcoder absolute position in rotations.
   *
   * @return Deploy CANcoder absolute position in rotations.
   */
  public double getDeployAbsolutePositionRotations() {
    return m_deployEncoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Returns the current shoe servo output command.
   *
   * @return Servo output value from 0.0 to 1.0.
   */
  public double getShoeOutput() {
    return m_shoeServo.get();
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
   * Sets intake and deploy motors to coast neutral mode and resets state tracking.
   */
  public void intakeNeutral() {
    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
    m_curIntakeState = State.STOP;
    m_curDeployState = State.STOP;
    m_intakeCommandedSpeedRpm = 0.0;
    m_deployCommandedPositionRotations = getDeployPositionRotations();
    NCDebug.Debug.debug("Intake: Switch to Coast");
  }

  /**
   * Creates a command to set intake and deploy motors to coast neutral mode.
   *
   * @return Command that neutralizes intake and deploy motors.
   */
  public Command intakeNeutralC() {
    return runOnce(this::intakeNeutral);
  }

  /**
   * Sets the intake motor speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for the intake motor.
   */
  public void setIntakeSpeedRPM(double rpm) {
    double rps = Helpers.RPMtoRPS(rpm);
    m_intakeMotor.setControl(m_intakeVelocityRequest.withVelocity(rps));
    m_intakeCommandedSpeedRpm = rpm;
    m_curIntakeState = stateFromSignedValue(rpm);
    NCDebug.Debug.debug("Intake: Set speed " + rpm + "RPM intake");
  }

  /**
   * Creates a command to set the intake motor speed setpoint in RPM.
   *
   * @param rpm Target speed in RPM for the intake motor.
   * @return Command that updates the intake speed setpoint.
   */
  public Command setIntakeSpeedC(double rpm) {
    return runOnce(() -> setIntakeSpeedRPM(rpm));
  }

  /**
   * Sets the deploy motor position setpoint in rotations.
   *
   * @param rotations Target position in rotations for the deploy motor.
   */
  public void setDeployPositionRotations(double rotations) {
    m_deployMotor.setControl(m_deployPositionRequest.withPosition(rotations));
    m_deployCommandedPositionRotations = rotations;
    m_curDeployState = stateFromSignedValue(rotations - getDeployPositionRotations());
    NCDebug.Debug.debug("Intake: Set position " + rotations + " rotations deploy");
  }

  /**
   * Creates a command to set the deploy motor position setpoint in rotations.
   *
   * @param rotations Target position in rotations for the deploy motor.
   * @return Command that updates the deploy position setpoint.
   */
  public Command setDeployPositionC(double rotations) {
    return runOnce(() -> setDeployPositionRotations(rotations));
  }

  /**
   * Sets the deploy motor target position using a named deploy position.
   *
   * @param position Named deploy target.
   */
  public void setDeployPosition(DeployPosition position) {
    setDeployPositionRotations(position.getPosition());
  }

  /**
   * Creates a command to set deploy to a named position.
   *
   * @param position Named deploy target.
   * @return Command that updates the deploy position setpoint.
   */
  public Command setDeployPositionC(DeployPosition position) {
    return runOnce(() -> setDeployPosition(position));
  }

  /**
   * Sets intake speed and deploy position setpoints.
   *
   * @param intakeRpm Target speed in RPM for the intake motor.
   * @param deployPositionRotations Target position in rotations for the deploy motor.
   */
  public void setIntakeDeploySetpoints(double intakeRpm, double deployPositionRotations) {
    setIntakeSpeedRPM(intakeRpm);
    setDeployPositionRotations(deployPositionRotations);
  }

  /**
   * Sets intake speed and deploy position setpoints using a named deploy position.
   *
   * @param intakeRpm Target speed in RPM for the intake motor.
   * @param deployPosition Named deploy target.
   */
  public void setIntakeDeploySetpoints(double intakeRpm, DeployPosition deployPosition) {
    setIntakeSpeedRPM(intakeRpm);
    setDeployPosition(deployPosition);
  }

  /**
   * Creates a command to set intake speed and deploy position setpoints.
   *
   * @param intakeRpm Target speed in RPM for the intake motor.
   * @param deployPositionRotations Target position in rotations for the deploy motor.
   * @return Command that updates intake speed and deploy position setpoints.
   */
  public Command setIntakeDeploySetpointsC(double intakeRpm, double deployPositionRotations) {
    return runOnce(() -> setIntakeDeploySetpoints(intakeRpm, deployPositionRotations));
  }

  /**
   * Creates a command to set intake speed and a named deploy position.
   *
   * @param intakeRpm Target speed in RPM for the intake motor.
   * @param deployPosition Named deploy target.
   * @return Command that updates intake speed and deploy position setpoints.
   */
  public Command setIntakeDeploySetpointsC(double intakeRpm, DeployPosition deployPosition) {
    return runOnce(() -> setIntakeDeploySetpoints(intakeRpm, deployPosition));
  }

  /**
   * Runs the shoe servo forward.
   */
  public void shoeForward() {
    m_shoeServo.set(IntakeConstants.Shoe.kForward);
    NCDebug.Debug.debug("Intake: Shoe Forward");
  }

  /**
   * Creates a command to run the shoe servo forward.
   *
   * @return Command that runs the shoe servo forward.
   */
  public Command shoeForwardC() {
    return runOnce(this::shoeForward);
  }

  /**
   * Runs the shoe servo in reverse.
   */
  public void shoeReverse() {
    m_shoeServo.set(IntakeConstants.Shoe.kReverse);
    NCDebug.Debug.debug("Intake: Shoe Reverse");
  }

  /**
   * Creates a command to run the shoe servo in reverse.
   *
   * @return Command that runs the shoe servo in reverse.
   */
  public Command shoeReverseC() {
    return runOnce(this::shoeReverse);
  }

  /**
   * Stops the shoe servo.
   */
  public void shoeStop() {
    m_shoeServo.set(IntakeConstants.Shoe.kStop);
    NCDebug.Debug.debug("Intake: Shoe Stop");
  }

  /**
   * Creates a command to stop the shoe servo.
   *
   * @return Command that stops the shoe servo.
   */
  public Command shoeStopC() {
    return runOnce(this::shoeStop);
  }

  /**
   * Seeds the deploy motor position from the deploy CANcoder absolute position.
   * This aligns the fused CANcoder feedback position at startup.
   */
  public void seedDeployMotorPositionFromCANcoder() {
    double absRot = getDeployAbsolutePositionRotations();
    m_deployMotor.setPosition(absRot);
    NCDebug.Debug.debug("Intake: Seed deploy position from CANcoder to " + absRot + " rotations");
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
