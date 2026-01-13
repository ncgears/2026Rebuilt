
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.*;
import frc.robot.classes.Lighting;
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * This subsystem handles managing the Algae.
 * It is responsible for running the Algae using information from the indexer
 * about whether we have a note.
 */
@SuppressWarnings({ "unused" })
public class AlgaeSubsystem extends SubsystemBase {
  private static AlgaeSubsystem instance;

  // private and public variables defined here
  // #region Declarations
  public enum Direction {
    OUT(DashboardConstants.Colors.ORANGE),
    IN(DashboardConstants.Colors.GREEN),
    HOLD(DashboardConstants.Colors.BLACK),
    STOP(DashboardConstants.Colors.BLACK);

    private final String color;

    Direction(String color) {
      this.color = color;
    }

    /**
     * Returns the dashboard color for this direction.
     *
     * @return Hex color string.
     */
    public String getColor() {
      return this.color;
    }
  }

  private Direction m_curDirection = Direction.STOP;

  public enum Position {
    DOWN(AlgaeConstants.wrist.Positions.down, DashboardConstants.Colors.RED),
    FLOOR(AlgaeConstants.wrist.Positions.floor, DashboardConstants.Colors.RED),
    PROC(AlgaeConstants.wrist.Positions.proc, DashboardConstants.Colors.ORANGE),
    REEF(AlgaeConstants.wrist.Positions.reef, DashboardConstants.Colors.BLUE),
    UP(AlgaeConstants.wrist.Positions.up, DashboardConstants.Colors.GREEN),
    STOW(AlgaeConstants.wrist.Positions.stow, DashboardConstants.Colors.BLACK);

    private final double position;
    private final String color;

    Position(double position, String color) {
      this.position = position;
      this.color = color;
    }

    /**
     * Returns the target wrist position in rotations.
     *
     * @return Target rotations.
     */
    public double getRotations() {
      return this.position;
    }

    /**
     * Returns the dashboard color for this position.
     *
     * @return Hex color string.
     */
    public String getColor() {
      return this.color;
    }
  }

  private Position m_curPosition = Position.STOW;

  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final PositionVoltage m_posVoltage = new PositionVoltage(0);
  // private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  private CANcoder m_encoder;
  private TalonFX m_wristmotor1;
  private TalonFXS m_toro_left, m_toro_right;
  // #endregion Declarations

  // #region Triggers
  public final Trigger isRunning = new Trigger(() -> {
    return (m_curDirection != Direction.STOP);
  });
  // #endregion Triggers

  // #region Setup
  /**
   * Returns the instance of the AlgaeSubsystem subsystem.
   * The purpose of this is to only create an instance if one does not already
   * exist.
   * 
   * @return AlgaeSubsystem instance
   */
  public static AlgaeSubsystem getInstance() {
    if (instance == null)
      instance = new AlgaeSubsystem();
    return instance;
  }

  /** Creates the algae subsystem and configures hardware. */
  public AlgaeSubsystem() {
    // m_motor1 = new TalonFXS(AlgaeConstants.kMotorID,AlgaeConstants.kCANBus);
    // TalonFXSConfigurator m_config = m_motor1.getConfigurator();
    // TalonFXSConfiguration m_fxsConfigs = new TalonFXSConfiguration();
    // m_fxsConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // m_config.apply(m_fxsConfigs);

    // initialize values for private and public variables, etc.
    CANBus algaeCANBus = new CANBus(AlgaeConstants.canBus);
    m_encoder = new CANcoder(AlgaeConstants.kCANcoderID, algaeCANBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.algaeCCConfig));

    m_wristmotor1 = new TalonFX(AlgaeConstants.wrist.kMotorID, algaeCANBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_wristmotor1.getConfigurator().apply(RobotContainer.ctreConfigs.algaewristFXConfig));

    m_toro_left = new TalonFXS(AlgaeConstants.left.kMotorID, algaeCANBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_toro_left.getConfigurator().apply(RobotContainer.ctreConfigs.algaeleftFXSConfig));
    m_toro_right = new TalonFXS(AlgaeConstants.right.kMotorID, algaeCANBus);
    RobotContainer.ctreConfigs
      .retryConfigApply(() -> m_toro_right.getConfigurator().apply(RobotContainer.ctreConfigs.algaerightFXSConfig));

    init();
    createDashboards();
  }

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    // AlgaeBrake();
    setPosition(Position.STOW);
    stopToro();
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Algae: Initialized");
  }

  /** Runs periodic updates for the algae subsystem. */
  @Override
  public void periodic() {
    m_wristmotor1.setControl(m_posVoltage.withPosition(m_curPosition.getRotations()));
  }
  // #endregion Setup

  // #region Dashboard
  /** Creates Shuffleboard widgets for the algae subsystem. */
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Algae", this::getDirectionColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(2, 7);
    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout AlgaeList = systemTab.getLayout("Algae", BuiltInLayouts.kList)
      .withSize(4, 4)
      .withPosition(8, 0)
      .withProperties(Map.of("Label position", "LEFT"));
    AlgaeList.addString("Status", this::getDirectionColor)
      .withWidget("Single Color View");
    AlgaeList.addString("Direction", this::getDirectionName);
    AlgaeList.addString("Position", this::getPositionName);
    AlgaeList.addNumber("Target Pos", this::getTargetPosition);
    AlgaeList.addNumber("Motor Pos", () -> {
      return NCDebug.General.roundDouble(getMotorPosition().in(Units.Rotations), 6);
    });

    if (AlgaeConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgAlgaeList = debugTab.getLayout("Algae", BuiltInLayouts.kList)
        .withSize(4, 11)
        .withPosition(12, 0)
        .withProperties(Map.of("Label position", "LEFT"));
      dbgAlgaeList.addString("Status", this::getDirectionColor)
        .withWidget("Single Color View");
      dbgAlgaeList.addString("Direction", this::getDirectionName);
      dbgAlgaeList.addString("Target", this::getTargetPositionName);
      dbgAlgaeList.addNumber("Target Pos", this::getTargetPosition);
      dbgAlgaeList.addNumber("Motor Pos", () -> {
        return NCDebug.General.roundDouble(getMotorPosition().in(Units.Rotations), 6);
      });
      dbgAlgaeList.addNumber("Absolute", () -> {
        return NCDebug.General.roundDouble(getPositionAbsolute().in(Units.Rotations), 6);
      });
      dbgAlgaeList.addNumber("L Toro Spd", this::getToroLeftSpeed);
      dbgAlgaeList.addNumber("R Toro Spd", this::getToroRightSpeed);
      // dbgAlgaeList.add("Algae In", new InstantCommand(this::AlgaeIn))
      // .withProperties(Map.of("show_type",false));
      // dbgAlgaeList.add("Algae Out", new InstantCommand(this::AlgaeOut))
      // .withProperties(Map.of("show_type",false));
      // dbgAlgaeList.add("Algae Stop", new InstantCommand(this::AlgaeStop))
      // .withProperties(Map.of("show_type",false));
    }
  }
  // #endregion Dashboard

  // #region Getters
  /**
   * Returns the current intake direction.
   *
   * @return Current direction.
   */
  public Direction getDirection() {
    return m_curDirection;
  }

  /**
   * Returns the current direction name.
   *
   * @return Direction name.
   */
  public String getDirectionName() {
    return m_curDirection.toString();
  }

  /**
   * Returns the current direction color.
   *
   * @return Hex color string.
   */
  public String getDirectionColor() {
    return m_curDirection.getColor();
  }

  /**
   * Returns the current wrist position.
   *
   * @return Current position.
   */
  public Position getPosition() {
    return m_curPosition;
  }

  /**
   * Returns the current position name.
   *
   * @return Position name.
   */
  public String getPositionName() {
    return m_curPosition.toString();
  }

  /**
   * Returns the current position color.
   *
   * @return Hex color string.
   */
  public String getPositionColor() {
    return m_curPosition.getColor();
  }

  /**
   * Returns the target position name.
   *
   * @return Target position name.
   */
  public String getTargetPositionName() {
    return m_curPosition.toString();
  }

  /**
   * Returns the target wrist position.
   *
   * @return Target position in rotations.
   */
  public double getTargetPosition() {
    return NCDebug.General.roundDouble(m_wristmotor1.getClosedLoopReference().getValue(),6);
  }

  /**
   * Returns the closed-loop position error.
   *
   * @return Position error.
   */
  public double getPositionError() {
    return m_wristmotor1.getClosedLoopError().getValue();
  }

  /**
   * Returns the left Toro velocity.
   *
   * @return Left Toro speed.
   */
  public double getToroLeftSpeed() {
    return m_toro_left.getVelocity().getValueAsDouble();
  }

  /**
   * Returns the right Toro velocity.
   *
   * @return Right Toro speed.
   */
  public double getToroRightSpeed() {
    return m_toro_right.getVelocity().getValueAsDouble();
  }

  /**
   * Returns the wrist motor stator current.
   *
   * @return Stator current in amps.
   */
  private double getStatorCurrent() {
    return m_wristmotor1.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Returns the wrist motor position.
   *
   * @return Motor position.
   */
  public Angle getMotorPosition() {
    return m_wristmotor1.getPosition().getValue();
  }

  /**
   * Returns the absolute wrist position from the encoder.
   *
   * @return Absolute position.
   */
  public Angle getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  /**
   * Returns motors used by this subsystem for orchestra.
   *
   * @return Array of TalonFX motors.
   */
  public TalonFX[] getMotors() {
    TalonFX[] motors = { m_wristmotor1 };
    return motors;
  }
  // #endregion Getters

  // #region Setters
  /**
   * Sets the target wrist position.
   *
   * @param position Target position.
   */
  private void setPosition(Position position) {
    // m_wristmotor1.setControl(m_mmVoltage.withPosition(position.getRotations()));
    // m_wristmotor1.setControl(m_posVoltage.withPosition(position.getRotations()));
    NCDebug.Debug.debug("Algae: Move to " + position.toString());
    m_curPosition = position;
  }

  /**
   * Creates a command to set the wrist position.
   *
   * @param position Target position.
   * @return Command that updates the position.
   */
  public Command setAlgaePositionC(Position position) {
    return runOnce(() -> setPosition(position));
  }

  /**
   * Returns true if the wrist is at the target position.
   *
   * @return True when within tolerance.
   */
  public boolean isAtTarget() {
    return m_wristmotor1.getClosedLoopError().getValueAsDouble() <= AlgaeConstants.wrist.kPositionTolerance;
  }
  // #endregion Setters

  // #region Limits
  /**
   * Returns true if the forward limit is reached.
   *
   * @return True when forward limit is reached.
   */
  public boolean getForwardLimit() {
    return m_wristmotor1.getPosition().getValueAsDouble() >= AlgaeConstants.Positions.kFwdLimit;
  }

  /**
   * Returns true if the reverse limit is reached.
   *
   * @return True when reverse limit is reached.
   */
  public boolean getReverseLimit() {
    return m_wristmotor1.getPosition().getValueAsDouble() <= AlgaeConstants.Positions.kRevLimit;
  }

  /**
   * Returns true if any limit is reached.
   *
   * @return True when at a limit.
   */
  public boolean atLimit() {
    return getForwardLimit() || getReverseLimit();
  }
  // #endregion Limits

  //#region Controls
  /**
   * Creates a command to set wrist neutral output.
   *
   * @return Command that sets neutral output.
   */
  public Command AlgaeNeutral() {
    return runOnce(() -> {m_wristmotor1.setControl(m_neutral); });
  }

  /**
   * Creates a command to brake the wrist motor.
   *
   * @return Command that applies brake.
   */
  public Command AlgaeBrake() {
    return runOnce(() -> {m_wristmotor1.setControl(m_brake); });
  }

  /** Stops the algae intake direction tracking. */
  public void AlgaeStop() {
    // m_wristmotor1.setControl(m_neutral);
    if(m_curDirection != Direction.HOLD) {
      m_curDirection = Direction.STOP;
      NCDebug.Debug.debug("Algae: Stop");
    }
  }

  /**
   * Starts the intake or outtake TORO motors.
   *
   * @param invert True to run outtake direction.
   */
  public void startToro(boolean invert) {
    double speed = (invert) ? -AlgaeConstants.kOuttakeSpeed : AlgaeConstants.kIntakeSpeed;
    m_toro_left.set(speed);
    m_toro_right.set(speed);
    if (invert) {
      RobotContainer.lighting.setColor(Lighting.Colors.WHITE);
      m_curDirection = Direction.OUT;
      NCDebug.Debug.debug("Algae: Start Toro Inverted");
    } else {
      RobotContainer.lighting.setColor(Lighting.Colors.ORANGE);
      m_curDirection = Direction.IN;
      NCDebug.Debug.debug("Algae: Start Toro");
    }
  }

  /** Stops the TORO motors and clears lighting. */
  public void stopToro() {
    m_toro_left.set(0);
    m_toro_right.set(0);
    RobotContainer.lighting.setColor(Lighting.Colors.OFF);
    m_curDirection = Direction.STOP;
    NCDebug.Debug.debug("Algae: Stop Toro");
  }

  /**
   * Creates a command to start the TORO motors.
   *
   * @param invert True to run outtake direction.
   * @return Command that starts TOROs.
   */
  public Command startToroC(boolean invert) {
    return runOnce(() -> startToro(invert));
  }

  /**
   * Creates a command to stop the TORO motors.
   *
   * @return Command that stops TOROs.
   */
  public Command stopToroC() {
    return runOnce(() -> stopToro());
  }
  // #endregion Controls

  // #region SysID Functions
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  // TOROs
  // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
  // new SysIdRoutine.Config(
  // null, //default ramp rate 1V/s
  // Volts.of(4), //reduce dynamic step voltage to 4 to prevent brownout
  // null, //default timeout 10s
  // (state) -> SignalLogger.writeString("SysId_State", state.toString())
  // ),
  // new SysIdRoutine.Mechanism(
  // (volts) -> m_toro_left.setControl(m_voltReq.withOutput(volts.in(Volts))),
  // null,
  // this
  // )
  // );
  // WRIST
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.per(Units.Second).of(0.2), // default ramp rate 1V/s
      Volts.of(1.6), // reduce dynamic step voltage to 4 to prevent brownout
      null, // default timeout 10s
      (state) -> SignalLogger.writeString("SysId_State", state.toString())),
    new SysIdRoutine.Mechanism(
      (volts) -> m_wristmotor1.setControl(m_voltReq.withOutput(volts.in(Volts))),
      null,
      this));

  /**
   * Runs the SysId quasistatic test.
   *
   * @param direction Direction to run.
   * @return Command for the test.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction); // .until(this::atLimit);
  }

  /**
   * Runs the SysId dynamic test.
   *
   * @param direction Direction to run.
   * @return Command for the test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction); // .until(this::atLimit);
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
  // #endregion SysID Functions
}
