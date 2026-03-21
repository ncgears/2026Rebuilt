package frc.robot.classes;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;

public class Gyro implements Sendable {
	private static Gyro instance;
	private static double m_yawOffset = 0.0;
  private boolean m_dashboardRegistered = false;

	private static Pigeon2 m_gyro = new Pigeon2(GyroConstants.kPigeonID, new CANBus(GyroConstants.kCANbus));

	/**
	 * Returns the instance of the Gyro subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return GyroSubSystem instance
	 */
    public static Gyro getInstance() {
		if (instance == null)
			instance = new Gyro();
		return instance;
	}

	/**
	 * Initializes the GyroSubsystem class, performs setup steps, etc.
	 */
    public Gyro() {
    }

	/**
	 * Publishes gyro telemetry to SmartDashboard.
	 * This method exists for backward compatibility with existing dashboard setup calls.
	 */
	public void buildDashboards() {
		updateDashboards();
	}

	/**
	 * Publishes current gyro values to SmartDashboard.
	 */
	public void updateDashboards() {
		if (!GlobalConstants.telemetryAtLeast(GyroConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
		// INFO level telemetry goes here

		if (!m_dashboardRegistered) {
			SmartDashboard.putData("Subsystems/Gyro/Widget", this);
			m_dashboardRegistered = true;
		}
		SmartDashboard.putNumber("Subsystems/Gyro/YawDegrees", getYaw().getDegrees());
		SmartDashboard.putNumber("Subsystems/Gyro/PitchDegrees", getPitch());
		SmartDashboard.putNumber("Subsystems/Gyro/RollDegrees", getRoll());
		SmartDashboard.putNumber("Subsystems/Gyro/YawOffsetDegrees", m_yawOffset);

		if (!GlobalConstants.telemetryAtLeast(GyroConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
		// DEBUG level telemetry goes here
	}

	/**
	 * Configures this gyro as a Sendable for dashboards.
	 *
	 * @param builder Sendable builder instance.
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.setActuator(false);
		builder.addDoubleProperty("Value", () -> getYaw().getDegrees(), null);
	}

	/** Zeros the gyro yaw and clears the yaw offset. */
    public void zeroYaw() {
		// m_gyro.reset();
		m_gyro.setYaw(0,0.1);
		m_gyro.getYaw().waitForUpdate(0.1);
		m_yawOffset = 0.0;
		NCDebug.Debug.debug("Gyro: Reset Gyro");
	}

	/**
	 * Zeros the gyro yaw and applies a new offset.
	 *
	 * @param offset Offset in degrees to apply after zeroing.
	 */
    public void zeroYaw(double offset) {
		zeroYaw();
		setYawOffset(offset);
	}

	/**
	 * Sets the current yaw offset.
	 *
	 * @param offset Offset in degrees.
	 */
	public void setYawOffset(double offset) {
		m_yawOffset = offset;
		NCDebug.Debug.debug("Gyro: Yaw offset set to "+offset+" degrees");
	}

   	/**
     * Returns the yaw/heading of the robot.
     * @return the robot's yaw as a Rotation2d
     */
	public Rotation2d getYaw() {
		// return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
		double yaw = m_gyro.getYaw().getValueAsDouble() - m_yawOffset; //subtract the offset
		yaw += (yaw < 0) ? 360.0 : 0; //make it positive
		yaw *= (GyroConstants.kGyroReversed) ? -1.0 : 1.0; //invert to CCW Positive
		return Rotation2d.fromDegrees(yaw);
	}

   	/**
     * Returns the pitch of the robot.
     * @return the robot's pitch as a double in degrees
     */
	public double getPitch() {
		return m_gyro.getPitch().getValueAsDouble();
	}

   	/**
     * Returns the roll of the robot.
     * @return the robot's roll as a double in degrees
     */
	public double getRoll() {
		return m_gyro.getRoll().getValueAsDouble();
	}
}
