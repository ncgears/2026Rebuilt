package frc.robot.classes;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*;
import frc.robot.utils.NCDebug;
import frc.robot.RobotContainer;

/**
 * The NCPose class handles getting and managing the different poses and calculations of them.
 * It is responsible for maintaining the robot poses and exposing methods to calculate items related to the poses
 */
public class Targeting {
	private static Targeting instance;

	/**
	 * Targets represents different locations on the field that we might be interested in tracking
	 */
	private static final double m_fieldLength = VisionConstants.kTagLayout.getFieldLength();
	private static final double m_fieldWidth = VisionConstants.kTagLayout.getFieldWidth();

	public enum Targets { //based on blue origin 0,0 (blue driver station, right corner)
		HUB(4.65, 4.05, 2.25, 0),
		POCKET_LEFT(3.0, 6.0, 0.0, 0),
		POCKET_RIGHT(3.0, 2.0, 0.0, 0),
		TOWER_LEFT(1.0, 5.5, 0.0, 180),
		TOWER_RIGHT(1.0, 4.2, 0.0, 0);

		private final double x, y, z, angle;

		Targets(double x, double y, double z, double angle) {
			this.x = x;
			this.y = y;
			this.z = z;
			this.angle = angle;
		}

		/**
		 * Returns the target angle in field coordinates without alliance transforms.
		 *
		 * @return Raw target angle.
		 */
		public Rotation2d getRawAngle() {
			return Rotation2d.fromDegrees(this.angle);
		}

		/**
		 * Returns the target angle adjusted for alliance perspective.
		 *
		 * @param redOrigin True when the origin is red alliance.
		 * @return Alliance-adjusted angle.
		 */
		public Rotation2d getAngle(boolean redOrigin) {
			return (redOrigin) ? getRawAngle() : getRawAngle().rotateBy(Rotation2d.k180deg);
		}

		/**
		 * Returns the mirrored target angle across the field.
		 *
		 * @param redOrigin True when the origin is red alliance.
		 * @return Mirrored angle.
		 */
		public Rotation2d getMirrorAngle(boolean redOrigin) {
			return (redOrigin) ? getRawAngle().unaryMinus() : getRawAngle().unaryMinus().rotateBy(Rotation2d.k180deg);
		}

		/**
		 * Returns the 3D pose of this target.
		 *
		 * @return Pose for this target.
		 */
		public Pose3d getPose() {
			return new Pose3d(
				new Translation3d(this.x, this.y, this.z),
				new Rotation3d()
				// new Rotation3d(0,0,Math.toRadians(this.angle))
			);
		}

		/**
		 * Returns the mirrored 3D pose across the field length.
		 *
		 * @return Mirrored pose for this target.
		 */
		public Pose3d getMirrorPose() {
			return new Pose3d(
				new Translation3d(m_fieldLength - this.x, this.y, this.z),
				new Rotation3d()
				// new Rotation3d(0,0,Math.PI - Math.toRadians(this.angle))
			);
		}

		/**
		 * Returns the rotated 3D pose for the opposite side of the field using
		 * 180-degree rotational symmetry.
		 *
		 * @return Rotated pose for this target.
		 */
		public Pose3d getRotatedPose() {
			return new Pose3d(
				new Translation3d(m_fieldLength - this.x, m_fieldWidth - this.y, this.z),
				new Rotation3d()
				// new Rotation3d(0,0,Math.PI + Math.toRadians(this.angle))
			);
		}
	}

	/** State represents different tracking system states */
	public enum State {
		READY(DashboardConstants.Colors.GREEN),
		TRACKING(DashboardConstants.Colors.ORANGE),
		ERROR(DashboardConstants.Colors.RED),
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

	private State m_trackingState = State.STOP; //current Tracking state
	private Targets m_trackingTarget = Targets.HUB; //current Tracking target
	// private Pose3d m_shooterPose = new Pose3d();
	// private boolean m_adjustUp = false;
	public final Trigger isTracking = new Trigger(() -> {
		return (m_trackingState == State.READY || m_trackingState == State.TRACKING);
	});
	public final Trigger isReady = new Trigger(() -> {
		return (m_trackingState == State.READY);
	});
	public final PIDController thetaController = new PIDController(
		ThetaConstants.kP,
		ThetaConstants.kI,
		ThetaConstants.kP,
		ThetaConstants.kIZone
	);

	/** Creates the targeting helper and initializes pose state. */
	public Targeting() {
		init();
		createDashboards();
	}

	/** Resets tracking state and initializes the starting pose. */
	public void init() {
		m_trackingState = State.STOP;
		m_trackingTarget = Targets.HUB;
		resetPose(
			(RobotContainer.isAllianceRed()) //more realistic starting position, center on black line
				? new Pose2d(m_fieldLength - 3.978, m_fieldWidth - 7.4279, Rotation2d.k180deg)
				: new Pose2d(3.978, 7.4279, Rotation2d.kZero)
		);
		NCDebug.Debug.debug("Targeting: Initialized");
	}

	/**
	 * Returns the instance of the class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return instance of this class
	 */
	public static Targeting getInstance() {
		if (instance == null) {
			instance = new Targeting();
		}
		return instance;
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Supplier<Pose2d> getPose() {
		return () -> RobotContainer.drivetrain.getState().Pose;
	}

	/**
	 * Returns the robot heading rotated for the shooter perspective.
	 *
	 * @return Robot heading as a Rotation2d.
	 */
	public Rotation2d getRobotHeading() {
		return getPose().get().getRotation().rotateBy(new Rotation2d(Math.PI));
	}

	/**
	 * Returns the bearing to a target from the robot.
	 *
	 * @param target Target to evaluate.
	 * @return Bearing in degrees.
	 */
	public double getBearingOfTarget(Targets target) {
		Pose2d robotPose = getPose().get();
		Pose2d targetPose = (RobotContainer.isAllianceRed()) ? target.getRotatedPose().toPose2d() : target.getPose().toPose2d();
		Translation2d targetVector = targetPose.getTranslation().minus(robotPose.getTranslation());
		if (targetVector.getNorm() < 1e-6) {
			return robotPose.getRotation().getDegrees();
		}
		return targetVector.getAngle().getDegrees();
	}

	/**
	 * Returns the distance to a target from the robot.
	 *
	 * @param target Target to evaluate.
	 * @return Distance in meters.
	 */
	public double getDistanceOfTarget(Targets target) {
		Pose2d robotPose = getPose().get();
		Pose2d targetPose = (RobotContainer.isAllianceRed()) ? target.getRotatedPose().toPose2d() : target.getPose().toPose2d();
		return robotPose.getTranslation().getDistance(targetPose.getTranslation());
	}

	/**
	 * Returns the field-relative angle to a target.
	 *
	 * @param target Target to evaluate.
	 * @return Angle to target.
	 */
	public Rotation2d getAngleOfTarget(Targets target) {
		boolean red = RobotContainer.isAllianceRed();
		Rotation2d perspective = (red) ? Rotation2d.k180deg : Rotation2d.kZero;
		return (red) ? target.getAngle(red).minus(perspective) : target.getMirrorAngle(red).unaryMinus().minus(perspective);
		// return target.getRawAngle();
	}

	// /**
	//  * Reset the estimated pose of the swerve drive on the field.
	//  *
	//  * @param heading Heading to reset robot to (for configuring a yaw offset)
	//  * @param pose New robot pose.
	//  */
	// public void resetPose(double heading, Pose2d pose) {
	// 	poseEstimator.resetPosition(RobotContainer.gyro.getYaw(), RobotContainer.drivetrain.getSwerveModulePositions(), pose);
	// }
	/**
	 * Reset the estimated pose of the swerve drive on the field.
	 *
	 * @param pose New robot pose.
	 */
	public void resetPose(Pose2d pose) {
		RobotContainer.drivetrain.resetPose(pose);
	}

	////#region "Tracking"
	/** Creates the dashboard for the tracking system */
	public void createDashboards() {
		// if(true) { //false to disable tracking dashboard
		ShuffleboardTab systemTab = Shuffleboard.getTab("System");
		// 	systemTab.addNumber("Bot Pose Hdg", () -> NCDebug.General.roundDouble(getPose().get().getRotation().getDegrees(),2))
		// 		.withSize(4,2)
		// 		.withPosition(0,4);
		// 	// systemTab.addNumber("Shooter Hdg", () -> NCDebug.General.roundDouble(getPose().get().rotateBy(new Rotation2d(Math.PI)).getRotation().getDegrees(),2))
		// 	// 	.withSize(4,2)
		// 	// 	.withPosition(4,2);
		ShuffleboardLayout trackingList = systemTab.getLayout("Target Tracking", BuiltInLayouts.kList)
			.withSize(4, 5)
			.withPosition(12, 5)
			.withProperties(Map.of("Label position", "LEFT"));
		trackingList.addString("Tracking", this::getTrackingStateColor)
			.withWidget("Single Color View");
		trackingList.addString("State", this::getTrackingStateName)
			.withWidget("Text Display");
		trackingList.addString("Target", this::getTrackingTargetName)
			.withWidget("Text Display");
		trackingList.addNumber("Bearing", this::getTrackingTargetBearing);
		trackingList.addNumber("Distance", this::getTrackingTargetDistance);
		trackingList.addNumber("Angle", this::getTrackingTargetAngle);
		// }
	}

	/** Determines if the robot should be tracking a target
	 * @return boolean indicating if robot is tracking
	 */
	public boolean getTracking() {
		return m_trackingState != State.STOP;
	}

	/** Gets the current target tracking state */
	public State getTrackingState() {
		return m_trackingState;
	}

	/** Gets the name of the current target tracking state */
	public String getTrackingStateName() {
		return m_trackingState.toString();
	}

	/** Gets the defined color of the current target tracking state */
	public String getTrackingStateColor() {
		return m_trackingState.getColor();
	}

	/** Gets the current tracking target */
	public Targets getTrackingTarget() {
		return m_trackingTarget;
	}

	/** Gets the name of the current tracking target */
	public String getTrackingTargetName() {
		return m_trackingTarget.toString();
	}

	/** Gets the relative bearing of the current tracking target */
	public double getTrackingTargetBearing() {
		return NCDebug.General.roundDouble(getBearingOfTarget(m_trackingTarget), 2);
	}

	/** Gets the relative distance from the shooter to the current tracking target */
	public double getTrackingTargetDistance() {
		return NCDebug.General.roundDouble(getDistanceOfTarget(m_trackingTarget), 2);
	}

	/** Gets the relative angle from the shooter to the current tracking target */
	public double getTrackingTargetAngle() {
		return NCDebug.General.roundDouble(getAngleOfTarget(m_trackingTarget).getDegrees(), 2);
	}

	/** Gets the relative angle from the shooter to the current tracking target as rotations from 0.0 */
	public double getTrackingTargetAngleAsRotations() {
		return getAngleOfTarget(m_trackingTarget).getRotations();
	} //.plus(getGravityAdjustmentOfTarget(m_trackingTarget))

	/** Enables target tracking */
	public void trackingStart() {
		m_trackingState = State.TRACKING;
		NCDebug.Debug.debug("Tracking: Start Tracking (" + m_trackingTarget.toString() + ")");
	}

	public Command trackingStartC() {
		return new InstantCommand(() -> trackingStart());
	}

	/** Disables target tracking */
	public void trackingStop() {
		m_trackingState = State.STOP;
		// RobotContainer.drivetrain.lockHeading();
		NCDebug.Debug.debug("Tracking: Stop Tracking");
	}

	public Command trackingStopC() {
		return new InstantCommand(() -> trackingStop());
	}

	/** Sets the requested tracking target
	 * @param target Target to track
	 */
	public void setTrackingTarget(Targets target) {
		m_trackingTarget = target;
		NCDebug.Debug.debug("Tracking: Set Tracking Target (" + target.toString() + ")");
	}

	/** Sets the tracking target to Hub (2026 Rebuilt) */
	public Command setTrackingHubC() {
		return new InstantCommand(() -> setTrackingTarget(Targets.HUB));
	}

	/** Sets the tracking target to Pocket Left (2026 Rebuilt) */
	public Command setTrackingPocketLeftC() {
		return new InstantCommand(() -> setTrackingTarget(Targets.POCKET_LEFT));
	}

	/** Sets the tracking target to Pocket Right (2026 Rebuilt) */
	public Command setTrackingPocketRightC() {
		return new InstantCommand(() -> setTrackingTarget(Targets.POCKET_RIGHT));
	}

	/**
	 * Updates the tracking state to ready or tracking.
	 *
	 * @param ready True when the robot is ready to track.
	 */
	public void setTrackingReady(boolean ready) {
		m_trackingState = (ready) ? State.READY : State.TRACKING;
	}
	////#endregion "Tracking"
}
