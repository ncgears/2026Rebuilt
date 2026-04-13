package frc.robot.classes;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.VisionConstants;

import java.util.HashMap;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

public class Vision extends Thread {
	private static Vision instance;

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public static PhotonCamera frontCam, backCam, leftCam, rightCam;
	public static boolean frontMultitag, backMultitag, leftMultitag, rightMultitag = false;

	PhotonPoseEstimator frontPhotonPoseEstimator;
	PhotonPoseEstimator backPhotonPoseEstimator;
	PhotonPoseEstimator leftPhotonPoseEstimator;
	PhotonPoseEstimator rightPhotonPoseEstimator;

	Optional<EstimatedRobotPose> resultFront;
	Optional<EstimatedRobotPose> resultBack;
	Optional<EstimatedRobotPose> resultLeft;
	Optional<EstimatedRobotPose> resultRight;

	boolean useVision = VisionConstants.kUseVisionForPose;
	double frontLastTimeStamp = 0;
	double backLastTimeStamp = 0;
	double leftLastTimeStamp = 0;
	double rightLastTimeStamp = 0;

	double visionRatio = 10;

	public static HashMap<PhotonCamera, Alert> connectedVisionAlerts = new HashMap<>();
	public static HashMap<PhotonCamera, Alert> wasDisconnectedVisionAlerts = new HashMap<>();

	public static void createAlert(PhotonCamera device, String deviceName) {
		Alert isAlert = new Alert("Vision Subsystem", deviceName + ": is disconnected", AlertType.kError);
		connectedVisionAlerts.put(device, isAlert);
		Alert wasAlert = new Alert("Vision Subsystem", deviceName + ": was disconnected", AlertType.kError);
		wasDisconnectedVisionAlerts.put(device, wasAlert);
	}

	public Vision() {
		super();
		aprilTagFieldLayout = VisionConstants.kTagLayout;

		frontCam = new PhotonCamera(VisionConstants.Front.kCameraName);
		frontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Front.kRobotToCam);
		// frontPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		backCam = new PhotonCamera(VisionConstants.Back.kCameraName);
		backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Back.kRobotToCam);
		// backPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		leftCam = new PhotonCamera(VisionConstants.Left.kCameraName);
		leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Left.kRobotToCam);
		// leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		rightCam = new PhotonCamera(VisionConstants.Right.kCameraName);
		rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Right.kRobotToCam);
		// rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		setVisionWeights(.2, .2, 10);

		createAlert(frontCam, "frontCam");
		createAlert(backCam, "backCam");
		createAlert(leftCam, "leftCam");
		createAlert(rightCam, "rightCam");
	}

	/**
	 * Returns the instance of the class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return instance of this class
	 */
  	public static Vision getInstance() {
		if (instance == null)
			instance = new Vision();
		return instance;
	}

	public void updateDashboards() {
    if (!GlobalConstants.telemetryAtLeast(VisionConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
    // INFO level telemetry goes here

    boolean frontSuppressed = false;
    boolean backSuppressed = false;
    boolean leftSuppressed = false;
    boolean rightSuppressed = false;
    if (RobotContainer.drivetrain != null) {
      frontSuppressed = RobotContainer.drivetrain.isFrontVisionSuppressed();
      backSuppressed = RobotContainer.drivetrain.isBackVisionSuppressed();
      leftSuppressed = RobotContainer.drivetrain.isLeftVisionSuppressed();
	  rightSuppressed = RobotContainer.drivetrain.isRightVisionSuppressed();
    }

	SmartDashboard.putBoolean("/Vision/Front/Connected", frontCam.isConnected());
	SmartDashboard.putBoolean("/Vision/Front/MultiTag", isFrontMultitag());

	SmartDashboard.putBoolean("/Vision/Back/Connected", backCam.isConnected());
	SmartDashboard.putBoolean("/Vision/Back/MultiTag", isBackMultitag());

	SmartDashboard.putBoolean("/Vision/Left/Connected", leftCam.isConnected());
	SmartDashboard.putBoolean("/Vision/Left/MultiTag", isLeftMultitag());

	SmartDashboard.putBoolean("/Vision/Right/Connected", rightCam.isConnected());
	SmartDashboard.putBoolean("/Vision/Right/MultiTag", isRightMultitag());

    // SmartDashboard.putBoolean("Subsystems/Vision/Front/HasTargets", m_frontHasTargets);
    // SmartDashboard.putBoolean("Subsystems/Vision/Back/HasTargets", m_backHasTargets);
    // SmartDashboard.putBoolean("Subsystems/Vision/Shooter/HasTargets", m_shooterHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/Suppressed", frontSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/Suppressed", backSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Left/Suppressed", leftSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Right/Suppressed", rightSuppressed);
    // SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedAmbiguity", m_frontRejectedAmbiguity);
    // SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedAmbiguity", m_backRejectedAmbiguity);
    // SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedAmbiguity", m_shooterRejectedAmbiguity);
    // SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedConsistency", m_frontRejectedConsistency);
    // SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedConsistency", m_backRejectedConsistency);
    // SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedConsistency", m_shooterRejectedConsistency);
    // SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedJump", m_frontRejectedJump);
    // SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedJump", m_backRejectedJump);
    // SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedJump", m_shooterRejectedJump);
    // SmartDashboard.putBoolean("Subsystems/Vision/Front/WarmupBypassActive", m_frontWarmupBypassActive);
    // SmartDashboard.putBoolean("Subsystems/Vision/Back/WarmupBypassActive", m_backWarmupBypassActive);
    // SmartDashboard.putBoolean("Subsystems/Vision/Shooter/WarmupBypassActive", m_shooterWarmupBypassActive);

    if (!GlobalConstants.telemetryAtLeast(VisionConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
    // DEBUG level telemetry goes here

    // SmartDashboard.putNumber("Subsystems/Vision/Front/PoseX", m_visFrontPose.getX());
    // SmartDashboard.putNumber("Subsystems/Vision/Front/PoseY", m_visFrontPose.getY());
    // SmartDashboard.putNumber("Subsystems/Vision/Shooter/PoseX", m_visShooterPose.getX());
    // SmartDashboard.putNumber("Subsystems/Vision/Shooter/PoseY", m_visShooterPose.getY());
    // SmartDashboard.putNumber("Subsystems/Vision/Back/PoseX", m_visBackPose.getX());
    // SmartDashboard.putNumber("Subsystems/Vision/Back/PoseY", m_visBackPose.getY());
    // SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseX", RobotContainer.drivetrain.getBotPose().getX());
    // SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseY", RobotContainer.drivetrain.getBotPose().getY());
    // SmartDashboard.putNumber("Subsystems/Vision/Front/LastTimestamp", lastEstTimestampFront);
    // SmartDashboard.putNumber("Subsystems/Vision/Shooter/LastTimestamp", lastEstTimestampShooter);
    // SmartDashboard.putNumber("Subsystems/Vision/Back/LastTimestamp", lastEstTimestampBack);
  }

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedFrontGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : frontCam.getAllUnreadResults()) {
			frontMultitag = change.getMultiTagResult().isPresent();
			visEst = frontPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedBackGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : backCam.getAllUnreadResults()) {
			backMultitag = change.getMultiTagResult().isPresent();
			visEst = backPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedLeftGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : leftCam.getAllUnreadResults()) {
			leftMultitag = change.getMultiTagResult().isPresent();
			visEst = leftPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedRightGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : rightCam.getAllUnreadResults()) {
			rightMultitag = change.getMultiTagResult().isPresent();
			visEst = rightPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	public boolean isFrontMultitag() { return frontMultitag; }
	public boolean isBackMultitag() { return backMultitag; }
	public boolean isLeftMultitag() { return leftMultitag; }
	public boolean isRightMultitag() { return rightMultitag; }

	public void setVisionWeights(double visionX, double visionY, double visionDeg) {
		// RobotContainer.driveSubsystem
		// .setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY,
		// Units.degreesToRadians(visionDeg)));
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[] { pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians() });
	}

	Alliance lastAlliance = null;

	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		// RobotContainer.driveSubsystem.addVisionMeasurement(pose, timestampSeconds,
		// weights);
	}

	public Matrix<N3, N1> getVisionWeights(double distanceAverage, int numTargets) {
		double targetMultiplier = 0.5;
		double visionCutOffDistance = 4;
		double distanceRatio = 0.1466 * Math.pow(1.6903, distanceAverage);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[] { 99999, 99999, 99999 }));
			}
			targetMultiplier = 5;
		}
		double dev = distanceRatio * targetMultiplier;
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[] { dev,
				dev, dev * 2 }));
		return weights;
	}

	@Override
	@SuppressWarnings("unused")
	public void run() {
		/* Run as fast as possible, our signals will control the timing */
		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// Vision Calculations
			// This is not required, since we design everything to use blue origin. 
			// The targeting class presents mirrored or rotated poses, depending on the needs of the season.
			// if (DriverStation.getAlliance().isPresent()) {
			// 	if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
			// 		lastAlliance = Alliance.Blue;
			// 		aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
			// 	}
			// 	if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {

			// 		lastAlliance = Alliance.Red;
			// 		aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
			// 	}
			// }
			// Always present blue origin
			aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

			frontPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultFront = getEstimatedFrontGlobalPose();
			this.resultBack = getEstimatedBackGlobalPose();
			this.resultLeft = getEstimatedLeftGlobalPose();
			this.resultRight = getEstimatedRightGlobalPose();

			if (useVision) {
				if (VisionConstants.Front.kUseForPose && resultFront.isPresent()) {
					EstimatedRobotPose camPoseFront = resultFront.get();
					double frontTimeStamp = camPoseFront.timestampSeconds;
					if (frontTimeStamp > Timer.getFPGATimestamp()) {
						frontTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseFront.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFront.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseFront.targetsUsed.size();
					double distanceAverage = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceAverage, camPoseFront.targetsUsed.size());

					if (frontTimeStamp != frontLastTimeStamp) {
						publishPose2d("/DriveTrain/FrontCamPose", camPoseFront.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/FrontWeights", weights.toString());
						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseFront.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(frontTimeStamp), weights);

					}
					frontLastTimeStamp = frontTimeStamp;
				}

				if (VisionConstants.Back.kUseForPose && resultBack.isPresent()) {
					EstimatedRobotPose camPoseBack = resultBack.get();
					double backTimestamp = camPoseBack.timestampSeconds;

					if (backTimestamp > Timer.getFPGATimestamp()) {
						backTimestamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseBack.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBack.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseBack.targetsUsed.size();
					double distanceAverage = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceAverage, camPoseBack.targetsUsed.size());

					if (backTimestamp != backLastTimeStamp) {
						publishPose2d("/DriveTrain/BackCamPose", camPoseBack.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackWeights", weights.toString());

						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseBack.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(backTimestamp),
								weights);
					}
					backLastTimeStamp = backTimestamp;
				}

				if (VisionConstants.Left.kUseForPose && resultLeft.isPresent()) {
					EstimatedRobotPose camPoseLeft = resultLeft.get();
					double leftTimeStamp = camPoseLeft.timestampSeconds;

					if (leftTimeStamp > Timer.getFPGATimestamp()) {
						leftTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseLeft.targetsUsed.size();
					double distanceAverage = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceAverage, camPoseLeft.targetsUsed.size());

					if (leftTimeStamp != leftLastTimeStamp) {
						publishPose2d("/DriveTrain/LeftCamPose", camPoseLeft.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/LeftWeights", weights.toString());

						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseLeft.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(leftTimeStamp),
								weights);
					}
					leftLastTimeStamp = leftTimeStamp;
				}

				if (VisionConstants.Right.kUseForPose && resultRight.isPresent()) {
					EstimatedRobotPose camPoseRight = resultRight.get();
					double rightTimeStamp = camPoseRight.timestampSeconds;

					if (rightTimeStamp > Timer.getFPGATimestamp()) {
						rightTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultRight.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseRight.targetsUsed.size();
					double distanceAverage = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceAverage, camPoseRight.targetsUsed.size());

					if (rightTimeStamp != rightLastTimeStamp) {
						publishPose2d("/DriveTrain/RightCamPose", camPoseRight.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/RightWeights", weights.toString());

						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseRight.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(rightTimeStamp),
								weights);
					}
					rightLastTimeStamp = rightTimeStamp;
				}

			}
		}
	}
}