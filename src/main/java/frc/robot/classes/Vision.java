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

import java.util.HashMap;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;

public class Vision extends Thread {
	private static Vision instance;

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public static PhotonCamera frontCam;
	Transform3d robotToFrontCam = new Transform3d(new Translation3d(0.27305, 0.1016 , 0.3556),
			new Rotation3d(Math.toRadians(0), Math.toRadians(6), Math.toRadians(0)));
	// Transform3d robotToLeftCam = new Transform3d(new Translation3d(0.18414,
	// 0.27305, 0.26353),
	// new Rotation3d(Math.toRadians(-1), Math.toRadians(9), Math.toRadians(-15)));

	public static PhotonCamera backCam;
	Transform3d robotToBackCam = new Transform3d(new Translation3d(-0.3302, -0.20955 , 0.29845),
			new Rotation3d(Math.toRadians(0), Math.toRadians(3), Math.toRadians(180)));

	PhotonPoseEstimator frontPhotonPoseEstimator;
	PhotonPoseEstimator backPhotonPoseEstimator;

	Optional<EstimatedRobotPose> resultFront;
	Optional<EstimatedRobotPose> resultBack;
	boolean useVision = true;
	double frontLastTimeStamp = 0;
	double backLastTimeStamp = 0;

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
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		frontCam = new PhotonCamera("frontcam");
		frontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCam);

		backCam = new PhotonCamera("rearcam");
		backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCam);

		setVisionWeights(.2, .2, 10);

		createAlert(frontCam, "leftCam");
		createAlert(backCam, "rightCam");
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

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedLeftGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : frontCam.getAllUnreadResults()) {
			visEst = frontPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedRightGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : backCam.getAllUnreadResults()) {
			visEst = backPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

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

	public void log() {
		SmartDashboard.putBoolean("/Vision/Left/Connected", frontCam.isConnected());
		SmartDashboard.putBoolean("/Vision/Right/Connected", backCam.isConnected());
	}

	public Matrix<N3, N1> getVisionWeights(double distanceRatio, int numTargets) {
		double targetMultiplier = 0.75;
		double visionCutOffDistance = 4;
		distanceRatio = 0.1466 * Math.pow(1.6903, distanceRatio);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[] { 99999, 99999, 99999 }));
			}
			targetMultiplier = 3;
		}
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[] { distanceRatio * targetMultiplier,
				distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier }));
		return weights;
	}

	@Override
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
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
					lastAlliance = Alliance.Blue;
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				}
				if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {

					lastAlliance = Alliance.Red;
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				}
			}

			frontPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultFront = getEstimatedLeftGlobalPose();
			this.resultBack = getEstimatedRightGlobalPose();

			if (useVision) {

				if (resultFront.isPresent()) {
					EstimatedRobotPose camPoseLeft = resultFront.get();
					double leftTimeStamp = camPoseLeft.timestampSeconds;
					if (leftTimeStamp > Timer.getFPGATimestamp()) {
						leftTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFront.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseLeft.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseLeft.targetsUsed.size());

					if (leftTimeStamp != frontLastTimeStamp) {
						publishPose2d("/DriveTrain/LeftCamPose", camPoseLeft.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/LeftWeights", weights.toString());
						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseLeft.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(leftTimeStamp), weights);

					}
					frontLastTimeStamp = leftTimeStamp;
				}

				if (resultBack.isPresent()) {
					EstimatedRobotPose camPoseRight = resultBack.get();
					double rightTimeStamp = camPoseRight.timestampSeconds;

					if (rightTimeStamp > Timer.getFPGATimestamp()) {
						rightTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBack.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseRight.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseRight.targetsUsed.size());

					if (rightTimeStamp != backLastTimeStamp) {
						publishPose2d("/DriveTrain/RightCamPose", camPoseRight.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/RightWeights", weights.toString());

						RobotContainer.drivetrain.addVisionMeasurement(
								camPoseRight.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(rightTimeStamp),
								weights);
					}
					backLastTimeStamp = rightTimeStamp;
				}

			}
		}
	}
}