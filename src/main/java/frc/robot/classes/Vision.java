
package frc.robot.classes;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.*; 
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Vision class handles getting and managing data from the PhotoVision system.
 * It is responsible for getting target data, selecting appropriate targets, and passing information to other subsystems.
 */
public class Vision {
	private static Vision instance;
  public final PhotonCamera front_camera, back_camera;
  private final PhotonPoseEstimator photonEstimatorFront, photonEstimatorBack;
  private Matrix<N3, N1> curStdDevsFront, curStdDevsBack;
  private double lastEstTimestampFront, lastEstTimestampBack = 0;
  private Pose2d m_visFrontPose = Pose2d.kZero;
  private Pose2d m_visBackPose = Pose2d.kZero;
  private boolean m_frontHasTargets = false;
  private boolean m_backHasTargets = false;

  // Simulator
  private VisionSystemSim visionSim;

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

  /** Creates the vision system and initializes cameras and simulation. */
  public Vision() {
    front_camera = new PhotonCamera(VisionConstants.Front.kCameraName);
    photonEstimatorFront = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Front.kRobotToCam);
    photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Matrix<N3, N1> front_curStdDevs;

    back_camera = new PhotonCamera(VisionConstants.Back.kCameraName);
    photonEstimatorBack = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Back.kRobotToCam);
    photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Matrix<N3, N1> back_curStdDevs;

    // Simulation
    if (Robot.isSimulation()) {
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(VisionConstants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        PhotonCameraSim front_cameraSim = new PhotonCameraSim(front_camera, cameraProp);
        PhotonCameraSim back_cameraSim = new PhotonCameraSim(back_camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(front_cameraSim, VisionConstants.Front.kRobotToCam);
        visionSim.addCamera(back_cameraSim,VisionConstants.Back.kRobotToCam);
    }
  }

  /**
   * Publishes vision telemetry to SmartDashboard.
   * INFO-level fields are always published at {@link GlobalConstants.TelemetryLevel#INFO}
   * and above. Additional pose details are published at
   * {@link GlobalConstants.TelemetryLevel#DEBUG}.
   */
  public void updateDashboards() {
    if (!GlobalConstants.telemetryAtLeast(VisionConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.INFO)) return;
    // INFO level telemetry goes here

    boolean frontSuppressed = false;
    boolean backSuppressed = false;
    if (RobotContainer.drivetrain != null) {
      frontSuppressed = RobotContainer.drivetrain.isFrontVisionSuppressed();
      backSuppressed = RobotContainer.drivetrain.isBackVisionSuppressed();
    }

    SmartDashboard.putBoolean("Subsystems/Vision/Front/HasTargets", m_frontHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/HasTargets", m_backHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/Suppressed", frontSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/Suppressed", backSuppressed);

    if (!GlobalConstants.telemetryAtLeast(VisionConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
    // DEBUG level telemetry goes here

    SmartDashboard.putNumber("Subsystems/Vision/Front/PoseX", m_visFrontPose.getX());
    SmartDashboard.putNumber("Subsystems/Vision/Front/PoseY", m_visFrontPose.getY());
    SmartDashboard.putNumber("Subsystems/Vision/Back/PoseX", m_visBackPose.getX());
    SmartDashboard.putNumber("Subsystems/Vision/Back/PoseY", m_visBackPose.getY());
    SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseX", RobotContainer.drivetrain.getBotPose().getX());
    SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseY", RobotContainer.drivetrain.getBotPose().getY());
    SmartDashboard.putNumber("Subsystems/Vision/Front/LastTimestamp", lastEstTimestampFront);
    SmartDashboard.putNumber("Subsystems/Vision/Back/LastTimestamp", lastEstTimestampBack);
  }

  /**
   * Gets the last cached vision pose for the requested camera.
   *
   * @param cam Camera name ("front" or "back").
   * @return Estimated pose for the camera.
   */
  public Pose2d getVisionPose(String cam) {
    if ("front".equalsIgnoreCase(cam)) {
      return m_visFrontPose;
    }
    return m_visBackPose;
  }

  /**
   * Returns the latest pipeline result for a camera.
   *
   * @param camera Camera to query.
   * @return Latest pipeline result.
   */
  public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    var results = camera.getAllUnreadResults();
    return results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);
  }

  /**
   * Returns the latest estimated global pose from the front camera.
   *
   * @return Optional estimated pose.
   */
  public Optional<EstimatedRobotPose> getFrontEstimatedGlobalPose() {
    return getEstimatedGlobalPose(photonEstimatorFront, front_camera, curStdDevsFront); 
  }
  /**
   * Returns the latest estimated global pose from the back camera.
   *
   * @return Optional estimated pose.
   */
  public Optional<EstimatedRobotPose> getBackEstimatedGlobalPose() {
    return getEstimatedGlobalPose(photonEstimatorBack, back_camera, curStdDevsBack); 
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator, PhotonCamera camera, Matrix<N3, N1> stdDevs) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change: camera.getAllUnreadResults()) {
        if (camera == front_camera) {
          m_frontHasTargets = change.hasTargets();
        } else if (camera == back_camera) {
          m_backHasTargets = change.hasTargets();
        }
        visionEst = estimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets(), estimator, stdDevs);
        if (visionEst.isPresent()) {
          Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
          if (camera == front_camera) {
            m_visFrontPose = estimatedPose;
            lastEstTimestampFront = visionEst.get().timestampSeconds;
          } else if (camera == back_camera) {
            m_visBackPose = estimatedPose;
            lastEstTimestampBack = visionEst.get().timestampSeconds;
          }
        }
        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
            est ->
              getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            }
          );
        }
      }
      return visionEst;
  }

      /**
       * Updates pose estimation standard deviations based on observed targets.
       *
       * @param estimatedPose Pose estimate to evaluate.
       * @param targets Visible targets in the frame.
       * @param estimator Pose estimator used for tag lookup.
       * @param stdDev Output standard deviation matrix to update.
       */
      private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator, Matrix<N3, N1> stdDev) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            stdDev = VisionConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                stdDev = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                stdDev = estStdDevs;
            }
        }
    }

  /**
   * Gets standard deviations for a front-camera pose estimate.
   *
   * @param pose Pose to evaluate.
   * @return Standard deviation matrix.
   */
  public Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d pose) {
    return getEstimationStdDevs(pose, photonEstimatorFront, front_camera);
  }
  /**
   * Gets standard deviations for a back-camera pose estimate.
   *
   * @param pose Pose to evaluate.
   * @return Standard deviation matrix.
   */
  public Matrix<N3, N1> getBackEstimationStdDevs(Pose2d pose) {
    return getEstimationStdDevs(pose, photonEstimatorBack, back_camera);
  }


  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose(PhotonPoseEstimator, PhotonCamera, Matrix)}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param estimator Pose estimator to query for tag locations.
   * @param camera Camera providing target observations.
   * @return Standard deviation matrix for the estimate.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPoseEstimator estimator, PhotonCamera camera) {
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      var targets = getLatestResult(camera).getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
          var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
  }

  // ----- Simulation

  /**
   * Updates vision simulation using the current simulated robot pose.
   *
   * @param robotSimPose Simulated robot pose.
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return visionSim.getDebugField();
  }

  /** Updates internal vision results (placeholder). */
  public void updateResults() {
    
  }

  /**
   * addVisionMeasurement fuses the Pose2d from the vision system into the robot pose
   * @param visionMeasurement
   * @param timestampSeconds
   */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		RobotContainer.drivetrain.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}
  /**
   * Adds a vision measurement with custom standard deviations.
   *
   * @param visionMeasurement Vision pose measurement.
   * @param timestampSeconds Measurement timestamp in seconds.
   * @param stdDevs Standard deviation matrix for the measurement.
   */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		RobotContainer.drivetrain.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	/**
	 * Corrects the bot pose based on information from the vision system.
	 */
  @SuppressWarnings({"unused"})
	public void correctPoseWithVision() {
		if(VisionConstants.kUseVisionForPose && VisionConstants.Front.kUseForPose) {
			var visionEstFront = RobotContainer.vision.getFrontEstimatedGlobalPose();
			visionEstFront.ifPresent(
				est -> {
					var estPose = est.estimatedPose.toPose2d();
					var estStdDevs = RobotContainer.vision.getFrontEstimationStdDevs(estPose);
          //For CTR, timestamp must be in correct timebase, use Utils.fpgaToCurrentTime(timestamp) to correct
					RobotContainer.drivetrain.addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
				}
			);
		}
		if(VisionConstants.kUseVisionForPose && VisionConstants.Back.kUseForPose) {
			var visionEstBack = RobotContainer.vision.getBackEstimatedGlobalPose();
			visionEstBack.ifPresent(
				est -> {
					var estPose = est.estimatedPose.toPose2d();
					var estStdDevs = RobotContainer.vision.getBackEstimationStdDevs(estPose);
          //For CTR, timestamp must be in correct timebase, use Utils.fpgaToCurrentTime(timestamp) to correct
					RobotContainer.drivetrain.addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
				}
			);
		}
	}

}
