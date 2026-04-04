
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
  private enum VisionCameraRole {
    FRONT,
    SHOOTER,
    BACK
  }

	private static Vision instance;
  public final PhotonCamera front_camera, shooter_camera, back_camera;
  private final PhotonPoseEstimator photonEstimatorFront, photonEstimatorShooter, photonEstimatorBack;
  private Matrix<N3, N1> curStdDevsFront, curStdDevsShooter, curStdDevsBack;
  private double lastEstTimestampFront, lastEstTimestampShooter, lastEstTimestampBack = 0;
  private Pose2d m_visFrontPose = Pose2d.kZero;
  private Pose2d m_visShooterPose = Pose2d.kZero;
  private Pose2d m_visBackPose = Pose2d.kZero;
  private boolean m_frontHasTargets = false;
  private boolean m_backHasTargets = false;
  private boolean m_shooterHasTargets = false;
  private boolean m_frontRejectedAmbiguity = false;
  private boolean m_backRejectedAmbiguity = false;
  private boolean m_shooterRejectedAmbiguity = false;
  private boolean m_frontRejectedConsistency = false;
  private boolean m_backRejectedConsistency = false;
  private boolean m_shooterRejectedConsistency = false;
  private boolean m_frontRejectedJump = false;
  private boolean m_backRejectedJump = false;
  private boolean m_shooterRejectedJump = false;
  private boolean m_frontWarmupBypassActive = false;
  private boolean m_backWarmupBypassActive = false;
  private boolean m_shooterWarmupBypassActive = false;
  private Pose2d m_frontLastAcceptedPose = Pose2d.kZero;
  private Pose2d m_backLastAcceptedPose = Pose2d.kZero;
  private Pose2d m_shooterLastAcceptedPose = Pose2d.kZero;
  private double m_frontLastAcceptedTimestampSeconds = Double.NaN;
  private double m_backLastAcceptedTimestampSeconds = Double.NaN;
  private double m_shooterLastAcceptedTimestampSeconds = Double.NaN;
  private boolean m_frontHasAcceptedPose = false;
  private boolean m_backHasAcceptedPose = false;
  private boolean m_shooterHasAcceptedPose = false;
  private int m_frontWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;
  private int m_backWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;
  private int m_shooterWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;

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

    shooter_camera = new PhotonCamera(VisionConstants.Shooter.kCameraName);
    photonEstimatorShooter = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Shooter.kRobotToCam);
    photonEstimatorShooter.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Matrix<N3, N1> shooter_curStdDevs;

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
        PhotonCameraSim shooter_cameraSim = new PhotonCameraSim(shooter_camera, cameraProp);
        PhotonCameraSim back_cameraSim = new PhotonCameraSim(back_camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(front_cameraSim, VisionConstants.Front.kRobotToCam);
        visionSim.addCamera(shooter_cameraSim, VisionConstants.Shooter.kRobotToCam);
        visionSim.addCamera(back_cameraSim, VisionConstants.Back.kRobotToCam);
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
    boolean shooterSuppressed = false;
    boolean backSuppressed = false;
    if (RobotContainer.drivetrain != null) {
      frontSuppressed = RobotContainer.drivetrain.isFrontVisionSuppressed();
      shooterSuppressed = RobotContainer.drivetrain.isShooterVisionSuppressed();
      backSuppressed = RobotContainer.drivetrain.isBackVisionSuppressed();
    }

    SmartDashboard.putBoolean("Subsystems/Vision/Front/HasTargets", m_frontHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/HasTargets", m_backHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/HasTargets", m_shooterHasTargets);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/Suppressed", frontSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/Suppressed", backSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/Suppressed", shooterSuppressed);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedAmbiguity", m_frontRejectedAmbiguity);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedAmbiguity", m_backRejectedAmbiguity);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedAmbiguity", m_shooterRejectedAmbiguity);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedConsistency", m_frontRejectedConsistency);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedConsistency", m_backRejectedConsistency);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedConsistency", m_shooterRejectedConsistency);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/RejectedJump", m_frontRejectedJump);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/RejectedJump", m_backRejectedJump);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/RejectedJump", m_shooterRejectedJump);
    SmartDashboard.putBoolean("Subsystems/Vision/Front/WarmupBypassActive", m_frontWarmupBypassActive);
    SmartDashboard.putBoolean("Subsystems/Vision/Back/WarmupBypassActive", m_backWarmupBypassActive);
    SmartDashboard.putBoolean("Subsystems/Vision/Shooter/WarmupBypassActive", m_shooterWarmupBypassActive);

    if (!GlobalConstants.telemetryAtLeast(VisionConstants.kTelemetryLevel, GlobalConstants.TelemetryLevel.DEBUG)) return;
    // DEBUG level telemetry goes here

    SmartDashboard.putNumber("Subsystems/Vision/Front/PoseX", m_visFrontPose.getX());
    SmartDashboard.putNumber("Subsystems/Vision/Front/PoseY", m_visFrontPose.getY());
    SmartDashboard.putNumber("Subsystems/Vision/Shooter/PoseX", m_visShooterPose.getX());
    SmartDashboard.putNumber("Subsystems/Vision/Shooter/PoseY", m_visShooterPose.getY());
    SmartDashboard.putNumber("Subsystems/Vision/Back/PoseX", m_visBackPose.getX());
    SmartDashboard.putNumber("Subsystems/Vision/Back/PoseY", m_visBackPose.getY());
    SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseX", RobotContainer.drivetrain.getBotPose().getX());
    SmartDashboard.putNumber("Subsystems/Vision/Robot/PoseY", RobotContainer.drivetrain.getBotPose().getY());
    SmartDashboard.putNumber("Subsystems/Vision/Front/LastTimestamp", lastEstTimestampFront);
    SmartDashboard.putNumber("Subsystems/Vision/Shooter/LastTimestamp", lastEstTimestampShooter);
    SmartDashboard.putNumber("Subsystems/Vision/Back/LastTimestamp", lastEstTimestampBack);
  }

  /**
   * Gets the last cached vision pose for the requested camera.
   *
   * @param cam Camera name ("front" or "shooter" or "back").
   * @return Estimated pose for the camera.
   */
  public Pose2d getVisionPose(String cam) {
    if ("front".equalsIgnoreCase(cam)) {
      return m_visFrontPose;
    }
    if ("shooter".equalsIgnoreCase(cam)) {
      return m_visShooterPose;
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
   * Returns the latest estimated global pose from the front camera.
   *
   * @return Optional estimated pose.
   */
  public Optional<EstimatedRobotPose> getShooterEstimatedGlobalPose() {
    return getEstimatedGlobalPose(photonEstimatorShooter, shooter_camera, curStdDevsShooter); 
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
      var results = camera.getAllUnreadResults();
      for (var result: results) {
        setCameraHasTargets(camera, result.hasTargets());
        var multiTagResult = result.getMultiTagResult();
        if(multiTagResult.isPresent()) {
          // var fieldToCamera = multiTagResult.get().estimatedPose.best;
        }
        visionEst = estimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = estimator.estimateLowestAmbiguityPose(result);
        }


        //  visionEst = estimator.update(result);
        updateEstimationStdDevs(visionEst, result.getTargets(), estimator, stdDevs);
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

        if (visionEst.isPresent()) {
          Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
          setCameraEstimatedPose(camera, estimatedPose, visionEst.get().timestampSeconds);
        }
      }
      return visionEst;
  }

    /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseOld(PhotonPoseEstimator estimator, PhotonCamera camera, Matrix<N3, N1> stdDevs) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      var results = camera.getAllUnreadResults();
      for (var result: results) {
        setCameraHasTargets(camera, result.hasTargets());
        visionEst = estimator.update(result);
        updateEstimationStdDevs(visionEst, result.getTargets(), estimator, stdDevs);
        if (visionEst.isPresent()) {
          Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
          setCameraEstimatedPose(camera, estimatedPose, visionEst.get().timestampSeconds);
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
   * Gets standard deviations for a shooter-camera pose estimate.
   *
   * @param pose Pose to evaluate.
   * @return Standard deviation matrix.
   */
  public Matrix<N3, N1> getShooterEstimationStdDevs(Pose2d pose) {
    return getEstimationStdDevs(pose, photonEstimatorShooter, shooter_camera);
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
   * Returns whether all usable target ambiguities in an estimate are at or below
   * the configured threshold.
   * Targets with negative ambiguity are ignored because PhotonVision uses negative
   * values when ambiguity is unavailable for that target.
   *
   * @param estimate Vision estimate to evaluate.
   * @return True when estimate passes ambiguity filtering.
   */
  private boolean passesPoseAmbiguityFilter(EstimatedRobotPose estimate) {
    if (!VisionConstants.kUsePoseAmbiguityFilter || estimate.targetsUsed == null) {
      return true;
    }
    for (PhotonTrackedTarget target : estimate.targetsUsed) {
      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity >= 0.0 && ambiguity > VisionConstants.kMaxPoseAmbiguity) {
        return false;
      }
    }
    return true;
  }

  /**
   * Returns whether a vision estimate is reasonably close to the current odometry
   * pose in both translation and heading.
   *
   * @param estimate Vision estimate to evaluate.
   * @return True when estimate passes consistency filtering.
   */
  private boolean passesPoseConsistencyFilter(EstimatedRobotPose estimate) {
    if (!VisionConstants.kUsePoseConsistencyFilter || RobotContainer.drivetrain == null) {
      return true;
    }
    if (VisionConstants.kConsistencyFilterSingleTagOnly && estimate.targetsUsed != null && estimate.targetsUsed.size() > 1) {
      return true;
    }
    Pose2d currentPose = RobotContainer.drivetrain.getBotPose();
    Pose2d estimatedPose = estimate.estimatedPose.toPose2d();
    double translationDeltaMeters = currentPose.getTranslation().getDistance(estimatedPose.getTranslation());
    double headingDeltaDegrees = Math.abs(currentPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
    return translationDeltaMeters <= VisionConstants.kMaxPoseTranslationDeltaMeters
      && headingDeltaDegrees <= VisionConstants.kMaxPoseHeadingDeltaDegrees;
  }

  /**
   * Returns the configured role for a camera instance used by vision filtering.
   *
   * @param camera Camera to classify.
   * @return Camera role used by per-camera state tracking.
   */
  private VisionCameraRole getCameraRole(PhotonCamera camera) {
    if (camera == front_camera) {
      return VisionCameraRole.FRONT;
    }
    if (camera == shooter_camera) {
      return VisionCameraRole.SHOOTER;
    }
    if (camera == back_camera) {
      return VisionCameraRole.BACK;
    }
    throw new IllegalArgumentException("Unknown camera passed to Vision: " + ((camera != null) ? camera.getName() : "null"));
  }

  /**
   * Updates the has-targets status for a specific camera.
   *
   * @param camera Camera that produced a pipeline result.
   * @param hasTargets True when the camera currently reports targets.
   */
  private void setCameraHasTargets(PhotonCamera camera, boolean hasTargets) {
    switch (getCameraRole(camera)) {
      case FRONT:
        m_frontHasTargets = hasTargets;
        return;
      case SHOOTER:
        m_shooterHasTargets = hasTargets;
        return;
      case BACK:
        m_backHasTargets = hasTargets;
        return;
      default:
        return;
    }
  }

  /**
   * Stores the latest estimated pose and timestamp for a specific camera.
   *
   * @param camera Camera associated with the estimate.
   * @param estimatedPose Estimated robot pose from that camera.
   * @param timestampSeconds Estimate timestamp in seconds.
   */
  private void setCameraEstimatedPose(PhotonCamera camera, Pose2d estimatedPose, double timestampSeconds) {
    switch (getCameraRole(camera)) {
      case FRONT:
        m_visFrontPose = estimatedPose;
        lastEstTimestampFront = timestampSeconds;
        return;
      case SHOOTER:
        m_visShooterPose = estimatedPose;
        lastEstTimestampShooter = timestampSeconds;
        return;
      case BACK:
        m_visBackPose = estimatedPose;
        lastEstTimestampBack = timestampSeconds;
        return;
      default:
        return;
    }
  }

  /**
   * Returns whether consistency filtering should be bypassed for this estimate
   * during a short per-camera warmup window.
   *
   * @param camera Camera associated with this estimate.
   * @param timestampSeconds Vision estimate timestamp.
   * @return True when consistency filtering should be bypassed.
   */
  private boolean isConsistencyWarmupBypassActive(PhotonCamera camera, double timestampSeconds) {
    if (VisionConstants.kConsistencyWarmupBypassFrames <= 0) {
      return false;
    }
    switch (getCameraRole(camera)) {
      case FRONT:
        if (Double.isFinite(m_frontLastAcceptedTimestampSeconds)
          && timestampSeconds - m_frontLastAcceptedTimestampSeconds > VisionConstants.kConsistencyWarmupResetGapSeconds) {
          m_frontWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;
        }
        return m_frontWarmupBypassRemaining > 0;
      case SHOOTER:
        if (Double.isFinite(m_shooterLastAcceptedTimestampSeconds)
          && timestampSeconds - m_shooterLastAcceptedTimestampSeconds > VisionConstants.kConsistencyWarmupResetGapSeconds) {
          m_shooterWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;
        }
        return m_shooterWarmupBypassRemaining > 0;
      case BACK:
        if (Double.isFinite(m_backLastAcceptedTimestampSeconds)
          && timestampSeconds - m_backLastAcceptedTimestampSeconds > VisionConstants.kConsistencyWarmupResetGapSeconds) {
          m_backWarmupBypassRemaining = VisionConstants.kConsistencyWarmupBypassFrames;
        }
        return m_backWarmupBypassRemaining > 0;
      default:
        return false;
    }
  }

  /**
   * Returns whether a vision estimate does not exhibit a sudden per-camera jump
   * compared to the last accepted pose for that camera.
   *
   * @param estimate Vision estimate to evaluate.
   * @param camera Camera associated with this estimate.
   * @return True when the estimate passes jump filtering.
   */
  private boolean passesPoseJumpFilter(EstimatedRobotPose estimate, PhotonCamera camera) {
    if (!VisionConstants.kUseVisionJumpFilter) {
      return true;
    }
    Pose2d estimatedPose = estimate.estimatedPose.toPose2d();
    Pose2d lastAcceptedPose;
    double lastAcceptedTimestamp;
    boolean hasAcceptedPose;
    switch (getCameraRole(camera)) {
      case FRONT:
        lastAcceptedPose = m_frontLastAcceptedPose;
        lastAcceptedTimestamp = m_frontLastAcceptedTimestampSeconds;
        hasAcceptedPose = m_frontHasAcceptedPose;
        break;
      case SHOOTER:
        lastAcceptedPose = m_shooterLastAcceptedPose;
        lastAcceptedTimestamp = m_shooterLastAcceptedTimestampSeconds;
        hasAcceptedPose = m_shooterHasAcceptedPose;
        break;
      case BACK:
        lastAcceptedPose = m_backLastAcceptedPose;
        lastAcceptedTimestamp = m_backLastAcceptedTimestampSeconds;
        hasAcceptedPose = m_backHasAcceptedPose;
        break;
      default:
        return true;
    }
    if (!hasAcceptedPose || !Double.isFinite(lastAcceptedTimestamp)) {
      return true;
    }
    double deltaTimeSeconds = estimate.timestampSeconds - lastAcceptedTimestamp;
    if (deltaTimeSeconds <= 0.0 || deltaTimeSeconds > VisionConstants.kMaxVisionJumpDeltaTimeSeconds) {
      return true;
    }
    double translationDeltaMeters = lastAcceptedPose.getTranslation().getDistance(estimatedPose.getTranslation());
    double headingDeltaDegrees = Math.abs(lastAcceptedPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
    return translationDeltaMeters <= VisionConstants.kMaxVisionJumpDeltaMeters
      && headingDeltaDegrees <= VisionConstants.kMaxVisionJumpHeadingDeltaDegrees;
  }

  /**
   * Stores per-camera state after a vision estimate is accepted and fused.
   *
   * @param camera Camera that produced the accepted estimate.
   * @param acceptedPose Pose that was accepted.
   * @param timestampSeconds Estimate timestamp.
   * @param warmupBypassUsed True when consistency filtering was bypassed for this estimate.
   */
  private void recordAcceptedVisionMeasurement(
    PhotonCamera camera,
    Pose2d acceptedPose,
    double timestampSeconds,
    boolean warmupBypassUsed
  ) {
    switch (getCameraRole(camera)) {
      case FRONT:
        m_frontLastAcceptedPose = acceptedPose;
        m_frontLastAcceptedTimestampSeconds = timestampSeconds;
        m_frontHasAcceptedPose = true;
        if (warmupBypassUsed && m_frontWarmupBypassRemaining > 0) {
          m_frontWarmupBypassRemaining--;
        }
        return;
      case SHOOTER:
        m_shooterLastAcceptedPose = acceptedPose;
        m_shooterLastAcceptedTimestampSeconds = timestampSeconds;
        m_shooterHasAcceptedPose = true;
        if (warmupBypassUsed && m_shooterWarmupBypassRemaining > 0) {
          m_shooterWarmupBypassRemaining--;
        }
        return;
      case BACK:
        m_backLastAcceptedPose = acceptedPose;
        m_backLastAcceptedTimestampSeconds = timestampSeconds;
        m_backHasAcceptedPose = true;
        if (warmupBypassUsed && m_backWarmupBypassRemaining > 0) {
          m_backWarmupBypassRemaining--;
        }
        return;
      default:
        return;
    }
  }

  /**
   * Updates per-camera dashboard filter status flags for the current estimate.
   *
   * @param camera Camera that produced the estimate.
   * @param warmupBypassActive True when consistency warmup bypass is active.
   * @param passesAmbiguity True when ambiguity filtering passes.
   * @param passesConsistency True when consistency filtering passes.
   * @param passesJump True when jump filtering passes.
   */
  private void setFilterFlagsForCamera(
    PhotonCamera camera,
    boolean warmupBypassActive,
    boolean passesAmbiguity,
    boolean passesConsistency,
    boolean passesJump
  ) {
    switch (getCameraRole(camera)) {
      case FRONT:
        m_frontWarmupBypassActive = warmupBypassActive;
        m_frontRejectedAmbiguity = !passesAmbiguity;
        m_frontRejectedConsistency = passesAmbiguity && !passesConsistency;
        m_frontRejectedJump = passesAmbiguity && passesConsistency && !passesJump;
        return;
      case SHOOTER:
        m_shooterWarmupBypassActive = warmupBypassActive;
        m_shooterRejectedAmbiguity = !passesAmbiguity;
        m_shooterRejectedConsistency = passesAmbiguity && !passesConsistency;
        m_shooterRejectedJump = passesAmbiguity && passesConsistency && !passesJump;
        return;
      case BACK:
        m_backWarmupBypassActive = warmupBypassActive;
        m_backRejectedAmbiguity = !passesAmbiguity;
        m_backRejectedConsistency = passesAmbiguity && !passesConsistency;
        m_backRejectedJump = passesAmbiguity && passesConsistency && !passesJump;
        return;
      default:
        return;
    }
  }

  /**
   * Resets per-camera dashboard filter status flags when no estimate is available.
   *
   * @param camera Camera whose status flags should be reset.
   */
  private void resetFilterFlagsForCamera(PhotonCamera camera) {
    switch (getCameraRole(camera)) {
      case FRONT:
        m_frontWarmupBypassActive = false;
        m_frontRejectedAmbiguity = false;
        m_frontRejectedConsistency = false;
        m_frontRejectedJump = false;
        return;
      case SHOOTER:
        m_shooterWarmupBypassActive = false;
        m_shooterRejectedAmbiguity = false;
        m_shooterRejectedConsistency = false;
        m_shooterRejectedJump = false;
        return;
      case BACK:
        m_backWarmupBypassActive = false;
        m_backRejectedAmbiguity = false;
        m_backRejectedConsistency = false;
        m_backRejectedJump = false;
        return;
      default:
        return;
    }
  }

  /**
   * Returns the standard-deviation matrix for an estimate from a specific camera.
   *
   * @param camera Camera that produced the estimate.
   * @param estimatedPose Estimated pose from that camera.
   * @return Standard deviation matrix for that camera estimate.
   */
  private Matrix<N3, N1> getEstimationStdDevsForCamera(PhotonCamera camera, Pose2d estimatedPose) {
    switch (getCameraRole(camera)) {
      case FRONT:
        return getFrontEstimationStdDevs(estimatedPose);
      case SHOOTER:
        return getShooterEstimationStdDevs(estimatedPose);
      case BACK:
        return getBackEstimationStdDevs(estimatedPose);
      default:
        return VisionConstants.kSingleTagStdDevs;
    }
  }

  /**
   * Adds a filtered vision estimate to the drivetrain pose estimator.
   *
   * @param estimate Vision estimate from one camera.
   * @param camera Camera that produced the estimate.
   */
  private void addFilteredVisionMeasurement(EstimatedRobotPose estimate, PhotonCamera camera) {
    boolean passesAmbiguity = passesPoseAmbiguityFilter(estimate);
    boolean warmupBypassActive = isConsistencyWarmupBypassActive(camera, estimate.timestampSeconds);
    boolean passesConsistency = warmupBypassActive || passesPoseConsistencyFilter(estimate);
    boolean passesJump = passesPoseJumpFilter(estimate, camera);
    setFilterFlagsForCamera(camera, warmupBypassActive, passesAmbiguity, passesConsistency, passesJump);
    if (!passesAmbiguity || !passesConsistency || !passesJump) {
      return;
    }
    Pose2d estPose = estimate.estimatedPose.toPose2d();
    Matrix<N3, N1> estStdDevs = getEstimationStdDevsForCamera(camera, estPose);
    // For CTRE, timestamp must be in correct timebase, use Utils.fpgaToCurrentTime(timestamp).
    RobotContainer.drivetrain.addVisionMeasurement(
      estPose,
      Utils.fpgaToCurrentTime(estimate.timestampSeconds),
      estStdDevs
    );
    recordAcceptedVisionMeasurement(camera, estPose, estimate.timestampSeconds, warmupBypassActive);
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
    correctFrontPoseWithVision();
    correctShooterPoseWithVision();
    correctBackPoseWithVision();
	}

  /**
   * Corrects pose using only front-camera vision.
   */
  public void correctFrontPoseWithVision() {
    if (VisionConstants.kUseVisionForPose && VisionConstants.Front.kUseForPose) {
      var visionEstFront = RobotContainer.vision.getFrontEstimatedGlobalPose();
      visionEstFront.ifPresentOrElse(
        est -> addFilteredVisionMeasurement(est, front_camera),
        () -> resetFilterFlagsForCamera(front_camera)
      );
    }
  }

  /**
   * Corrects pose using only shooter-camera vision.
   */
  public void correctShooterPoseWithVision() {
    if (VisionConstants.kUseVisionForPose && VisionConstants.Shooter.kUseForPose) {
      var visionEstShooter = RobotContainer.vision.getShooterEstimatedGlobalPose();
      visionEstShooter.ifPresentOrElse(
        est -> addFilteredVisionMeasurement(est, shooter_camera),
        () -> resetFilterFlagsForCamera(shooter_camera)
      );
    }
  }

  /**
   * Corrects pose using only back-camera vision.
   */
  public void correctBackPoseWithVision() {
    if (VisionConstants.kUseVisionForPose && VisionConstants.Back.kUseForPose) {
      var visionEstBack = RobotContainer.vision.getBackEstimatedGlobalPose();
      visionEstBack.ifPresentOrElse(
        est -> addFilteredVisionMeasurement(est, back_camera),
        () -> resetFilterFlagsForCamera(back_camera)
      );
    }
  }

}
