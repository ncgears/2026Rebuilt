
package frc.robot.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
/**
 * Constants for the Vision class
 */
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
public class VisionConstants {
    public static final GlobalConstants.TelemetryLevel kTelemetryLevel = GlobalConstants.TelemetryLevel.DEBUG;
    public static final boolean kUseVisionForPose = true; //enable vision measurements to pose correction
    /** Enables rejecting vision estimates that use targets above the ambiguity threshold. */
    public static final boolean kUsePoseAmbiguityFilter = true;
    /** Maximum allowed per-target pose ambiguity for accepted vision estimates. */
    public static final double kMaxPoseAmbiguity = 0.20;
    /** Enables rejecting vision estimates that differ too much from current odometry. */
    public static final boolean kUsePoseConsistencyFilter = false;  //true
    /** Apply consistency filtering only to single-tag estimates (most prone to flips). */
    public static final boolean kConsistencyFilterSingleTagOnly = false; //true
    /** Number of accepted frames per camera to bypass consistency filtering after acquisition. */
    public static final int kConsistencyWarmupBypassFrames = 3;
    /** Time gap (seconds) after which per-camera warmup bypass is rearmed. */
    public static final double kConsistencyWarmupResetGapSeconds = 0.75;
    /** Maximum allowed translation delta (meters) from current odometry to accept vision. */
    public static final double kMaxPoseTranslationDeltaMeters = 2.50;
    /** Maximum allowed heading delta (degrees) from current odometry to accept vision. */
    public static final double kMaxPoseHeadingDeltaDegrees = 100.0;
    /** Enables rejecting sudden per-camera vision jumps between consecutive accepted estimates. */
    public static final boolean kUseVisionJumpFilter = true;
    /** Maximum allowed per-camera translation jump over a short time window (meters). */
    public static final double kMaxVisionJumpDeltaMeters = 1.25;
    /** Maximum allowed per-camera heading jump over a short time window (degrees). */
    public static final double kMaxVisionJumpHeadingDeltaDegrees = 75.0;
    /** Maximum time window (seconds) used for per-camera jump filtering. */
    public static final double kMaxVisionJumpDeltaTimeSeconds = 0.35;
    public static final boolean kUseAutoSuppress = false; //enable suppressing vision measurements based on speed
    public static final double kAutosuppressSpeedMetersPerSecond = 2.5; //speed at which to suppress vision addition
    public static final AprilTagFieldLayout kTagLayout = getTagLayout();
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.15, 0.15, 0.3);

    /* AHA!
     * The first argument of kRobotToCam is the translation3d representing the center of the robot
     * to the center of the camera, regardless of orientation of the camera, such that +X is toward 
     * the front of the robot, +Y is to the left of the robot (when facing forward), +Z is the height from the floor. 
     * All measured in meters.
     * 
     * The second argument of kRobotToCam is the rotation3d representing the orientation of the camera
     * as it relates to the front facing robot. 
     * ie.
     *  yaw is rotation around Z (heading) 
     *  pitch is rotation around Y (tilt)
     *  roll is rotation around X, Photonvision site says counterclockwise
     * 
     * Front facing camera with no yaw, pitch, or roll would typically be Rotation3d.kZero, which
     * is the same as "new Rotation3d(0,0,0)"
     * 
     * Rear facing camera will typically be something like "new Rotation3d(0,0,Math.toRadians(180))" 
     */

    public static final class Front { //Right facing camera 2026 *****
        public static final String kCameraName = "frontcam";
        public static final boolean kUseForPose = true;
        //+x left from center, +y forward from center, +z up from ground
        public static final Transform3d kRobotToCam = new Transform3d(
          // new Translation3d(0.290,0.250,0.280), //x,y,z location of camera on robot in meters
          new Translation3d(-0.0864,-0.3637,0.51), //x,y,z location of camera on robot in meters
          //Rotation3d.kZero //yaw,pitch/roll of camera on robot in radians
          new Rotation3d(0,0,Math.toRadians(270)) //yaw,pitch/roll of camera on robot in radians
        );
    }
    public static final class Back { //backwards facing camera
        public static final String kCameraName = "rearcam";
        public static final boolean kUseForPose = true;
        public static final Transform3d kRobotToCam = new Transform3d(
          new Translation3d(-0.3076,0.2094,0.359), //x,y,z location of camera on robot in meters
          new Rotation3d(0,0,Math.toRadians(180)) //yaw,pitch/roll of camera on robot in radians
        );
    }

    /** This method tries to load a custom tag layout json file, or falls back to the default field 
     * @return AprilTagFieldLayout for the custom layout file or the default field
    */
    private static AprilTagFieldLayout getTagLayout() {
      try {
        return new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "/apriltags/2026-rebuilt-welded.json");
      } catch (Exception e) {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      }
    }
}

