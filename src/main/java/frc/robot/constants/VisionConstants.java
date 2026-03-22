
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
    public static final boolean kUseAutoSuppress = false; //enable suppressing vision measurements based on speed
    public static final double kAutosuppressSpeedMetersPerSecond = 2.5; //speed at which to suppress vision addition
    public static final AprilTagFieldLayout kTagLayout = getTagLayout();
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.0, 1.0, 2.0);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, 0.5);

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

