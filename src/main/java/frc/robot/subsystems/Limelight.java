package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private double[] poseList, targetPoseList, cameraPoseList;
    private NetworkTableEntry V_angle, H_angle, TwoH_angle, hasTargets, botPose, aprilID, targetPose, cameraPose;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    public Pose3d getRelative3dBotPose() {
        /*
        * Its specific because it determines what type of botpose we need
        * For example, we may need the botpose, botpose_wpiblue, botpose_wpired, etc
        * in order to tell our distance from the apriltag.
        * This method should give us an x and y position to the april tag as well as a
        * rotaiton angle to it
        */
        poseList = botPose.getDoubleArray(new double[6]);
        // position
        double x = poseList[0];
        double y = poseList[1];
        double z = poseList[2];
        // rotation
        double roll = poseList[3];
        double pitch = poseList[4];
        double yaw = poseList[5];

        Pose3d pose3d = new Pose3d(
            x,
            y,
            z,
            new Rotation3d(
                roll,
                pitch,
                yaw));
        return pose3d;
    }  

   public double getH_angle() {
    return H_angle.getDouble(0);
    }

  public boolean getHasTargets() {
    if (hasTargets.getDouble(0) == 0) {
      return false;
    }
    return true;
  }

  public double getTX(){
    return NetworkTableInstance.getDefault().getTable("ll").getEntry("tx").getDouble(0);
  }

  public double getTY(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

}
