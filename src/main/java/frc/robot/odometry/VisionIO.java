package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

public interface VisionIO {

  public static class VisionPoseEstimate {
    /** Estimated pose, in meters. */
    public Pose3d pose = new Pose3d();
    /** Timestamp of the pose estimate, in seconds. */
    public double timestampSeconds = 0.0;
  }

  /** Configures vision hardware to default. */
  public void configure();

  /**
   * Gets the latest estimated robot pose from vision data.
   *
   * @return the estimated pose of the robot from vision data.
   */
  public Optional<VisionPoseEstimate> getEstimatedPose();
}
