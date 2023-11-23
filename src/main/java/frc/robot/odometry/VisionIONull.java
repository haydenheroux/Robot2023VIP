package frc.robot.odometry;

import java.util.Optional;

public class VisionIONull implements VisionIO {

  public VisionIONull() {}

  @Override
  public void configure() {}

  @Override
  public Optional<VisionPoseEstimate> getEstimatedPose() {
    return Optional.empty();
  }
}
