package frc.robot.odometry;

import frc.robot.Constants.Physical;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

  private final PhotonCamera camera;

  private final VisionSystemSim visionSim;

  private final PhotonPoseEstimator photonEstimator;

  public VisionIOSim(String cameraName) {
    camera = new PhotonCamera(cameraName);

    visionSim = new VisionSystemSim(cameraName);
    visionSim.addAprilTags(Physical.APRIL_TAGS);

    photonEstimator =
        new PhotonPoseEstimator(
            Physical.APRIL_TAGS,
            PoseStrategy.AVERAGE_BEST_TARGETS,
            camera,
            Physical.ROBOT_TO_CAMERA);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void configure() {}

  @Override
  public Optional<VisionPoseEstimate> getEstimatedPose() {
    visionSim.update(Odometry.getInstance().getPose());

    Optional<EstimatedRobotPose> optional = photonEstimator.update();

    if (optional.isEmpty()) return Optional.empty();

    EstimatedRobotPose estimate = optional.get();

    VisionPoseEstimate visionPoseEstimate = new VisionPoseEstimate();

    visionPoseEstimate.pose = estimate.estimatedPose;
    visionPoseEstimate.timestampSeconds = estimate.timestampSeconds;

    return Optional.of(visionPoseEstimate);
  }
}
