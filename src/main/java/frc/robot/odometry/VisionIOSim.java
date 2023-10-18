package frc.robot.odometry;

import frc.robot.Constants.Physical;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;

public class VisionIOSim implements VisionIO {

  private final PhotonCamera camera;

  private final SimVisionSystem visionSim;

  private final PhotonPoseEstimator photonEstimator;

  public VisionIOSim(String cameraName) {
    camera = new PhotonCamera(cameraName);

    double cameraDiagonalFOV = 170.0;
    double maxLEDRange = 20;
    double minTargetArea = 10;

    visionSim =
        new SimVisionSystem(
            cameraName,
            cameraDiagonalFOV,
            Physical.ROBOT_TO_CAMERA,
            maxLEDRange,
            640,
            480,
            minTargetArea);
    visionSim.addVisionTargets(Physical.APRIL_TAGS);

    photonEstimator =
        new PhotonPoseEstimator(
            Physical.APRIL_TAGS, PoseStrategy.MULTI_TAG_PNP, camera, Physical.ROBOT_TO_CAMERA);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void configure() {}

  @Override
  public Optional<VisionPoseEstimate> getEstimatedPose() {
    visionSim.processFrame(Odometry.getInstance().getPose());

    Optional<EstimatedRobotPose> optional = photonEstimator.update();

    if (optional.isEmpty()) return Optional.empty();

    EstimatedRobotPose estimate = optional.get();

    VisionPoseEstimate visionPoseEstimate = new VisionPoseEstimate();

    visionPoseEstimate.pose = estimate.estimatedPose;
    visionPoseEstimate.timestampSeconds = estimate.timestampSeconds;

    return Optional.of(visionPoseEstimate);
  }
}
