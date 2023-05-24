package frc.robot.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Robot;
import frc.robot.pose.GyroIO.GyroIOValues;
import frc.robot.swerve.Swerve;
import java.util.function.Supplier;

public class PoseEstimator extends SubsystemBase implements TelemetryOutputter {
  private static PoseEstimator instance = null;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

  private final GyroIO gyro;
  private final GyroIOValues gyroValues = new GyroIOValues();

  private final Supplier<SwerveModulePosition[]> positions;

  private PoseEstimator() {
    if (Robot.isSimulation()) {
      gyro = new GyroIOSim(() -> 0.0);
    } else {
      gyro = new GyroIOPigeon2(7, "Drivetrain");
    }

    positions = () -> Swerve.getInstance().getPositions();

    Pose2d initialPoseMeters = new Pose2d();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Swerve.getInstance().kinematics,
            initialPoseMeters.getRotation(),
            positions.get(),
            initialPoseMeters,
            stateStandardDeviations,
            visionStandardDeviations);

    gyro.configure();
  }

  public static PoseEstimator getInstance() {
    if (instance == null) {
      instance = new PoseEstimator();
    }
    return instance;
  }

  @Override
  public void periodic() {
    gyro.updateValues(gyroValues);

    poseEstimator.update(getYaw(), positions.get());
  }

  @Override
  public void initializeDashboard() {
    // TODO Auto-generated method stub
  }

  @Override
  public void outputTelemetry() {}

  public Rotation2d getRoll() {
    return Rotation2d.fromRotations(gyroValues.rollAngleRotations);
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromRotations(gyroValues.pitchAngleRotations);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromRotations(gyroValues.yawAngleRotations);
  }

  public void setYaw(Rotation2d yaw) {
    gyro.setYawAngle(yaw.getRotations());

    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), positions.get(), pose);
  }

  public void setPose(Pose2d pose, double timestamp, boolean hard, double accuracy) {
    if (hard) {
      setPose(pose);
    } else {
      poseEstimator.addVisionMeasurement(
          pose, timestamp, visionStandardDeviations.times(1.0 / accuracy));
    }
  }
}
