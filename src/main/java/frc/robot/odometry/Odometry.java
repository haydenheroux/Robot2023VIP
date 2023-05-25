package frc.robot.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.odometry.GyroIO.GyroIOValues;
import frc.robot.swerve.Swerve;
import java.util.function.Supplier;

public class Odometry extends SubsystemBase implements TelemetryOutputter {
  private static Odometry instance = null;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

  private final GyroIO gyro;
  private final GyroIOValues gyroValues = new GyroIOValues();

  private final Supplier<SwerveModulePosition[]> positions;

  private final Field2d field;

  private Odometry() {
    if (Robot.isSimulation()) {
      gyro = new GyroIOSim(() -> Units.radiansToRotations(getRobotVelocity().omegaRadiansPerSecond));
    } else {
      gyro = new GyroIOPigeon2(7, "Drivetrain");
    }

    positions = () -> Swerve.getInstance().getPositions();

    Pose2d initialPoseMeters = new Pose2d();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.KINEMATICS,
            initialPoseMeters.getRotation(),
            positions.get(),
            initialPoseMeters,
            stateStandardDeviations,
            visionStandardDeviations);

    gyro.configure();

    field = new Field2d();
  }

  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }
    return instance;
  }

  @Override
  public void periodic() {
    gyro.updateValues(gyroValues);

    poseEstimator.update(getYaw(), positions.get());

    field.setRobotPose(getPose());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Odometry");

    tab.add(field);

    ShuffleboardLayout rotation = tab.getLayout("Rotation", BuiltInLayouts.kList);
    rotation.addNumber("Roll (deg)", () -> getRoll().getDegrees());
    rotation.addNumber("Pitch (deg)", () -> getPitch().getDegrees());
    rotation.addNumber("Yaw (deg)", () -> getYaw().getDegrees());

    ShuffleboardLayout position = tab.getLayout("Position", BuiltInLayouts.kList);
    position.addNumber("X (m)", () -> getPose().getX());
    position.addNumber("Y (m)", () -> getPose().getY());

    ShuffleboardLayout robotVelocity = tab.getLayout("Robot Velocity", BuiltInLayouts.kList);
    robotVelocity.addNumber("X Velocity (mps)", () -> getRobotVelocity().vxMetersPerSecond);
    robotVelocity.addNumber("Y Velocity (mps)", () -> getRobotVelocity().vyMetersPerSecond);
    robotVelocity.addNumber("Angular Velocity (dps)", () -> Units.radiansToDegrees(getRobotVelocity().omegaRadiansPerSecond));

    ShuffleboardLayout fieldVelocity = tab.getLayout("Field Velocity", BuiltInLayouts.kList);
    fieldVelocity.addNumber("X Velocity (mps)", () -> getFieldVelocity(getRobotVelocity()).getX());
    fieldVelocity.addNumber("Y Velocity (mps)", () -> getFieldVelocity(getRobotVelocity()).getY());
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public ChassisSpeeds getRobotVelocity() {
    return Constants.Swerve.KINEMATICS.toChassisSpeeds(Swerve.getInstance().getStates());
  }

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
  public Translation2d getFieldVelocity(ChassisSpeeds robotVelocity) {
    final Rotation2d yaw = getYaw(); 

    // https://www.chiefdelphi.com/t/determining-robot-relative-velocity-with-odometry-field-relative-speeds-on-swerve/412233/19
    double vxMetersPerSecond = robotVelocity.vxMetersPerSecond * yaw.getCos() - robotVelocity.vyMetersPerSecond * yaw.getSin();
    double vyMetersPerSecond = robotVelocity.vxMetersPerSecond * yaw.getSin() + robotVelocity.vyMetersPerSecond * yaw.getCos();

    return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
  }

  public void setYaw(Rotation2d yaw) {
    gyro.setYawAngle(yaw.getRotations());

    setPose(new Pose2d(getPose().getTranslation(), yaw));
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
