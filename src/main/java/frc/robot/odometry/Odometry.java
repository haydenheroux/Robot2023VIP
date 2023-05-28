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

/**
 * Tracks the robot's position on the field over the course of a match using drivetrain sensor data
 * and pose estimates from fudicial tracking.
 *
 * <p>Odometry data is used during the autonomous period for complex tasks such as path following,
 * during the teleoperated period for aligning with field elements, and after the match for
 * optimizing match play.
 */
public class Odometry extends SubsystemBase implements TelemetryOutputter {
  private static Odometry instance = null;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

  private final GyroIO gyro;
  private final GyroIOValues gyroValues = new GyroIOValues();

  private final Supplier<SwerveModulePosition[]> ArmPosition;

  private final Field2d field;

  private Odometry() {
    if (Robot.isSimulation()) {
      gyro =
          new GyroIOSim(() -> Units.radiansToRotations(getRobotVelocity().omegaRadiansPerSecond));
    } else {
      gyro = new GyroIOPigeon2(7, "Drivetrain");
    }

    ArmPosition = () -> Swerve.getInstance().getArmPosition();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.KINEMATICS,
            new Rotation2d(),
            ArmPosition.get(),
            new Pose2d(),
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

    poseEstimator.update(getGyroYaw(), ArmPosition.get());

    field.setRobotPose(getPose());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Odometry");

    tab.add(field);

    ShuffleboardLayout rotation = tab.getLayout("Rotation", BuiltInLayouts.kList);
    rotation.addNumber("Roll (deg)", () -> getRoll().getDegrees());
    rotation.addNumber("Pitch (deg)", () -> getPitch().getDegrees());
    // FIXME Gyro yaw and pose yaw are 180deg out of phase
    rotation.addNumber("Gyro Yaw (deg)", () -> getGyroYaw().getDegrees());
    rotation.addNumber("Pose Yaw (deg)", () -> getPoseYaw().getDegrees());

    ShuffleboardLayout position = tab.getLayout("Position", BuiltInLayouts.kList);
    position.addNumber("X (m)", () -> getPose().getX());
    position.addNumber("Y (m)", () -> getPose().getY());

    ShuffleboardLayout robotVelocity = tab.getLayout("Robot Velocity", BuiltInLayouts.kList);
    robotVelocity.addNumber("X Velocity (mps)", () -> getRobotVelocity().vxMetersPerSecond);
    robotVelocity.addNumber("Y Velocity (mps)", () -> getRobotVelocity().vyMetersPerSecond);
    robotVelocity.addNumber(
        "Angular Velocity (dps)",
        () -> Units.radiansToDegrees(getRobotVelocity().omegaRadiansPerSecond));

    ShuffleboardLayout fieldVelocity = tab.getLayout("Field Velocity", BuiltInLayouts.kList);
    fieldVelocity.addNumber("X Velocity (mps)", () -> getFieldVelocity(getRobotVelocity()).getX());
    fieldVelocity.addNumber("Y Velocity (mps)", () -> getFieldVelocity(getRobotVelocity()).getY());
  }

  @Override
  public void outputTelemetry() {}

  /**
   * Gets the estimated position of the robot on the field.
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
   *     Reference Frame</a>
   * @return the estimated position of the robot on the field.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Sets the position of the robot on the field.
   *
   * @param pose tne position of the robot on the field.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), ArmPosition.get(), pose);
  }

  /**
   * Sets the estimated position of the robot on the field.
   *
   * @param pose the estimated position of the robot on the field.
   * @param timestamp the timestamp of the position estimate.
   * @param hard if true, treat the estimated position as the position.
   * @param confidence the confidence of the position estimate. Lower values are less confident.
   */
  public void setPose(Pose2d pose, double timestamp, boolean hard, double confidence) {
    if (hard) {
      setPose(pose);
    } else {
      poseEstimator.addVisionMeasurement(
          pose, timestamp, visionStandardDeviations.times(1.0 / confidence));
    }
  }

  // TODO Are these robot axis relative or field axis relative?

  /**
   * Gets the angle of the robot relative to the robot X axis.
   *
   * @return the angle of the robot relative to the robot X axis.
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromRotations(gyroValues.rollAngleRotations);
  }

  /**
   * Gets the angle of the robot relative to the robot Y axis.
   *
   * @return the angle of the robot relative to the robot Y axis.
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromRotations(gyroValues.pitchAngleRotations);
  }

  /**
   * Gets the angle of the robot relative to the robot Z axis.
   *
   * @return the angle of the robot relative to the robot Z axis.
   */
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromRotations(gyroValues.yawAngleRotations);
  }

  /**
   * Gets the angle of the robot relative to the robot Z axis.
   *
   * @return the angle of the robot relative to the robot Z axis.
   */
  public Rotation2d getPoseYaw() {
    return getPose().getRotation();
  }

  /**
   * Sets the angle of the robot relative to the robot Z axis.
   *
   * <p>Setting the angle also updates the robot pose measurement.
   *
   * @param yaw the angle of the robot relative to the robot Z axis.
   */
  public void setYaw(Rotation2d yaw) {
    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  /**
   * Gets the velocity of the robot in the robot reference frame.
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *     Reference Frame</a>
   * @return the velocity of the robot in the robot reference frame.
   */
  private ChassisSpeeds getRobotVelocity() {
    return Constants.Swerve.KINEMATICS.toChassisSpeeds(Swerve.getInstance().getStates());
  }

  /**
   * Gets the velocity of the robot in the field reference frame.
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
   *     Reference Frame</a>
   * @return the velocity of the robot in the field reference frame.
   */
  private Translation2d getFieldVelocity(ChassisSpeeds robotVelocity) {
    final Rotation2d yaw = getGyroYaw();

    // https://www.chiefdelphi.com/t/determining-robot-relative-velocity-with-odometry-field-relative-speeds-on-swerve/412233/19
    double vxMetersPerSecond =
        robotVelocity.vxMetersPerSecond * yaw.getCos()
            - robotVelocity.vyMetersPerSecond * yaw.getSin();
    double vyMetersPerSecond =
        robotVelocity.vxMetersPerSecond * yaw.getSin()
            + robotVelocity.vyMetersPerSecond * yaw.getCos();

    return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
  }
}
