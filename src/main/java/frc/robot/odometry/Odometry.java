package frc.robot.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.CustomRotation3d;
import frc.lib.math.Util;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.odometry.GyroIO.GyroIOValues;
import frc.robot.swerve.Swerve;
import java.util.function.DoubleSupplier;
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

  private final VisionIO vision;

  private final Supplier<SwerveModulePosition[]> positions;

  private final Field2d field;

  private final TripTracker tripTracker;

  private Odometry() {
    if (Robot.isSimulation()) {
      NetworkTable odometrySim = NetworkTableInstance.getDefault().getTable("odometrySim");

      DoubleSupplier odometryOmegaRotationsPerSecond =
          () -> getVelocity().getRotation().getRotations();

      DoubleEntry rotationsPerSecondPerMetersPerSecond =
          odometrySim.getDoubleTopic("rotationsPerSecondPerMetersPerSecond").getEntry(0.0);
      rotationsPerSecondPerMetersPerSecond.set(0.0);

      DoubleSupplier direction =
          () -> {
            Transform2d velocity = getVelocity();
            double signX = Math.signum(velocity.getX());
            double signY = Math.signum(velocity.getY());

            return (signX < 0 || signY < 0) ? -1.0 : 1.0;
          };

      DoubleSupplier driftOmegaRotationsPerSecond =
          () ->
              direction.getAsDouble()
                  * rotationsPerSecondPerMetersPerSecond.get()
                  * getVelocity().getTranslation().getNorm();

      DoubleSupplier omegaRotationsPerSecond =
          () ->
              odometryOmegaRotationsPerSecond.getAsDouble()
                  + driftOmegaRotationsPerSecond.getAsDouble();

      gyro = new GyroIOSim(omegaRotationsPerSecond);

      vision = new VisionIONull();
    } else {
      gyro = new GyroIOPigeon2(7, Ports.CANIVORE);

      vision = new VisionIONull();
    }

    positions = () -> Swerve.getInstance().getPositions();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.KINEMATICS,
            new Rotation2d(),
            positions.get(),
            new Pose2d(),
            stateStandardDeviations,
            visionStandardDeviations);

    gyro.configure();

    field = new Field2d();

    tripTracker = new TripTracker();
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

    vision
        .getEstimatedPose()
        .ifPresent(
            estimate -> {
              Pose2d pose = estimate.pose.toPose2d();

              poseEstimator.addVisionMeasurement(pose, estimate.timestampSeconds);
            });

    poseEstimator.update(getGyro().getYaw(), positions.get());

    field.setRobotPose(getPose());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Odometry");

    tab.add(field);

    ShuffleboardLayout position = tab.getLayout("Position", BuiltInLayouts.kList);
    position.addNumber("X (m)", () -> getPose().getX());
    position.addNumber("Y (m)", () -> getPose().getY());
    position.addNumber("Rotation (deg)", () -> getPose().getRotation().getDegrees());

    ShuffleboardLayout velocity = tab.getLayout("Velocity", BuiltInLayouts.kList);
    velocity.addNumber("X Velocity (mps)", () -> getVelocity().getX());
    velocity.addNumber("Y Velocity (mps)", () -> getVelocity().getY());
    velocity.addNumber("Velocity (mps)", () -> getVelocity().getTranslation().getNorm());

    ShuffleboardLayout trip = tab.getLayout("Trip Meter", BuiltInLayouts.kList);
    trip.addNumber("Trip Distance X (m)", () -> tripTracker.getDistance().getX());
    trip.addNumber("Trip Distance Y (m)", () -> tripTracker.getDistance().getY());
    trip.addNumber("Trip Distance (m)", () -> tripTracker.getDistance().getTranslation().getNorm());
    trip.add(Commands.runOnce(tripTracker::start).withName("Reset Trip Meter"));

    ShuffleboardLayout gyro = tab.getLayout("Gyro", BuiltInLayouts.kList);
    gyro.addNumber("Roll (deg)", () -> getGyro().getRoll().getDegrees());
    gyro.addNumber("Pitch (deg)", () -> getGyro().getPitch().getDegrees());
    gyro.addNumber("Yaw (deg)", () -> getGyro().getYaw().getDegrees());
    gyro.addNumber("Tilt (deg)", () -> getTilt().getDegrees());

    ShuffleboardLayout platform = tab.getLayout("Platform", BuiltInLayouts.kList);
    platform.addBoolean("Is Level?", this::isLevel);
    platform.addBoolean("On Flap?", this::onFlap);
    platform.addBoolean("On Platform?", this::onPlatform);
  }

  @Override
  public void outputTelemetry() {}

  public void startTrip() {
    tripTracker.start();
  }

  public void stopTrip() {
    tripTracker.stop();
  }

  public Transform2d getTripDistance() {
    return tripTracker.getDistance();
  }

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
    poseEstimator.resetPosition(getGyro().getYaw(), positions.get(), pose);
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

  /**
   * Gets the velocity of the robot in the field reference frame.
   *
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
   *     Reference Frame</a>
   * @return the velocity of the robot in the field reference frame.
   */
  private Transform2d getVelocity() {
    final ChassisSpeeds robotVelocity = Swerve.getInstance().getChassisSpeeds();

    final Rotation2d rotation = getPose().getRotation();

    // https://www.chiefdelphi.com/t/determining-robot-relative-velocity-with-odometry-field-relative-speeds-on-swerve/412233/19
    double vxMetersPerSecond =
        robotVelocity.vxMetersPerSecond * rotation.getCos()
            - robotVelocity.vyMetersPerSecond * rotation.getSin();
    double vyMetersPerSecond =
        robotVelocity.vxMetersPerSecond * rotation.getSin()
            + robotVelocity.vyMetersPerSecond * rotation.getCos();

    Translation2d velocity = new Translation2d(vxMetersPerSecond, vyMetersPerSecond);

    return new Transform2d(velocity, Rotation2d.fromRadians(robotVelocity.omegaRadiansPerSecond));
  }

  /**
   * Gets the angle of the robot relative to the robot X axis.
   *
   * @return the angle of the robot relative to the robot X axis.
   */
  public CustomRotation3d getGyro() {
    return new CustomRotation3d(
        Rotation2d.fromRotations(gyroValues.rollAngleRotations),
        Rotation2d.fromRotations(gyroValues.pitchAngleRotations),
        Rotation2d.fromRotations(gyroValues.yawAngleRotations));
  }

  public Rotation2d getTilt() {
    // https://github.com/Mechanical-Advantage/RobotCode2023/blob/81a63b84a29d67d62154e4af02631a525014eafa/src/main/java/org/littletonrobotics/frc2023/commands/AutoBalance.java#L49
    final Rotation2d pitch = getGyro().getPitch().times(getPose().getRotation().getCos());
    final Rotation2d roll = getGyro().getRoll().times(getPose().getRotation().getSin());
    return pitch.plus(roll);
  }

  public boolean tiltedBelow(Rotation2d tilt) {
    // https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
    // Pitching nose down is positive
    return getTilt().getDegrees() > tilt.getDegrees();
  }

  public boolean tiltedAbove(Rotation2d tilt) {
    // https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
    // Pitching nose down is positive
    return getTilt().getDegrees() < tilt.getDegrees();
  }

  public boolean tiltedAt(Rotation2d tilt, Rotation2d threshold) {
    return Util.approximatelyEqual(
        getTilt().getDegrees(), tilt.getDegrees(), threshold.getDegrees());
  }

  private final Rotation2d TILT_THRESHOLD = Rotation2d.fromDegrees(3.0);

  public boolean tiltedAt(Rotation2d tilt) {
    return tiltedAt(tilt, TILT_THRESHOLD);
  }

  public boolean onFlap() {
    final double kFlapDegrees = 5;
    return tiltedAt(Rotation2d.fromDegrees(-kFlapDegrees))
        || tiltedAt(Rotation2d.fromDegrees(kFlapDegrees));
  }

  public boolean onPlatform() {
    final double kPlatformDegrees = 12;
    return tiltedAt(Rotation2d.fromDegrees(-kPlatformDegrees))
        || tiltedAt(Rotation2d.fromDegrees(kPlatformDegrees));
  }

  public boolean isLevel() {
    return tiltedAt(Rotation2d.fromDegrees(0));
  }
}
