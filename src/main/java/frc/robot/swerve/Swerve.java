package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.GyroIO.GyroIOValues;

public class Swerve extends SubsystemBase implements TelemetryOutputter {
  // Singleton instance
  private static Swerve instance = null;

  public final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

  private final GyroIO gyro;
  private final GyroIOValues gyroValues = new GyroIOValues();

  private final Module[] modules = new Module[4]; // FL=NW, FR=NE, BR=SE, BL=SW

  private final PIDController thetaController;

  private final Field2d field = new Field2d();

  /** Creates a new Swerve. */
  private Swerve() {
    modules[0] = new Module(Constants.Swerve.NORTH_WEST);
    modules[1] = new Module(Constants.Swerve.NORTH_EAST);
    modules[2] = new Module(Constants.Swerve.SOUTH_EAST);
    modules[3] = new Module(Constants.Swerve.SOUTH_WEST);

    kinematics =
        new SwerveDriveKinematics(
            modules[0].config.location,
            modules[1].config.location,
            modules[2].config.location,
            modules[3].config.location);

    thetaController = new PIDController(4.0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    if (Robot.isSimulation()) {
      gyro =
          new GyroIOSim(
              () ->
                  Units.radiansToRotations(
                      kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond));
    } else {
      gyro =  new GyroIOPigeon2(7, "Drivetrain");
    }

    Pose2d initialPoseMeters = new Pose2d();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getPositions(),
            initialPoseMeters,
            stateStandardDeviations,
            visionStandardDeviations);

    gyro.configure();
    // Assume that we are flat on the ground
    setRoll(Rotation2d.fromDegrees(0));
    setPitch(Rotation2d.fromDegrees(0));
    setYaw(initialPoseMeters.getRotation());
  }

  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  @Override
  public void periodic() {
    for (var module : modules) {
      module.update();
    }

    gyro.updateValues(gyroValues);

    poseEstimator.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add(field);

    tab.addNumber(
        "omegaRotationsPerSecond",
        () ->
            Units.radiansToRotations(
                kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond));

    tab.addDoubleArray("Module States", this::getStatesAsArray);

    ShuffleboardLayout gyroLayout = tab.getLayout("Gyroscope", BuiltInLayouts.kList);
    gyroLayout.addNumber(
        "Roll (deg)", () -> Units.rotationsToDegrees(gyroValues.rollAngleRotations));
    gyroLayout.addNumber(
        "Pitch (deg)", () -> Units.rotationsToDegrees(gyroValues.pitchAngleRotations));
    gyroLayout.addNumber("Yaw (deg)", () -> Units.rotationsToDegrees(gyroValues.yawAngleRotations));

    ShuffleboardLayout module0Layout =
        tab.getLayout("[0] " + modules[0].config.name, BuiltInLayouts.kList);
    module0Layout.addNumber("Angle (deg)", () -> getStates()[0].angle.getDegrees());
    module0Layout.addNumber(
        "Absolute Angle (deg)", () -> modules[0].getAbsoluteAzimuthAngle().getDegrees());
    module0Layout.addNumber("Velocity (mps)", () -> getStates()[0].speedMetersPerSecond);

    ShuffleboardLayout module1Layout =
        tab.getLayout("[1] " + modules[1].config.name, BuiltInLayouts.kList);
    module1Layout.addNumber("Angle (deg)", () -> getStates()[1].angle.getDegrees());
    module1Layout.addNumber(
        "Absolute Angle (deg)", () -> modules[1].getAbsoluteAzimuthAngle().getDegrees());
    module1Layout.addNumber("Velocity (mps)", () -> getStates()[1].speedMetersPerSecond);

    ShuffleboardLayout module2Layout =
        tab.getLayout("[2] " + modules[2].config.name, BuiltInLayouts.kList);
    module2Layout.addNumber("Angle (deg)", () -> getStates()[2].angle.getDegrees());
    module2Layout.addNumber(
        "Absolute Angle (deg)", () -> modules[2].getAbsoluteAzimuthAngle().getDegrees());
    module2Layout.addNumber("Velocity (mps)", () -> getStates()[2].speedMetersPerSecond);

    ShuffleboardLayout module3Layout =
        tab.getLayout("[3] " + modules[3].config.name, BuiltInLayouts.kList);
    module3Layout.addNumber("Angle (deg)", () -> getStates()[3].angle.getDegrees());
    module3Layout.addNumber(
        "Absolute Angle (deg)", () -> modules[3].getAbsoluteAzimuthAngle().getDegrees());
    module3Layout.addNumber("Velocity (mps)", () -> getStates()[3].speedMetersPerSecond);
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

  public void setRoll(Rotation2d roll) {
    gyro.setRollAngle(roll.getRotations());
  }

  public void setPitch(Rotation2d pitch) {
    gyro.setPitchAngle(pitch.getRotations());
  }

  public void setYaw(Rotation2d yaw) {
    gyro.setYawAngle(yaw.getRotations());

    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  public void setSetpoints(SwerveModuleState[] setpoints) {
    setSetpoints(setpoints, false);
  }

  public void setSetpoints(SwerveModuleState[] setpoints, boolean force) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Swerve.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i], force);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  public double[] getStatesAsArray() {
    double[] doubles = new double[8];

    for (int i = 0; i < 4; i++) {
      SwerveModuleState state = modules[i].getState();
      doubles[2 * i] = state.angle.getRadians();
      doubles[2 * i + 1] = state.speedMetersPerSecond;
    }

    return doubles;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }

    return positions;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public void setPose(Pose2d pose, double timestamp, boolean hard, double accuracy) {
    if (hard) {
      setPose(pose);
    } else {
      poseEstimator.addVisionMeasurement(
          pose, timestamp, visionStandardDeviations.times(1.0 / accuracy));
    }
  }

  public void drive(Translation2d velocity, Rotation2d heading) {
    double omegaRotationsPerSecond =
        thetaController.calculate(getYaw().getRotations(), heading.getRotations());

    drive(velocity, omegaRotationsPerSecond);
  }

  public void drive(Translation2d velocity, double omegaRotationsPerSecond) {
    ChassisSpeeds chassisVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(),
            velocity.getY(),
            Units.rotationsToRadians(omegaRotationsPerSecond),
            getYaw());

    chassisVelocity = SwerveMath.getCorrectedChassisVelocity(chassisVelocity);

    SwerveModuleState[] setpoints = kinematics.toSwerveModuleStates(chassisVelocity);

    setSetpoints(setpoints);
  }

  public Command lock() {
    return this.runOnce(
        () -> {
          setSetpoints(
              new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
              },
              true);
        });
  }
}
