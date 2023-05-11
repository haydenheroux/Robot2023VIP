// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.GyroIO.GyroIOValues;

public class Swerve extends SubsystemBase implements TelemetryOutputter {
  // Singleton instance
  private static Swerve instance = null;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private final Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.9, 0.9, 0.9);

  private final GyroIO gyro;
  private final GyroIOValues gyroValues = new GyroIOValues();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final PIDController thetaController;

  private final SwerveAutoBuilder autoBuilder;

  private final Field2d field = new Field2d();

  /** Creates a new Swerve. */
  private Swerve() {
    modules[0] = new Module(Constants.Swerve.FrontLeft.CONFIG);
    modules[1] = new Module(Constants.Swerve.FrontRight.CONFIG);
    modules[2] = new Module(Constants.Swerve.BackLeft.CONFIG);
    modules[3] = new Module(Constants.Swerve.BackRight.CONFIG);

    kinematics =
        new SwerveDriveKinematics(
            modules[0].getLocation(),
            modules[1].getLocation(),
            modules[2].getLocation(),
            modules[3].getLocation());

    thetaController = new PIDController(4.0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    autoBuilder =
        new SwerveAutoBuilder(
            this::getPose,
            this::setPose,
            this.kinematics,
            new PIDConstants(5.0, 0, 0),
            new PIDConstants(50, 0, 0),
            this::setSetpoints,
            Constants.Auto.EVENT_MAP,
            true,
            this);

    if (Robot.isSimulation()) {
      gyro = new GyroIOSim(() -> kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond);
    } else {
      gyro = null;
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
        "omegaRadiansPerSecond",
        () -> kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond);

    tab.addDoubleArray("Module States", this::getStatesAsArray);

	ShuffleboardLayout gyroLayout = tab.getLayout("Gyroscope", BuiltInLayouts.kList);
	gyroLayout.addNumber("Roll (deg)", () -> Units.radiansToDegrees(gyroValues.rollAngleRadians));
	gyroLayout.addNumber("Pitch (deg)", () -> Units.radiansToDegrees(gyroValues.pitchAngleRadians));
	gyroLayout.addNumber("Yaw (deg)", () -> Units.radiansToDegrees(gyroValues.yawAngleRadians));

    ShuffleboardLayout module0Layout = tab.getLayout("Module 0", BuiltInLayouts.kList);
    module0Layout.addNumber("Angle (deg)", () -> getStates()[0].angle.getDegrees());
    module0Layout.addNumber("Absolute Angle (deg)", () -> modules[0].getAbsoluteAzimuthAngle().getDegrees());
    module0Layout.addNumber("Velocity (mps)", () -> getStates()[0].speedMetersPerSecond);

    ShuffleboardLayout module1Layout = tab.getLayout("Module 1", BuiltInLayouts.kList);
    module1Layout.addNumber("Angle (deg)", () -> getStates()[1].angle.getDegrees());
    module1Layout.addNumber("Absolute Angle (deg)", () -> modules[1].getAbsoluteAzimuthAngle().getDegrees());
    module1Layout.addNumber("Velocity (mps)", () -> getStates()[1].speedMetersPerSecond);

    ShuffleboardLayout module2Layout = tab.getLayout("Module 2", BuiltInLayouts.kList);
    module2Layout.addNumber("Angle (deg)", () -> getStates()[2].angle.getDegrees());
    module2Layout.addNumber("Absolute Angle (deg)", () -> modules[2].getAbsoluteAzimuthAngle().getDegrees());
    module2Layout.addNumber("Velocity (mps)", () -> getStates()[2].speedMetersPerSecond);

    ShuffleboardLayout module3Layout = tab.getLayout("Module 3", BuiltInLayouts.kList);
    module3Layout.addNumber("Angle (deg)", () -> getStates()[3].angle.getDegrees());
    module3Layout.addNumber("Absolute Angle (deg)", () -> modules[3].getAbsoluteAzimuthAngle().getDegrees());
    module3Layout.addNumber("Velocity (mps)", () -> getStates()[3].speedMetersPerSecond);
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }

  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(gyroValues.rollAngleRadians);
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(gyroValues.pitchAngleRadians); }

  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(gyroValues.yawAngleRadians);
  }

  public void setRoll(Rotation2d roll) {
    gyro.setRollAngle(roll.getRadians());
  }

  public void setPitch(Rotation2d pitch) {
    gyro.setPitchAngle(pitch.getRadians());
  }

  public void setYaw(Rotation2d yaw) {
    gyro.setYawAngle(yaw.getRadians());

    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }

  public void setSetpoints(SwerveModuleState[] setpoints) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, Constants.Swerve.MAX_SPEED);

    for (int i = 0; i < 4; i++) {
      modules[i].setSetpoint(setpoints[i]);
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
    double omegaRadiansPerSecond =
        thetaController.calculate(getYaw().getRadians(), heading.getRadians());

    drive(velocity, omegaRadiansPerSecond);
  }

  public void drive(Translation2d velocity, double omegaRadiansPerSecond) {
    SwerveModuleState[] desiredStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(), velocity.getY(), omegaRadiansPerSecond, getYaw()));

    setSetpoints(desiredStates);
  }

  public SwerveAutoBuilder getAutoBuilder() {
    return autoBuilder;
  }
}
