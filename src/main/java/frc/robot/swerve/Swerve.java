// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;
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

  private final Module[] modules = new Module[4];

  /** Creates a new Swerve. */
  private Swerve() {
    if (Robot.isSimulation()) {
      gyro = null; 
    } else {
      gyro = null; 
    }

    gyro.configure();

    modules[0] = new Module(null);
    modules[1] = new Module(null);
    modules[2] = new Module(null);
    modules[3] = new Module(null);

    kinematics = new SwerveDriveKinematics(modules[0].getLocation(), modules[1].getLocation(), modules[2].getLocation(), modules[3].getLocation());
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), new Pose2d(), stateStandardDeviations, visionStandardDeviations);

    setYaw(Rotation2d.fromDegrees(0));
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

    poseEstimator.update(getYaw(), getModulePositions());
  }

  @Override
  public void initializeDashboard() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }

  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(gyroValues.rollAngleRadians);
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(gyroValues.pitchAngleRadians);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(gyroValues.yawAngleRadians);
  }

  public void setRoll(Rotation2d roll) {
    gyro.setRollOffset(roll.getRadians());
  }

  public void setPitch(Rotation2d pitch) {
    gyro.setPitchOffset(pitch.getRadians());
  }

  public void setYaw(Rotation2d yaw) {
    gyro.setYawOffset(yaw.getRadians());

    setPose(new Pose2d(getPose().getTranslation(), yaw));
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    final double kMaxSpeedMetersPerSecond = 0.0;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
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
    poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose, double timestamp, boolean hard, double accuracy) {
    if (hard) {
      setPose(pose);
    } else {
      poseEstimator.addVisionMeasurement(pose, timestamp, visionStandardDeviations.times(1.0 / accuracy));
    }
  }

}
