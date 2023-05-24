package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.telemetry.TelemetryOutputter;
import frc.robot.Robot;
import frc.robot.swerve.AngleMotorIO.AngleMotorIOValues;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;

public class Module implements TelemetryOutputter {

  public final ModuleConfiguration config;

  private final AngleMotorIO angleMotor;
  private final AngleMotorIOValues angleMotorValues = new AngleMotorIOValues();

  private final DriveMotorIO driveMotor;
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

  private final AzimuthEncoderIO azimuthEncoder;
  private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

  public Module(ModuleConfiguration config) {
    this.config = config;

    if (Robot.isSimulation()) {
      angleMotor = new AngleMotorIOSim();
      driveMotor = new DriveMotorIOSim();
      azimuthEncoder = new AzimuthEncoderIOSim(config.azimuthOffsetRotations);
    } else {
      angleMotor = new AngleMotorIOTalonFX(config.can.angle, config.can.bus);
      driveMotor = new DriveMotorIOTalonFX(config.can.drive, config.can.bus);
      azimuthEncoder =
          new AzimuthEncoderIOCANCoder(
              config.can.azimuth, config.can.bus, config.azimuthOffsetRotations);
    }

    angleMotor.configure();
    driveMotor.configure();

    azimuthEncoder.configure();

    azimuthEncoder.updateValues(azimuthEncoderValues);

    angleMotor.setPosition(azimuthEncoderValues.angleRotations);
  }

  public void update() {
    azimuthEncoder.updateValues(azimuthEncoderValues);

    angleMotor.updateValues(angleMotorValues);
    driveMotor.updateValues(driveMotorValues);
  }

  public void setSetpoint(SwerveModuleState setpoint) {
    driveMotor.setVelocitySetpoint(setpoint.speedMetersPerSecond);

    angleMotor.setSetpoint(setpoint.angle.getRotations());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }

  public Rotation2d getAzimuthAngle() {
    return Rotation2d.fromRotations(azimuthEncoderValues.angleRotations);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }

  @Override
  public void initializeDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    ShuffleboardLayout layout = tab.getLayout(this.config.name, BuiltInLayouts.kList);
    layout.addNumber("Angle (deg)", () -> getState().angle.getDegrees());
    layout.addNumber("Absolute Angle (deg)", () -> getAzimuthAngle().getDegrees());
    layout.addNumber("Velocity (mps)", () -> getState().speedMetersPerSecond);
  }

  @Override
  public void outputTelemetry() {}
}
