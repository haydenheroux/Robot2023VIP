package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import frc.robot.swerve.AngleMotorIO.AngleMotorIOValues;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;

public class Module {

  private final ModuleConfiguration config;

  private final AngleMotorIO angleMotor;
  private final AngleMotorIOValues angleMotorValues = new AngleMotorIOValues();

  private final DriveMotorIO driveMotor;
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

  private final AzimuthEncoderIO azimuthEncoder;
  private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

  private SwerveModuleState state;

  public Module(ModuleConfiguration config) {
    this.config = config;

    if (Robot.isSimulation()) {
      angleMotor = new AngleMotorIOSim();
      driveMotor = new DriveMotorIOSim();
      azimuthEncoder = new AzimuthEncoderIOSim(config.azimuthOffset.getRotations());
    } else {
      angleMotor = new AngleMotorIOTalonFX(config.can.angle, config.can.bus);
      driveMotor = new DriveMotorIOTalonFX(config.can.drive, config.can.bus);
      azimuthEncoder = new AzimuthEncoderIOCANCoder(config.can.azimuth, config.can.bus);
    }

    angleMotor.configure();
    driveMotor.configure();

    azimuthEncoder.configure();

    azimuthEncoder.updateValues(azimuthEncoderValues);
    angleMotor.setPosition(
        azimuthEncoderValues.absoluteAngleRotations - config.azimuthOffset.getRotations());

    state = getState();
  }

  public void update() {
    angleMotor.updateValues(angleMotorValues);
    driveMotor.updateValues(driveMotorValues);

    azimuthEncoder.updateValues(azimuthEncoderValues);

    state = getState();

    // TODO angleMotor.setPosition({azimuthAngle})
  }

  public void setSetpoint(SwerveModuleState setpoint) {
    setSetpoint(setpoint, false);
  }

  public void setSetpoint(SwerveModuleState setpoint, boolean isForced) {
    setpoint =
        SwerveModuleState.optimize(
            setpoint, Rotation2d.fromRotations(angleMotorValues.angleRotations));

    driveMotor.setVelocitySetpoint(setpoint.speedMetersPerSecond);

    if (isForced == false) {
      setpoint = SwerveMath.dejitter(setpoint, state.angle, Swerve.DEJITTER_SPEED);
    }

    boolean angleChanged = setpoint.angle.equals(state.angle) == false;

    if (angleChanged) {
      angleMotor.setSetpoint(setpoint.angle.getRotations());
    }

    state = setpoint;
  }

  public Translation2d getLocation() {
    return config.location;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }

  public Rotation2d getAbsoluteAzimuthAngle() {
    return Rotation2d.fromRotations(azimuthEncoderValues.absoluteAngleRotations);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(angleMotorValues.angleRotations));
  }

  public void setMotorBrake(boolean isActive) {
    driveMotor.setBrake(isActive);
  }
}
