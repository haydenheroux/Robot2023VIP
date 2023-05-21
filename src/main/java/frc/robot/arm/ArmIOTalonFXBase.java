package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.hardware.Hardware;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.RobotMap;

public class ArmIOTalonFXBase implements ArmIO {

  protected final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  protected final WPI_TalonFX extensionMotor, rotationMotor;
  private final Solenoid extensionBrake, rotationBrake;

  public ArmIOTalonFXBase() {
    extensionMotor = new WPI_TalonFX(RobotMap.EXTENSION);
    extensionBrake = Hardware.getSolenoid(RobotMap.EXTENSION_BRAKE);

    rotationMotor = new WPI_TalonFX(RobotMap.ROTATION);
    rotationBrake = Hardware.getSolenoid(RobotMap.ROTATION_BRAKE);
  }

  @Override
  public void configure() {
    extensionMotor.setInverted(false);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    rotationMotor.setInverted(true);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateValues(ArmIOValues values) {
    this.values.extensionBrakeIsActive = getExtensionBrakeIsActive();
    this.values.extensionLengthMeters = getExtensionPosition();
    this.values.extensionVoltage = getExtensionVoltage();

    this.values.rotationAngleRotations = getRotationPosition();
    this.values.rotationBrakeIsActive = getRotationBrakeIsActive();
    this.values.rotationVoltage = getRotationVoltage();

    // Copy our values up to the caller
    values = this.values;
  }

  @Override
  public void setExtensionPosition(double lengthMeters) {
    double rotations =
        Conversions.General.toRotations(lengthMeters, Extension.DISTANCE_PER_ROTATION);

    double ticks = Conversions.TalonFX.Position.fromRotations(rotations, Extension.GEAR_RATIO);

    extensionMotor.setSelectedSensorPosition(ticks);
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {}

  @Override
  public void setExtensionVoltage(double volts) {
    if (getExtensionBrakeIsActive()) {
      extensionMotor.disable();
      return;
    }

    volts +=
        Extension.FEEDFORWARD.calculateTelescoping(
            Rotation2d.fromRotations(values.rotationAngleRotations));

    extensionMotor.setVoltage(volts);
  }

  @Override
  public void setExtensionBrake(boolean isActive) {
    extensionBrake.set(!isActive);
  }

  @Override
  public void setExtensionDisabled() {
    setExtensionVoltage(0);
  }

  @Override
  public void setRotationPosition(double angleRotations) {
    double ticks = Conversions.TalonFX.Position.fromRotations(angleRotations, Rotation.GEAR_RATIO);
    rotationMotor.setSelectedSensorPosition(ticks);
  }

  @Override
  public void setRotationSetpoint(double angleRotations) {}

  @Override
  public void setRotationVoltage(double volts) {
    if (getRotationBrakeIsActive()) {
      rotationMotor.disable();
      return;
    }

    volts +=
        Rotation.FEEDFORWARD.calculatePivot(
            Rotation2d.fromRotations(values.rotationAngleRotations),
            ArmPosition.fromSensorValues(values.extensionLengthMeters, 0).getLeverLength());

    rotationMotor.setVoltage(volts);
  }

  @Override
  public void setRotationBrake(boolean isActive) {
    rotationBrake.set(!isActive);
  }

  @Override
  public void setRotationDisabled() {
    setRotationVoltage(0);
  }

  protected boolean getExtensionBrakeIsActive() {
    return !extensionBrake.get();
  }

  protected double getExtensionPosition() {
    double rotations =
        Conversions.TalonFX.Position.toRotations(
            extensionMotor.getSelectedSensorPosition(), Extension.GEAR_RATIO);

    return Conversions.General.toMeters(rotations, Constants.Arm.Extension.DISTANCE_PER_ROTATION);
  }

  protected double getExtensionVoltage() {
    return extensionMotor.getMotorOutputVoltage();
  }

  protected boolean getRotationBrakeIsActive() {
    return !rotationBrake.get();
  }

  protected double getRotationPosition() {
    return Conversions.TalonFX.Position.toRotations(
        rotationMotor.getSelectedSensorPosition(), Rotation.GEAR_RATIO);
  }

  protected double getRotationVoltage() {
    return rotationMotor.getMotorOutputVoltage();
  }
}
