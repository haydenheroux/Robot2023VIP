package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Constants.Pneumatics;

public class ArmIOTalonFXBase implements ArmIO {

  protected final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  protected final WPI_TalonFX extensionMotor, rotationMotor;
  private final Solenoid extensionBrake, rotationBrake;

  public ArmIOTalonFXBase() {
    extensionMotor = new WPI_TalonFX(Extension.CAN_ID);
    extensionBrake =
        new Solenoid(Pneumatics.CAN_ID, Pneumatics.MODULE_TYPE, Extension.BRAKE_CHANNEL);

    rotationMotor = new WPI_TalonFX(Rotation.CAN_ID);
    rotationBrake = new Solenoid(Pneumatics.CAN_ID, Pneumatics.MODULE_TYPE, Rotation.BRAKE_CHANNEL);
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

    this.values.rotationAngleRadians = getRotationPosition();
    this.values.rotationBrakeIsActive = getRotationBrakeIsActive();
    this.values.rotationVoltage = getRotationVoltage();

    // Copy our values up to the caller
    values = this.values;
  }

  @Override
  public void setExtensionPosition(double lengthMeters) {
    double ticks =
        Conversions.TalonFX.Position.fromMeters(
            lengthMeters, Extension.DISTANCE_PER_REVOLUTION, Extension.GEAR_RATIO);
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

    volts += Extension.Feedforward.KS;

    volts += ExtensionRotationFeedforward.calculateExtensionG(ArmPosition.fromValues(values));

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
  public void setRotationPosition(double angleRadians) {
    double ticks = Conversions.TalonFX.Position.fromRadians(angleRadians, Rotation.GEAR_RATIO);
    rotationMotor.setSelectedSensorPosition(ticks);
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {}

  @Override
  public void setRotationVoltage(double volts) {
    if (getRotationBrakeIsActive()) {
      rotationMotor.disable();
      return;
    }

    volts += Rotation.Feedforward.SPRING_VOLTAGE;

    volts += Rotation.Feedforward.KS;

    volts += ExtensionRotationFeedforward.calculateRotationG(ArmPosition.fromValues(values));

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
    return Conversions.TalonFX.Position.toMeters(
        extensionMotor.getSelectedSensorPosition(),
        Constants.Arm.Extension.DISTANCE_PER_REVOLUTION,
        Constants.Arm.Extension.GEAR_RATIO);
  }

  protected double getExtensionVoltage() {
    return extensionMotor.getMotorOutputVoltage();
  }

  protected boolean getRotationBrakeIsActive() {
    return !rotationBrake.get();
  }

  protected double getRotationPosition() {
    return Conversions.TalonFX.Position.toRadians(
        rotationMotor.getSelectedSensorPosition(), Rotation.GEAR_RATIO);
  }

  protected double getRotationVoltage() {
    return rotationMotor.getMotorOutputVoltage();
  }
}
