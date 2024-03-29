package frc.robot.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.hardware.ConfigurationApplier;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Pivot;
import frc.robot.Constants.Arm.Telescoping;
import frc.robot.Constants.Ports;

public class ArmIOTalonFXBase implements ArmIO {

  protected final ArmIO.ArmIOValues values = new ArmIO.ArmIOValues();

  protected final TalonFX telescopingMotor, pivotMotor;
  private final Solenoid telescopingBrake, pivotBrake;

  public ArmIOTalonFXBase() {
    telescopingMotor = new TalonFX(Ports.TELESCOPING_MOTOR);
    telescopingBrake =
        new Solenoid(
            Ports.PNEUMATICS_MODULE, Ports.PNEUMATICS_MODULE_TYPE, Ports.TELESCOPING_BRAKE);

    pivotMotor = new TalonFX(Ports.PIVOT_MOTOR);
    pivotBrake =
        new Solenoid(Ports.PNEUMATICS_MODULE, Ports.PNEUMATICS_MODULE_TYPE, Ports.PIVOT_BRAKE);
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Telescoping.CONFIG, telescopingMotor);
    ConfigurationApplier.apply(Pivot.CONFIG, pivotMotor);
  }

  @Override
  public void updateValues(ArmIOValues values) {
    this.values.telescopingLengthMeters =
        Conversions.General.toMeters(
            telescopingMotor.getPosition().getValue(),
            Constants.Arm.Telescoping.DISTANCE_PER_ROTATION);
    this.values.telescopingBrakeIsActive = !telescopingBrake.get();
    this.values.telescopingVoltage = telescopingMotor.getSupplyVoltage().getValue();

    this.values.pivotAngleRotations = pivotMotor.getPosition().getValue();
    this.values.pivotBrakeIsActive = !pivotBrake.get();
    this.values.pivotVoltage = pivotMotor.getSupplyVoltage().getValue();

    // Copy our values up to the caller
    values = this.values;
  }

  @Override
  public void setTelescopingPosition(double lengthMeters) {
    double rotations =
        Conversions.General.toRotations(lengthMeters, Telescoping.DISTANCE_PER_ROTATION);

    telescopingMotor.setPosition(rotations);
  }

  @Override
  public void setTelescopingSetpoint(double lengthMeters) {}

  @Override
  public void setTelescopingVoltage(double volts) {
    if (this.values.telescopingBrakeIsActive) {
      telescopingMotor.disable();
      return;
    }

    volts += Telescoping.FEEDFORWARD.calculateTelescoping(ArmPosition.fromValues(values));

    telescopingMotor.setVoltage(volts);
  }

  @Override
  public void setTelescopingBrake(boolean isActive) {
    telescopingBrake.set(!isActive);
  }

  @Override
  public void setTelescopingDisabled() {
    setTelescopingVoltage(0);
  }

  @Override
  public void setPivotPosition(double angleRotations) {
    pivotMotor.setPosition(angleRotations);
  }

  @Override
  public void setPivotSetpoint(double angleRotations) {}

  @Override
  public void setPivotVoltage(double volts) {
    if (this.values.pivotBrakeIsActive) {
      pivotMotor.disable();
      return;
    }

    volts += Pivot.FEEDFORWARD.calculatePivot(ArmPosition.fromValues(values));

    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setPivotBrake(boolean isActive) {
    pivotBrake.set(!isActive);
  }

  @Override
  public void setPivotDisabled() {
    setPivotVoltage(0);
  }
}
