package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Pivot;
import frc.robot.Constants.Arm.Telescoping;

public class ArmIOTalonFXPID extends ArmIOTalonFXBase {

  private final PIDController extensionPID, rotationPID;

  public ArmIOTalonFXPID() {
    super();

    extensionPID = new PIDController(Telescoping.PID.KP, 0, 0);
    rotationPID = new PIDController(Pivot.PID.KP, 0, 0);
  }

  @Override
  public void setTelescopingSetpoint(double lengthMeters) {
    double volts =
        extensionPID.calculate(
            Conversions.General.toMeters(
                telescopingMotor.getPosition().getValue(),
                Constants.Arm.Telescoping.DISTANCE_PER_ROTATION),
            lengthMeters);
    setTelescopingVoltage(volts);
  }

  @Override
  public void setPivotSetpoint(double angleRotations) {
    double volts = rotationPID.calculate(pivotMotor.getPosition().getValue(), angleRotations);
    setPivotVoltage(volts);
  }
}
