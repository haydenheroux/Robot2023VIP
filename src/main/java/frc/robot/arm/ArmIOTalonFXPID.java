package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;

public class ArmIOTalonFXPID extends ArmIOTalonFXBase {

  private final PIDController extensionPID, rotationPID;

  public ArmIOTalonFXPID() {
    super();

    extensionPID = new PIDController(Extension.PID.KP, 0, 0);
    rotationPID = new PIDController(Rotation.PID.KP, 0, 0);
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {
    double volts = extensionPID.calculate(getExtensionPosition(), lengthMeters);
    setExtensionVoltage(volts);
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {
    double volts = rotationPID.calculate(getRotationPosition(), angleRadians);
    setRotationVoltage(volts);
  }
}
