package frc.robot.intake;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClawIOSim implements ClawIO {

  private double simMotorCurrent;

  public final BooleanSupplier isHoldingCone, isHoldingCube;
  public final DoubleSupplier holdingConeCurrent, holdingCubeCurrent;

  private final double kResistance = 1.0;

  public ClawIOSim() {
    NetworkTable clawSim = NetworkTableInstance.getDefault().getTable("clawSim");

    BooleanTopic _isHoldingCone = clawSim.getBooleanTopic("isHoldingCone");
    _isHoldingCone.publish().setDefault(false);
    isHoldingCone = clawSim.getBooleanTopic("isHoldingCone").subscribe(false);

    BooleanTopic _isHoldingCube = clawSim.getBooleanTopic("isHoldingCube");
    _isHoldingCube.publish().setDefault(false);
    isHoldingCube = clawSim.getBooleanTopic("isHoldingCube").subscribe(false);

    DoubleTopic _holdingConeCurrent = clawSim.getDoubleTopic("holdingConeCurrent");
    _holdingConeCurrent.publish().setDefault(0.0);
    holdingConeCurrent = clawSim.getDoubleTopic("holdingConeCurrent").subscribe(0.0);

    DoubleTopic _holdingCubeCurrent = clawSim.getDoubleTopic("holdingCubeCurrent");
    _holdingCubeCurrent.publish().setDefault(0.0);
    holdingCubeCurrent = clawSim.getDoubleTopic("holdingCubeCurrent").subscribe(0.0);
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(ClawIOValues values) {
    values.motorCurrentAmps = simMotorCurrent;
  }

  @Override
  public void setMotorVoltage(double volts) {
    if (volts == 0.0) {
      simMotorCurrent = 0.0;
    } else if (isHoldingCone.getAsBoolean()) {
      simMotorCurrent = holdingConeCurrent.getAsDouble();
    } else if (isHoldingCube.getAsBoolean()) {
      simMotorCurrent = holdingCubeCurrent.getAsDouble();
    } else {
      simMotorCurrent = volts * kResistance;
    }
  }

  @Override
  public void setMotorDisabled() {
    setMotorVoltage(0);
  }
}
