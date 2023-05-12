package frc.robot.intake;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SideIntakeIOSim implements SideIntakeIO {

  private final BooleanSupplier isHolding;
  private final DoubleSupplier holdingCurrent;

  private double bottomMotorCurrentAmps = 0.0;
  private double topMotorCurrentAmps = 0.0;

  private final double kResistance = 1.0;

  public SideIntakeIOSim() {
    NetworkTable clawSim = NetworkTableInstance.getDefault().getTable("sideIntakeSim");

    BooleanTopic _isHolding = clawSim.getBooleanTopic("isHolding");
    _isHolding.publish().setDefault(false);
    isHolding = _isHolding.subscribe(false);

    DoubleTopic _holdingCurrent = clawSim.getDoubleTopic("holdingCurrent");
    _holdingCurrent.publish().setDefault(0.0);
    holdingCurrent = _holdingCurrent.subscribe(0.0);
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(SideIntakeIOValues values) {
    values.bottomMotorCurrentAmps = bottomMotorCurrentAmps;
    values.topMotorCurrentAmps = topMotorCurrentAmps;
  }

  @Override
  public void setBottomMotorVoltage(double volts) {
    if (volts == 0.0) {
      bottomMotorCurrentAmps = 0.0;
    } else if (isHolding.getAsBoolean()) {
      bottomMotorCurrentAmps = holdingCurrent.getAsDouble();
    } else {
      bottomMotorCurrentAmps = volts * kResistance;
    }
  }

  @Override
  public void setBottomMotorDisabled() {
    setBottomMotorVoltage(0.0);
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    if (volts == 0.0) {
      topMotorCurrentAmps = 0.0;
    } else if (isHolding.getAsBoolean()) {
      topMotorCurrentAmps = holdingCurrent.getAsDouble();
    } else {
      topMotorCurrentAmps = volts * kResistance;
    }
  }

  @Override
  public void setTopMotorDisabled() {
    setTopMotorVoltage(0.0);
  }
}
