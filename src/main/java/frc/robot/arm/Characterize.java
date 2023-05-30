package frc.robot.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm.Selector;

public class Characterize extends CommandBase {
    
  private final Arm arm;
  private final Selector selector;

  private final DoubleSupplier volts;

  public Characterize(Arm arm, Selector selector, String key) {
    addRequirements(arm);

    this.arm = arm;
    this.selector = selector;

    NetworkTable table = NetworkTableInstance.getDefault().getTable(key);

    table.getDoubleTopic("volts").publish().setDefault(0.0);
    volts = table.getDoubleTopic("volts").subscribe(0.0);
  }

  @Override
  public void initialize() {
    arm.unlock(selector);
  }

  @Override
  public void execute() {
    arm.setVoltage(selector, volts.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(selector);
    arm.lock(selector);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
