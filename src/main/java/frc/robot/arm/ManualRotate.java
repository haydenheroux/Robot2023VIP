package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.arm.Arm.Selector;
import java.util.function.DoubleSupplier;

public class ManualRotate extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier percentSupplier;

  public ManualRotate(Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(arm);

    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void initialize() {
    arm.unlock(Selector.kPivot);
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();

    boolean isLeavingBounds = isLeavingBounds(percent);
    boolean isIntersectingGrid = arm.isIntersectingGrid() && percent < 0;

    if (isLeavingBounds || isIntersectingGrid) {
      arm.disable(Selector.kPivot);
      arm.lock(Selector.kPivot);
    } else {
      arm.unlock(Selector.kPivot);
      double volts = percent * Constants.NOMINAL_VOLTAGE;
      arm.setVoltage(Selector.kPivot, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Selector.kPivot);
    arm.lock(Selector.kPivot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isLeavingBounds(double percent) {
    boolean aboveMax = arm.getPosition().angleAtMax() && percent > 0;
    boolean belowMin = arm.getPosition().angleAtMin() && percent < 0;
    return aboveMax || belowMin;
  }
}
