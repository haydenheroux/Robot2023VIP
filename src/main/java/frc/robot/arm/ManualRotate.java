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
    arm.unlock(Selector.kRotation);
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();

    boolean isLeavingBounds = isLeavingBounds(percent);
    boolean isIntersectingGrid = arm.isIntersectingGrid() && percent < 0;

    if (isLeavingBounds || isIntersectingGrid) {
      arm.disable(Selector.kRotation);
      arm.lock(Selector.kRotation);
    } else {
      arm.unlock(Selector.kExtension);
      double volts = percent * Constants.NOMINAL_VOLTAGE;
      arm.setVoltage(Selector.kRotation, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Selector.kRotation);
    arm.lock(Selector.kRotation);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isLeavingBounds(double percent) {
    boolean aboveMax = arm.rotationIsAtMax() && percent > 0;
    boolean belowMin = arm.rotationIsAtMin() && percent < 0;
    return aboveMax || belowMin;
  }
}
