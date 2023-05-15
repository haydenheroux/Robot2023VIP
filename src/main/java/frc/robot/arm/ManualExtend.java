package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.arm.Arm.Selector;
import java.util.function.DoubleSupplier;

public class ManualExtend extends CommandBase {

  private final Arm arm;
  private final DoubleSupplier percentSupplier;

  public ManualExtend(Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(arm);

    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void initialize() {
    arm.unlock(Selector.kExtension);
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();

    boolean isLeavingBounds = isLeavingBounds(percent);
    boolean isLeavingRuleZone = isLeavingRuleZone(percent);
    boolean isIntersectingGrid = arm.isIntersectingGrid() && percent > 0;
    boolean isAboveSafeExtensionAngle =
        arm.getPosition().isAbove(Constants.Arm.Positions.SAFE) && percent > 0;

    if (isLeavingBounds || isLeavingRuleZone || isIntersectingGrid || isAboveSafeExtensionAngle) {
      arm.disable(Selector.kExtension);
      arm.lock(Selector.kExtension);
    } else {
      arm.unlock(Selector.kExtension);
      double volts = percent * Constants.NOMINAL_VOLTAGE;
      arm.setVoltage(Selector.kExtension, volts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.disable(Selector.kExtension);
    arm.lock(Selector.kExtension);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isLeavingBounds(double percent) {
    boolean aboveMax = arm.extensionIsAtMax() && percent > 0;
    boolean belowMin = arm.extensionIsAtMin() && percent < 0;
    return aboveMax || belowMin;
  }

  private boolean isLeavingRuleZone(double percent) {
    return arm.isWithinRuleZone() == false && percent > 0;
  }
}
