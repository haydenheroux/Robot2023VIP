package frc.lib.math;

import java.util.function.BooleanSupplier;

public class SwitchableConstant {

  private final BooleanSupplier condition;

  private final double truthy;
  private final double falsy;

  public SwitchableConstant(BooleanSupplier condition, double truthy, double falsy) {
    this.condition = condition;

    this.truthy = truthy;
    this.falsy = falsy;
  }

  public double get() {
    return condition.getAsBoolean() ? truthy : falsy;
  }

  public double get(boolean condition) {
    return condition ? truthy : falsy;
  }
}
