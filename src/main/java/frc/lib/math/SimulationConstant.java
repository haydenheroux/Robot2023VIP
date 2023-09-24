package frc.lib.math;

import frc.robot.Robot;

public class SimulationConstant extends SwitchableConstant {

  public SimulationConstant(double simulated, double real) {
    super(Robot::isSimulation, simulated, real);
  }

  public double getSimulated() {
    return get(true);
  }

  public double getReal() {
    return get(false);
  }
}
