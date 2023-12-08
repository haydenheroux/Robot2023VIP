package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.MK4I;

/** Implements steer motor behaviors for a simulated steer motor. */
public class SteerMotorIOSim implements SteerMotorIO {

  private double angleRotations, omegaRotationsPerSecond;

  private final FlywheelSim motorSim =
      new FlywheelSim(DCMotor.getFalcon500(1), MK4I.STEER_RATIO, 0.004);

  /* Position feedback controller. Outputs voltages to correct positional error (in rotations) of this steer motor. */
  private final PIDController angleController = new PIDController(16, 0, 0);

  /** Constructs a new simulated steer motor. */
  public SteerMotorIOSim() {
    // Emulates AbsoluteSensorRangeValue.Unsigned_0To1 azimuth encoder configuration
    angleController.enableContinuousInput(0, 1);
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(SteerMotorValues values) {
    angleRotations += omegaRotationsPerSecond * Constants.LOOP_TIME;

    values.angleRotations = angleRotations;
    values.omegaRotationsPerSecond = omegaRotationsPerSecond;
  }

  @Override
  public void setPosition(double angleRotations) {
    this.angleRotations = angleRotations;
  }

  @Override
  public void setSetpoint(double angleRotations) {
    motorSim.update(Constants.LOOP_TIME);

    double volts = angleController.calculate(this.angleRotations, angleRotations);

    motorSim.setInputVoltage(volts);

    omegaRotationsPerSecond = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());
  }
}
