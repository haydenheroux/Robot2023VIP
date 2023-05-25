package frc.robot.odometry;

import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class GyroIOSim implements GyroIO {

  private double yawAngleRotations = 0.0;
  private final DoubleSupplier omegaRotationsPerSecondSupplier;

  public GyroIOSim(DoubleSupplier omegaRotationsPerSecondSupplier) {
    this.omegaRotationsPerSecondSupplier = omegaRotationsPerSecondSupplier;
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(GyroIOValues values) {
    yawAngleRotations += omegaRotationsPerSecondSupplier.getAsDouble() * Constants.LOOP_TIME;

    values.rollAngleRotations = 0.0;
    values.pitchAngleRotations = 0.0;
    values.yawAngleRotations = yawAngleRotations;
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    this.yawAngleRotations = yawAngleRotations;
  }
}
