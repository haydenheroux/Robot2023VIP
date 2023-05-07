// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class GyroIOSim implements GyroIO {

  private double yawAngleRadians = 0.0;
  private final DoubleSupplier omegaRadiansPerSecondSupplier;

  public GyroIOSim(DoubleSupplier omegaRadiansPerSecondSupplier) {
    this.omegaRadiansPerSecondSupplier = omegaRadiansPerSecondSupplier;
  }

  @Override
  public void configure() {}

  @Override
  public void updateValues(GyroIOValues values) {
    yawAngleRadians += omegaRadiansPerSecondSupplier.getAsDouble() * Constants.LOOP_TIME;

    values.rollAngleRadians = 0.0;
    values.pitchAngleRadians = 0.0;
    values.yawAngleRadians = yawAngleRadians;
  }

  @Override
  public void setRollAngle(double rollAngleRadians) {}

  @Override
  public void setPitchAngle(double pitchAngleRadians) {}

  @Override
  public void setYawAngle(double yawAngleRadians) {
    this.yawAngleRadians = yawAngleRadians;
  }
}
