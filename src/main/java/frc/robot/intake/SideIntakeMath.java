// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Intake.SideIntake;

public class SideIntakeMath {

  /**
   * Computes the measure of the angle between beta and the normal of alpha.
   *
   * @param alphaRadians angle whose normal will be calculated.
   * @param betaRadians angle who will be measured.
   * @return the measure of the angle between beta and the normal of alpha.
   */
  private static double getAngleRelativeToNormal(double alphaRadians, double betaRadians) {
    double normalAngleRadians = alphaRadians - Units.degreesToRadians(90.0);
    return normalAngleRadians + betaRadians;
  }

  private static double getBiasFor(double alphaRadians, double betaRadians) {
    double relativeAngle = getAngleRelativeToNormal(alphaRadians, betaRadians);
    return getBiasFor(relativeAngle);
  }

  private static double getBiasFor(double relativeAngleRadians) {
    return SideIntake.Voltages.RELATIVE_BIAS * Math.sin(relativeAngleRadians);
  }

  public static double getBias() {
    return getBiasFor(SideIntake.MECHANISM_ANGLE, SideIntake.ACCEPT_ANGLE);
  }
}
