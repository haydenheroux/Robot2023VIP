// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ClawIOSim implements ClawIO {

  private final DCMotor simMotor = DCMotor.getVex775Pro(1);

  private final double flywheelEstimatedMOI =
      1 * Constants.Intake.Claw.MASS * Math.pow(Constants.Intake.Claw.RADIUS, 2);
  private final FlywheelSim flywheelSim =
      new FlywheelSim(simMotor, Constants.Intake.Claw.GEAR_RATIO, flywheelEstimatedMOI);

  @Override
  public void configure() {}

  @Override
  public void updateValues(ClawIOValues values) {
    values.motorCurrentAmps = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setMotorVoltage(double volts) {
    flywheelSim.setInputVoltage(
        (volts / Constants.NOMINAL_VOLTAGE) * RobotController.getBatteryVoltage());
    flywheelSim.update(Constants.LOOP_TIME);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
  }

  @Override
  public void setMotorDisabled() {
    setMotorVoltage(0);
  }
}
