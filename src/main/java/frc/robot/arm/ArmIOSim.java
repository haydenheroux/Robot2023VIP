// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

  private double extensionLengthMeters;
  private boolean extensionBrakeIsActive;

  private double rotationAngleRadians;
  private boolean rotationBrakeIsActive;

  private final double kMetersPerVolt = 0.0025;

  private final DCMotor simMotor =
      new DCMotor(Constants.NOMINAL_VOLTAGE, 4.69, 2.57, 1.5, 668.1120369, 1);

  private final double simLength = 1.0;
  private final SingleJointedArmSim rotationSim =
      new SingleJointedArmSim(
          simMotor,
          Constants.Arm.Rotation.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(simLength, Constants.Arm.MASS),
          simLength,
          Constants.Arm.Rotation.MIN_ANGLE,
          Constants.Arm.Rotation.MAX_ANGLE,
          Constants.Arm.MASS,
          true);

  private final PIDController extensionPID = new PIDController(Constants.Arm.Extension.PID.KP, 0, 0);

  private final PIDController rotationPID = new PIDController(Constants.Arm.Rotation.PID.KP, 0, 0);
  private final ArmFeedforward rotationFeedforward = new ArmFeedforward(0, 1.5, 0);

  public ArmIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(ArmIOValues values) {
    values.extensionLengthMeters = extensionLengthMeters;
    values.extensionBrakeIsActive = extensionBrakeIsActive;

    values.rotationAngleRadians = rotationAngleRadians;
    values.rotationBrakeIsActive = rotationBrakeIsActive;
  }

  @Override
  public void setExtensionPosition(double lengthMeters) {
    extensionLengthMeters = lengthMeters;
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {
    if (extensionBrakeIsActive) return; // Stop motor

    double volts = extensionPID.calculate(extensionLengthMeters, lengthMeters);
    setExtensionVoltage(volts);
  }

  @Override
  public void setExtensionVoltage(double volts) {
    if (extensionBrakeIsActive) return; // Stop motor

    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);

    extensionLengthMeters += volts * kMetersPerVolt;
  }

  @Override
  public void setExtensionBrake(boolean isActive) {
    extensionBrakeIsActive = isActive;
  }

  @Override
  public void setExtensionDisabled() {}

  @Override
  public void setRotationPosition(double angleRadians) {
    rotationAngleRadians = angleRadians;
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {
    if (rotationBrakeIsActive) return; // Stop motor

    double volts = rotationPID.calculate(rotationAngleRadians, angleRadians);
    volts = volts + rotationFeedforward.calculate(angleRadians, 0);
    setRotationVoltage(volts);
  }

  @Override
  public void setRotationVoltage(double volts) {
    if (rotationBrakeIsActive) return; // Stop motor

    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);

    rotationSim.setInput(volts / Constants.NOMINAL_VOLTAGE * RobotController.getBatteryVoltage());
    rotationSim.update(Constants.LOOP_TIME);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rotationSim.getCurrentDrawAmps()));
    rotationAngleRadians = rotationSim.getAngleRads();
  }

  @Override
  public void setRotationBrake(boolean isActive) {
    rotationBrakeIsActive = isActive;
  }

  @Override
  public void setRotationDisabled() {}
}
