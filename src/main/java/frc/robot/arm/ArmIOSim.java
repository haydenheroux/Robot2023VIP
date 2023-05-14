package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;
import frc.robot.Constants.Physical;

public class ArmIOSim implements ArmIO {

  private double extensionLengthMeters;
  private boolean extensionBrakeIsActive;

  private double rotationAngleRadians;
  private boolean rotationBrakeIsActive;

  private final double kMetersPerVolt = 0.0025;
  private final double kMetersPerGravity = -0.005;

  private final DCMotor simMotor =
      new DCMotor(Constants.NOMINAL_VOLTAGE, 4.69, 2.57, 1.5, 668.1120369, 1);

  // FIXME Simulation assumes constant length
  private final double fakeSimLength = Constants.Arm.Positions.STOW.getLength();
  private final SingleJointedArmSim rotationSim =
      new SingleJointedArmSim(
          simMotor,
          Rotation.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(fakeSimLength, Physical.ARM_MASS),
          fakeSimLength,
          Rotation.MIN_ANGLE.getRadians(),
          Rotation.MAX_ANGLE.getRadians(),
          Constants.Physical.ARM_MASS,
          true);

  private final ExtensionRotationFeedforward feedforward = new ExtensionRotationFeedforward();

  private final PIDController extensionPID = new PIDController(Extension.PID.KP, 0, 4);
  private final PIDController rotationPID = new PIDController(Rotation.PID.KP, 0, 0);

  private double extensionVoltage = 0.0;
  private double rotationVoltage = 0.0;

  public ArmIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(ArmIOValues values) {
    if (!extensionBrakeIsActive) {
      extensionLengthMeters += extensionVoltage * kMetersPerVolt;

      double metersPulledByGravity = Math.sin(rotationAngleRadians) * kMetersPerGravity;

      boolean atMin = extensionLengthMeters < Extension.MIN_LENGTH;
      boolean belowMin = atMin && metersPulledByGravity < 0;

      if (!belowMin) {
        extensionLengthMeters += metersPulledByGravity;
      }
    }

    values.extensionBrakeIsActive = extensionBrakeIsActive;
    values.extensionLengthMeters = extensionLengthMeters;
    values.extensionVoltage = extensionVoltage;

    if (!rotationBrakeIsActive) {
      rotationSim.setInput(
          rotationVoltage / Constants.NOMINAL_VOLTAGE * RobotController.getBatteryVoltage());
      rotationSim.update(Constants.LOOP_TIME);

      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(rotationSim.getCurrentDrawAmps()));

      rotationAngleRadians = rotationSim.getAngleRads();
    }

    values.rotationAngleRadians = rotationAngleRadians;
    values.rotationBrakeIsActive = rotationBrakeIsActive;
    values.rotationVoltage = rotationVoltage;
  }

  @Override
  public void setExtensionPosition(double lengthMeters) {
    extensionLengthMeters = lengthMeters;
  }

  @Override
  public void setExtensionSetpoint(double lengthMeters) {
    double volts = extensionPID.calculate(extensionLengthMeters, lengthMeters);
    setExtensionVoltage(volts);
  }

  @Override
  public void setExtensionVoltage(double volts) {
    volts +=
        feedforward.calculateExtensionVoltageToOvercomeGravity(
            ArmPosition.fromState(
                new Arm.State(
                    extensionLengthMeters, Rotation2d.fromRadians(rotationAngleRadians))));
    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);
    extensionVoltage = volts;
  }

  @Override
  public void setExtensionBrake(boolean isActive) {
    extensionBrakeIsActive = isActive;
  }

  @Override
  public void setExtensionDisabled() {
    setExtensionVoltage(0.0);
  }

  @Override
  public void setRotationPosition(double angleRadians) {
    rotationAngleRadians = angleRadians;
  }

  @Override
  public void setRotationSetpoint(double angleRadians) {
    double volts = rotationPID.calculate(rotationAngleRadians, angleRadians);
    setRotationVoltage(volts);
  }

  @Override
  public void setRotationVoltage(double volts) {
    volts +=
        feedforward.calculateRotationVoltageToOvercomeGravity(
            ArmPosition.fromState(
                new Arm.State(fakeSimLength, Rotation2d.fromRadians(rotationAngleRadians))));
    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);
    rotationVoltage = volts;
  }

  @Override
  public void setRotationBrake(boolean isActive) {
    rotationBrakeIsActive = isActive;
  }

  @Override
  public void setRotationDisabled() {
    // FIXME Applies feedforward
    setRotationVoltage(0.0);
  }
}
