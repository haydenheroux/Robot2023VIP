package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Pivot;
import frc.robot.Constants.Arm.Telescoping;
import frc.robot.Constants.Physical;

public class ArmIOSim implements ArmIO {

  private double telescopingLengthMeters;
  private boolean telescopingBrakeIsActive = true;

  private double pivotAngleRotations;
  private boolean pivotBrakeIsActive = true;

  private final double kMetersPerVolt = 0.0025;
  private final double kMetersPerGravity = -0.005;

  private final DCMotor simMotor =
      new DCMotor(Constants.NOMINAL_VOLTAGE, 4.69, 2.57, 1.5, 668.1120369, 1);

  // FIXME Simulation assumes constant length
  private final double fakeSimLength = ArmPosition.STOW.getLength();
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          simMotor,
          Pivot.RATIO,
          SingleJointedArmSim.estimateMOI(fakeSimLength, Physical.ARM_MASS),
          fakeSimLength,
          Pivot.MIN_ANGLE.getRadians(),
          Pivot.MAX_ANGLE.getRadians(),
          Constants.Physical.ARM_MASS,
          true);

  private final PIDController telescopingPID = new PIDController(Telescoping.PID.KP, 0, 0);
  private final PIDController pivotPID = new PIDController(Pivot.PID.KP, 0, 0);

  private double telescopingVoltage = 0.0;
  private double pivotVoltage = 0.0;

  public ArmIOSim() {}

  @Override
  public void configure() {}

  @Override
  public void updateValues(ArmIOValues values) {
    double lengthMeters = telescopingLengthMeters;

    if (!telescopingBrakeIsActive) {
      telescopingLengthMeters += telescopingVoltage * kMetersPerVolt;

      double metersPulledByGravity =
          Math.sin(Units.rotationsToRadians(pivotAngleRotations)) * kMetersPerGravity;

      boolean atMin = telescopingLengthMeters < Telescoping.MIN_LENGTH;
      boolean belowMin = atMin && metersPulledByGravity < 0;

      if (!belowMin) {
        telescopingLengthMeters += metersPulledByGravity;
      }
    }

    values.telescopingBrakeIsActive = telescopingBrakeIsActive;
    values.telescopingLengthMeters = telescopingLengthMeters;
    values.telescopingVelocityMetersPerSecond = (telescopingLengthMeters - lengthMeters) / Constants.LOOP_TIME;
    values.telescopingVoltage = telescopingVoltage;

    double angleRotations = pivotAngleRotations;

    if (!pivotBrakeIsActive) {
      pivotSim.setInput(
          pivotVoltage / Constants.NOMINAL_VOLTAGE * RobotController.getBatteryVoltage());
      pivotSim.update(Constants.LOOP_TIME);

      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

      pivotAngleRotations = Units.radiansToRotations(pivotSim.getAngleRads());
    }

    values.pivotAngleRotations = pivotAngleRotations;
    values.pivotOmegaRotationsPerSecond = (pivotAngleRotations - angleRotations) / Constants.LOOP_TIME;
    values.pivotBrakeIsActive = pivotBrakeIsActive;
    values.pivotVoltage = pivotVoltage;
  }

  @Override
  public void setTelescopingPosition(double lengthMeters) {
    telescopingLengthMeters = lengthMeters;
  }

  @Override
  public void setTelescopingSetpoint(double lengthMeters) {
    double volts = telescopingPID.calculate(telescopingLengthMeters, lengthMeters);
    setTelescopingVoltage(volts);
  }

  @Override
  public void setTelescopingVoltage(double volts) {
    volts +=
        Telescoping.FEEDFORWARD.calculateTelescoping(ArmPosition.fromValues(fakeSimLength - Physical.LENGTH_OFFSET, pivotAngleRotations));

    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);
    telescopingVoltage = volts;
  }

  @Override
  public void setTelescopingBrake(boolean isActive) {
    telescopingBrakeIsActive = isActive;
  }

  @Override
  public void setTelescopingDisabled() {
    telescopingVoltage = 0;
  }

  @Override
  public void setPivotPosition(double angleRotations) {
    pivotAngleRotations = angleRotations;
  }

  @Override
  public void setPivotSetpoint(double angleRotations) {
    double volts = pivotPID.calculate(pivotAngleRotations, angleRotations);
    setPivotVoltage(volts);
  }

  @Override
  public void setPivotVoltage(double volts) {
    volts +=
        Pivot.FEEDFORWARD.calculatePivot(ArmPosition.fromValues(fakeSimLength - Physical.LENGTH_OFFSET, pivotAngleRotations));

    volts = MathUtil.clamp(volts, -Constants.NOMINAL_VOLTAGE, Constants.NOMINAL_VOLTAGE);
    pivotVoltage = volts;
  }

  @Override
  public void setPivotBrake(boolean isActive) {
    pivotBrakeIsActive = isActive;
  }

  @Override
  public void setPivotDisabled() {
    pivotVoltage = 0;
  }
}
