// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanism;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstraintsSolver;
import frc.robot.arm.ArmPosition;

public class ArmMechanism {

  private static ArmMechanism instance = null;

  private double metersToMechanism(double meters) {
    return meters * 20;
  }

  private double metersToMechanismThickness(double meters) {
    return 8 * metersToMechanism(meters);
  }

  private final Color8Bit kLockedColor = new Color8Bit(Color.kGray);
  private final Color8Bit kUnlockedColor = new Color8Bit(Color.kRed);
  private final Color8Bit kClawColor = new Color8Bit(Color.kLimeGreen);
  private final Color8Bit kConstraintColor = new Color8Bit(Color.kOrange);

  private final Mechanism2d mech2d;
  private final MechanismRoot2d root;
  private final MechanismLigament2d armRotatingLigament, armExtendingLigament, clawLigament;

  private ArmMechanism() {
    mech2d =
        new Mechanism2d(
            metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH * 2),
            metersToMechanism(Constants.Arm.Constraints.MAX_HEIGHT));
    root =
        mech2d.getRoot(
            "root",
            metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH),
            metersToMechanism(Constants.Arm.Constraints.HEIGHT_OFFSET));

    armRotatingLigament =
        root.append(
            new MechanismLigament2d(
                "Static",
                metersToMechanism(Constants.Arm.Constraints.STATIC_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(2)),
                new Color8Bit(Color.kGray)));

    armExtendingLigament =
        armRotatingLigament.append(
            new MechanismLigament2d(
                "Extending",
                0,
                Math.toDegrees(0),
                4 * metersToMechanism(Units.inchesToMeters(1.5)),
                new Color8Bit(Color.kGray)));

    clawLigament =
        armExtendingLigament.append(
            new MechanismLigament2d(
                "Claw",
                metersToMechanism(Constants.Arm.Constraints.CLAW_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(4.717)),
                new Color8Bit(Color.kLimeGreen)));
  }

  public static ArmMechanism getInstance() {
    if (instance == null) {
      instance = new ArmMechanism();
    }

    return instance;
  }

  public Mechanism2d getMechanism() {
    return mech2d;
  }

  private void setAngle(double angleRadians) {
    armRotatingLigament.setAngle(Math.toDegrees(angleRadians));
  }

  private void setLength(double lengthMeters) {
    armExtendingLigament.setLength(metersToMechanism(lengthMeters));
  }

  private void setBrake(Arm.LockType isLocked) {
    switch (isLocked) {
      case kBoth:
        armExtendingLigament.setColor(kLockedColor);
        armRotatingLigament.setColor(kLockedColor);
        break;
      case kExtension:
        armExtendingLigament.setColor(kLockedColor);
        armRotatingLigament.setColor(kUnlockedColor);
        break;
      case kNeither:
        armExtendingLigament.setColor(kUnlockedColor);
        armRotatingLigament.setColor(kUnlockedColor);
        break;
      case kRotation:
        armExtendingLigament.setColor(kUnlockedColor);
        armRotatingLigament.setColor(kLockedColor);
        break;
    }
  }

  private void setConstraint(ArmPosition position) {
    if (ArmConstraintsSolver.isWithinRuleZone(position) == false) {
      armExtendingLigament.setColor(new Color8Bit(Color.kOrange));
    }
  }

  public void update(ArmPosition position, Arm.LockType isLocked) {
    setAngle(position.rotationAngleRadians);
    setLength(position.extensionLengthMeters);
    setBrake(isLocked);
    setConstraint(position);
  }
}
