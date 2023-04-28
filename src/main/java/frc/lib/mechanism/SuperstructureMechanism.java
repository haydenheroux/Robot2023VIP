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
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;

public class SuperstructureMechanism {

  private static SuperstructureMechanism instance = null;

  private double metersToMechanism(double meters) {
    return meters * 20;
  }

  private double metersToMechanismThickness(double meters) {
    return 8 * metersToMechanism(meters);
  }

  private final Color8Bit kDefaultColor = new Color8Bit(Color.kGray);

  private final Color8Bit kLockedColor = kDefaultColor;
  private final Color8Bit kUnlockedColor = new Color8Bit(Color.kHotPink);
  private final Color8Bit kConstraintColor = new Color8Bit(Color.kOrange);

  private final Color8Bit kIntakeDisabledColor = kDefaultColor;
  private final Color8Bit kIntakeHoldingConeColor = new Color8Bit(Color.kYellow);
  private final Color8Bit kIntakeHoldingCubeColor = new Color8Bit(Color.kPurple);
  private final Color8Bit kIntakeAcceptingColor = new Color8Bit(Color.kLimeGreen);
  private final Color8Bit kIntakeEjectingColor = new Color8Bit(Color.kTomato);

  private final Mechanism2d mechanism;

  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armRotation, armExtension, armClaw;

  private final MechanismRoot2d sideIntakeRoot;
  private final MechanismLigament2d sideIntake;

  private SuperstructureMechanism() {
    mechanism =
        new Mechanism2d(
            metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH * 2),
            metersToMechanism(Constants.Arm.Constraints.MAX_HEIGHT));

    armRoot =
        mechanism.getRoot(
            "armRoot",
            metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH),
            metersToMechanism(Constants.Arm.Constraints.HEIGHT_OFFSET));

    armRotation =
        armRoot.append(
            new MechanismLigament2d(
                "armRotation",
                metersToMechanism(Constants.Arm.Constraints.STATIC_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(2)),
                kLockedColor));

    armExtension =
        armRotation.append(
            new MechanismLigament2d(
                "armExtension",
                0,
                Math.toDegrees(0),
                4 * metersToMechanism(Units.inchesToMeters(1.5)),
                kLockedColor));

    armClaw =
        armExtension.append(
            new MechanismLigament2d(
                "claw",
                metersToMechanism(Constants.Arm.Constraints.CLAW_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(4.717)),
                kIntakeDisabledColor));

    sideIntakeRoot =
        mechanism.getRoot(
            "sideIntakeRoot",
            metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH)
                - metersToMechanism(Units.inchesToMeters(12)) / 2,
            metersToMechanism(Units.inchesToMeters(18)));

    sideIntake =
        sideIntakeRoot.append(
            new MechanismLigament2d(
                "sideIntake",
                metersToMechanism(Units.inchesToMeters(12)),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(4)),
                kIntakeDisabledColor));
  }

  public static SuperstructureMechanism getInstance() {
    if (instance == null) {
      instance = new SuperstructureMechanism();
    }

    return instance;
  }

  public Mechanism2d getMechanism() {
    return mechanism;
  }

  private void setAngle(double angleRadians) {
    armRotation.setAngle(Math.toDegrees(angleRadians));
  }

  private void setLength(double lengthMeters) {
    armExtension.setLength(metersToMechanism(lengthMeters));
  }

  private void setBrake(Arm.LockType isLocked) {
    switch (isLocked) {
      case kBoth:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kLockedColor);
        break;
      case kExtension:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kNeither:
        armExtension.setColor(kUnlockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kRotation:
        armExtension.setColor(kUnlockedColor);
        armRotation.setColor(kLockedColor);
        break;
    }
  }

  private void setConstraint(ArmPosition position) {
    if (ArmConstraintsSolver.isWithinRuleZone(position) == false) {
      armExtension.setColor(kConstraintColor);
    }
  }

  public void updateArm(ArmPosition position, Arm.LockType isLocked) {
    setAngle(position.rotationAngleRadians);
    setLength(position.extensionLengthMeters);
    setBrake(isLocked);
    setConstraint(position);
  }

  public void updateClaw(Claw.State state) {
    switch (state) {
      case kAccepting:
        armClaw.setColor(kIntakeAcceptingColor);
        break;
      case kDisabled:
        armClaw.setColor(kIntakeDisabledColor);
        break;
      case kEjecting:
        armClaw.setColor(kIntakeEjectingColor);
        break;
      case kHoldingCone:
        armClaw.setColor(kIntakeHoldingConeColor);
        break;
      case kHoldingCube:
        armClaw.setColor(kIntakeHoldingCubeColor);
        break;
    }
  }

  public void updateSideIntake(SideIntake.State state) {
    switch (state) {
      case kAccepting:
        sideIntake.setColor(kIntakeAcceptingColor);
        break;
      case kDisabled:
        sideIntake.setColor(kIntakeDisabledColor);
        break;
      case kEjecting:
        sideIntake.setColor(kIntakeEjectingColor);
        break;
      case kHolding:
        sideIntake.setColor(kIntakeHoldingConeColor);
        break;
    }
  }
}
