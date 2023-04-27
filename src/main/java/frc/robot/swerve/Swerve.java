// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.TelemetryOutputter;

public class Swerve extends SubsystemBase implements TelemetryOutputter {
  // Singleton instance
  private static Swerve instance = null;

  /** Creates a new Swerve. */
  private Swerve() {}

  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initializeDashboard() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    
  }
}
