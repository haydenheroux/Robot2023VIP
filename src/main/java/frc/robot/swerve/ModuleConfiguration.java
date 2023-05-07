// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleConfiguration {

    public final Translation2d locationRelativeToCenterMeters;

    public ModuleConfiguration() {
        locationRelativeToCenterMeters = new Translation2d();
    }
}
