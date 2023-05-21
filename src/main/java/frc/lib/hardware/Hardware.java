package frc.lib.hardware;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.Ports;

public class Hardware {

  public static Solenoid getSolenoid(int channel) {
    return new Solenoid(Ports.PNEUMATICS_MODULE, Ports.PNEUMATICS_MODULE_TYPE, channel);
  }
}
