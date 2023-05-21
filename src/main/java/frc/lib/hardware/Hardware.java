package frc.lib.hardware;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;

public class Hardware {
   
    public static Solenoid getSolenoid(int channel) {
        return new Solenoid(RobotMap.PNEUMATICS_MODULE, RobotMap.PNEUMATICS_MODULE_TYPE, channel);
    }

}
