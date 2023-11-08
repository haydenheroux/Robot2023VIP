package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public interface CANdleIO {

  public static class CANdleIOValues {
    public double red = 0.0;
    public double green = 0.0;
    public double blue = 0.0;
  }

  public void configure();

  public void updateValues(CANdleIOValues values);

  public void setColor(Color color);
}
