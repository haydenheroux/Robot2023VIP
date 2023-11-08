package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public class CANdleIOSim implements CANdleIO {

  private Color color = Color.kBlack;

  @Override
  public void configure() {}

  @Override
  public void updateValues(CANdleIOValues values) {
    values.red = color.red;
    values.green = color.green;
    values.blue = color.blue;
  }

  @Override
  public void setColor(Color color) {
    this.color = color;
  }
}
