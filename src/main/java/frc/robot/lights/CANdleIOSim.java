package frc.robot.lights;

public class CANdleIOSim implements CANdleIO {

  private int red, green, blue;

  @Override
  public void configure() {}

  @Override
  public void updateValues(CANdleIOValues values) {
    values.red = this.red;
    values.green = this.green;
    values.blue = this.blue;
  }

  @Override
  public void setColor(int red, int green, int blue) {
    this.red = red;
    this.green = green;
    this.blue = blue;
  }
}
