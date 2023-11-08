package frc.robot.lights;

public interface CANdleIO {

  public static class CANdleIOValues {
    public int red = 0;
    public int green = 0;
    public int blue = 0;
  }

  public void configure();

  public void updateValues(CANdleIOValues values);

  public void setColor(int red, int green, int blue);
}
