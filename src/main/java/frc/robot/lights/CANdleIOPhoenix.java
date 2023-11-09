package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.Ports;

public class CANdleIOPhoenix implements CANdleIO {

  private final CANdle candle;

  private int red, green, blue;

  public CANdleIOPhoenix() {
    candle = new CANdle(Ports.CANDLE);
  }

  @Override
  public void configure() {
    CANdleConfiguration config = new CANdleConfiguration();

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;

    candle.configAllSettings(config);
  }

  @Override
  public void updateValues(CANdleIOValues values) {
    values.red = this.red;
    values.green = this.green;
    values.blue = this.blue;
  }

  @Override
  public void setColor(int red, int green, int blue) {
    candle.setLEDs(red, green, blue);

    this.red = red;
    this.green = green;
    this.blue = blue;
  }
}
