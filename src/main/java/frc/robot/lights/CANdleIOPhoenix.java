package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Ports;

public class CANdleIOPhoenix implements CANdleIO {

  private final CANdle candle;

  private Color color = Color.kBlack;

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
    values.red = color.red;
    values.green = color.green;
    values.blue = color.blue;
  }

  @Override
  public void setColor(Color color) {
    final int red = (int) color.red * 255;
    final int green = (int) color.green * 255;
    final int blue = (int) color.blue * 255;

    candle.setLEDs(red, green, blue);

    this.color = color;
  }
}
