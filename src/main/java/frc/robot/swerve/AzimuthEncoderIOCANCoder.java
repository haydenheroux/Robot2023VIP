package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AzimuthEncoderIOCANCoder implements AzimuthEncoderIO {

  private final CANcoder encoder;
  private final double magnetOffsetRotations;

  public AzimuthEncoderIOCANCoder(int id, String canbus, double offsetRotations) {
    encoder = new CANcoder(id, canbus);
    magnetOffsetRotations = -offsetRotations;
  }

  @Override
  public void configure() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    // TODO
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // TODO
    config.MagnetSensor.MagnetOffset = magnetOffsetRotations;

    /*
     * https://github.com/TitaniumTitans/2023ChargedUp/blob/0306f0274d170ba5cd87808f60e1d64475917b67/src/main/java/frc/robot/subsystems/swerve/module/FalconProModule.java#L201
     */
    StatusCode status;
    do {
      status = encoder.getConfigurator().apply(config);
    } while (!status.isOK());

    encoder.getAbsolutePosition().setUpdateFrequency(100);
  }

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.angleRotations = encoder.getAbsolutePosition().getValue();
  }

  /**
   * Sets the current position of the magnet to be the zero.
   *
   * <p>https://github.com/FRC3476/FRC-2023/blob/5a91e0a8a68047fc604b17d02e7192f63f010413/src/main/java/frc/subsytem/drive/DriveIOFalcon.java#L268
   */
  private void setMagnetZero() {
    MagnetSensorConfigs config = new MagnetSensorConfigs();

    encoder.getConfigurator().refresh(config);

    config.MagnetOffset = 0.0;

    encoder.getConfigurator().apply(config);
    // TODO Reduce update timeout
    encoder.getAbsolutePosition().waitForUpdate(1);

    double newOffset = -encoder.getAbsolutePosition().getValue();
    // magnetOffsetRotations = newOffset;
    config.MagnetOffset = newOffset;
    encoder.getConfigurator().apply(config);

    // TODO Reduce update timeout
    encoder.getAbsolutePosition().waitForUpdate(1);
  }
}
