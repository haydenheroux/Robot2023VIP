package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

public class AzimuthEncoderIOCANCoder implements AzimuthEncoderIO {

  private final CANcoder encoder;
  private final double magnetOffset;

  public AzimuthEncoderIOCANCoder(int id, String canbus, double offsetRotations) {
    encoder = new CANcoder(id, canbus);
    magnetOffset = -offsetRotations;
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Swerve.AZIMUTH_CONFIG, encoder);

    // TODO Don't overwrite previous config
    MagnetSensorConfigs offset = new MagnetSensorConfigs();
    offset.MagnetOffset = magnetOffset;

    ConfigurationApplier.apply(offset, encoder);

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

    ConfigurationApplier.apply(config, encoder);
    // TODO Reduce update timeout
    encoder.getAbsolutePosition().waitForUpdate(1);

    double newOffset = -encoder.getAbsolutePosition().getValue();
    // magnetOffsetRotations = newOffset;
    config.MagnetOffset = newOffset;
    ConfigurationApplier.apply(config, encoder);

    // TODO Reduce update timeout
    encoder.getAbsolutePosition().waitForUpdate(1);
  }
}
