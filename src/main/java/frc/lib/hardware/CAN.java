package frc.lib.hardware;

public class CAN {

  public final int id;
  public final String bus;

  public CAN(int id) {
    this.id = id;
    this.bus = "";
  }

  public CAN(int id, String bus) {
    this.id = id;
    this.bus = bus;
  }
}
