package frc.lib.util;

public class ControlAreaNetworkDevice {

  public final int id;
  public final String bus;

  public ControlAreaNetworkDevice(int id) {
    this.id = id;
    this.bus = "";
  }

  public ControlAreaNetworkDevice(int id, String bus) {
    this.id = id;
    this.bus = bus;
  }
}
