package org.firstinspires.ftc.teamcode.Config.Utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Utility wrapper for a beam-break sensor which may be wired as either a digital input (simple
 * broken/unbroken) or an analog sensor that provides a voltage proportional to distance.
 *
 * <p>This class abstracts the hardware differences and exposes methods to check whether the beam is
 * broken and to read an approximate distance in millimeters for analog sensors.
 */
public class BeamBreak {
  /** Digital beam-break sensor (used when isAnalog == false). */
  private DigitalChannel beamBreakSensor;

  /** Analog beam-break sensor (used when isAnalog == true). */
  private AnalogInput beamBreakSensorAnalog;

  /** True when the sensor is analog; false for digital. */
  private boolean isAnalog;

  /** Distance threshold in millimeters that defines "beam broken" for analog sensors. */
  private double setDistance_MM = 200.0;

  /** Maximum voltage expected from the analog sensor. */
  private double max_volts = 3.3;

  /** Maximum measurable distance (in millimeters) corresponding to max_volts. */
  private double max_Distance_MM = 1000.0;

  /***
   * Constructor for a BeamBreak sensor.
   *
   * @param hardwareMap The hardware map to retrieve the sensor from.
   * @param sensorName The name of the sensor in the hardware map.
   * @param isAnalog True if the sensor is analog (provides voltage); false if digital.
   * @param setDistance_MM Distance threshold in millimeters for analog sensors below which
   *                       the beam is considered broken.
   */
  public BeamBreak(
      HardwareMap hardwareMap, String sensorName, boolean isAnalog, double setDistance_MM) {
    this.isAnalog = isAnalog;
    this.setDistance_MM = setDistance_MM;
    if (this.isAnalog) {
      this.beamBreakSensorAnalog = hardwareMap.get(AnalogInput.class, sensorName);
    } else {
      this.beamBreakSensor = hardwareMap.get(DigitalChannel.class, sensorName);
      this.beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);
    }
  }

  /****
   * Overloaded constructor for BeamBreak sensor with a default distance threshold.
   *
   * @param hardwareMap The hardware map to retrieve the sensor from.
   * @param sensorName The name of the sensor in the hardware map.
   * @param isAnalog True if the sensor is analog (provides voltage); false if digital.
   * */
  public BeamBreak(HardwareMap hardwareMap, String sensorName, boolean isAnalog) {
    this(hardwareMap, sensorName, isAnalog, 200.0);
  }

  /***
   * Overloaded constructor for a digital BeamBreak sensor using default threshold.
   *
   * @param hardwareMap The hardware map to retrieve the sensor from.
   * @param sensorName The name of the sensor in the hardware map.
   * */
  public BeamBreak(HardwareMap hardwareMap, String sensorName) {
    this(hardwareMap, sensorName, false, 200.0);
  }

  /***
   * Check whether the beam is broken.
   *
   * <p>For analog sensors this converts the measured voltage to a distance and compares it to
   * the configured setDistance_MM threshold. For digital sensors the digital input state is returned.</p>
   *
   * @return true if the beam is considered broken; false otherwise.
   * */
  public boolean isBeamBroken() {
    if (isAnalog) {
      double voltage = beamBreakSensorAnalog.getVoltage();
      double distance = (voltage / max_volts) * max_Distance_MM;
      return distance <= setDistance_MM;
    }
    return beamBreakSensor.getState();
  }

  /**
   * Checks the distance from the analog beam break sensor in millimeters.
   *
   * <p>Only applicable for analog sensors. For digital sensors the method currently returns an
   * invalid value (keeps existing behavior).
   *
   * @return estimated distance in millimeters for analog sensors; behavior undefined for digital
   *     sensors.
   */
  public double getDistanceMM() {
    if (isAnalog) {
      double voltage = beamBreakSensorAnalog.getVoltage();
      return (voltage / max_volts) * max_Distance_MM;
    }
    return Double.parseDouble(null); // Not applicable for digital sensors
  }

  /***
   * Sets the distance threshold for analog beam break sensors.
   * Not applicable for digital sensors.
   *
   * @param setDistance_MM The distance threshold in millimeters to use for analog sensors.
   * */
  public void setSetDistance_MM(double setDistance_MM) {
    this.setDistance_MM = setDistance_MM;
  }
}
