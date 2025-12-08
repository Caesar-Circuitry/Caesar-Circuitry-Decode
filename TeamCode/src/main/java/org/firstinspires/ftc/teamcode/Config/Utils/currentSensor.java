package org.firstinspires.ftc.teamcode.Config.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility wrapper around a {@link DcMotorEx} to read and monitor motor current with a simple
 * overcurrent detection that requires the current to remain above the threshold for a configurable
 * duration before reporting an overcurrent condition.
 *
 * <p>This class caches the last read current value (in the configured {@link CurrentUnit}) and
 * exposes convenience constructors to set an overcurrent threshold and debounce time (in
 * milliseconds) used by {@link #isOverCurrent()}.
 *
 * <p>Usage example:
 *
 * <pre>
 * DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorName");
 * currentSensor cs = new currentSensor(motor, 10, CurrentUnit.AMPS, 300);
 * double amps = cs.readCurrent();
 * if (cs.isOverCurrent()) { ... }
 * </pre>
 */
public class currentSensor {
  /** The wrapped {@link DcMotorEx} instance used to read current. */
  private DcMotorEx motor;

  /**
   * Overcurrent threshold. If the measured current is greater than or equal to this value, the
   * overcurrent timer logic is engaged; see {@link #isOverCurrent()}.
   *
   * <p>The value is in the units specified by {@link #currentUnit}.
   */
  private double overcurrent = 7;

  /** Last-read current value (in the configured {@link CurrentUnit}). */
  private double current = 0;

  /** Unit used when reading current from the motor. */
  private CurrentUnit currentUnit = CurrentUnit.AMPS;

  /**
   * Timer used to measure how long the current has remained at or above the overcurrent threshold.
   * The timer is reset whenever the current drops below the threshold.
   */
  private ElapsedTime currentTimer;

  /**
   * Debounce duration (in milliseconds) that the current must remain above {@link #overcurrent}
   * before {@link #isOverCurrent()} returns true.
   */
  private double currentMilis = 250;

  /**
   * Create a currentSensor for a motor with a specified overcurrent threshold, unit, and debounce
   * time.
   *
   * @param motor the {@link DcMotorEx} to monitor
   * @param overcurrent the threshold (inclusive) above which overcurrent timing begins (in {@code
   *     currentUnit})
   * @param currentUnit the unit to use when reading current (e.g. {@link CurrentUnit#AMPS})
   * @param currentMilis debounce time in milliseconds the current must stay >= {@code overcurrent}
   *     before {@link #isOverCurrent()} returns {@code true}
   */
  public currentSensor(
      DcMotorEx motor, double overcurrent, CurrentUnit currentUnit, double currentMilis) {
    this.motor = motor;
    this.overcurrent = overcurrent;
    this.currentUnit = currentUnit;
    this.currentTimer = new ElapsedTime();
    this.currentTimer.reset();
    this.currentMilis = currentMilis;
  }

  /**
   * Create a currentSensor with a specified overcurrent threshold and unit, using a default
   * debounce time of 250 ms.
   *
   * @param motor the {@link DcMotorEx} to monitor
   * @param overcurrent the overcurrent threshold (in {@code currentUnit})
   * @param currentUnit the unit to use when reading current
   */
  public currentSensor(DcMotorEx motor, double overcurrent, CurrentUnit currentUnit) {
    this(motor, overcurrent, currentUnit, 250);
  }

  /**
   * Create a currentSensor for a motor with a specified overcurrent threshold in AMPS and default
   * debounce time.
   *
   * @param motor the {@link DcMotorEx} to monitor
   * @param overcurrent the overcurrent threshold in AMPS
   */
  public currentSensor(DcMotorEx motor, double overcurrent) {
    this(motor, overcurrent, CurrentUnit.AMPS, 250);
  }

  /**
   * Create a currentSensor for a motor with the default overcurrent threshold (7 AMPS) and default
   * debounce time.
   *
   * @param motor the {@link DcMotorEx} to monitor
   */
  public currentSensor(DcMotorEx motor) {
    this(motor, 7, CurrentUnit.AMPS, 250);
  }

  /**
   * Read the current from the wrapped motor and update the internal cached value.
   *
   * @return the current value read from the motor (in the configured {@link CurrentUnit})
   */
  public double readCurrent() {
    this.current = this.motor.getCurrent(currentUnit);
    return this.current;
  }

  /**
   * Determine whether the motor is in an overcurrent condition.
   *
   * <p>Behavior: - If the most recently cached {@link #current} is below {@link #overcurrent}, the
   * internal timer is reset and this method returns {@code false}. - If {@link #current} is greater
   * than or equal to {@link #overcurrent}, the method returns {@code true} only after the value has
   * remained >= threshold for at least {@link #currentMilis} milliseconds.
   *
   * <p>Note: call {@link #readCurrent()} prior to this method to refresh the cached current from
   * hardware.
   *
   * @return {@code true} if the current has been >= {@link #overcurrent} for at least {@link
   *     #currentMilis} ms; {@code false} otherwise
   */
  public boolean isOverCurrent() {
    if (current < overcurrent) {
      currentTimer.reset();
      return false;
    }
    if (currentTimer.milliseconds() >= this.currentMilis) {
      return true;
    }
    return false;
  }

  /**
   * Get the overcurrent threshold.
   *
   * @return the overcurrent threshold value (in the configured {@link CurrentUnit})
   */
  public double getOvercurrent() {
    return this.overcurrent;
  }

  /**
   * Set the overcurrent threshold.
   *
   * @param overcurrent new overcurrent threshold (in the configured {@link CurrentUnit})
   */
  public void setOvercurrent(double overcurrent) {
    this.overcurrent = overcurrent;
  }

  /**
   * Get the most recently read current value.
   *
   * @return last-read current (in the configured {@link CurrentUnit})
   */
  public double getCurrent() {
    return this.current;
  }

  /**
   * Manually set the cached current value.
   *
   * <p>Note: this does not read from hardware; use {@link #readCurrent()} to refresh from the
   * motor.
   *
   * @param current value to set as the cached current
   */
  public void setCurrent(double current) {
    this.current = current;
  }

  /**
   * Get the {@link CurrentUnit} used for motor current reads.
   *
   * @return the configured {@link CurrentUnit}
   */
  public CurrentUnit getCurrentUnit() {
    return this.currentUnit;
  }

  /**
   * Set the {@link CurrentUnit} used for subsequent motor current reads.
   *
   * @param currentUnit the {@link CurrentUnit} to use (e.g. {@link CurrentUnit#AMPS})
   */
  public void setCurrentUnit(CurrentUnit currentUnit) {
    this.currentUnit = currentUnit;
  }

  /**
   * Get the debounce duration (in milliseconds) used by {@link #isOverCurrent()}.
   *
   * @return debounce time in milliseconds
   */
  public double getCurrentMilis() {
    return currentMilis;
  }

  /**
   * Set the debounce duration (in milliseconds) used by {@link #isOverCurrent()}.
   *
   * @param currentMilis debounce time in milliseconds; must be non-negative
   */
  public void setCurrentMilis(double currentMilis) {
    this.currentMilis = currentMilis;
  }
}
