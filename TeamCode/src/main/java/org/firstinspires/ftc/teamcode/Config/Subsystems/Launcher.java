package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.FlywheelKinematics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Launcher extends WSubsystem {

  private DcMotorEx flywheelLead;
  private DcMotorEx flywheelFollow;

  private PIDFController flywheelController;

  private double flywheelVelocity = 0.0;
  private double flywheelTargetVelocity = 0.0; // Ticks per second


  private List<VoltageSensor> voltageSensors;
  private double batteryVoltage = Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
  private ElapsedTime voltageTimer = new ElapsedTime();

  private double compensatedPower = 0.0;

  public Launcher(HardwareMap hardwareMap) {
    // Initialize motors
    flywheelLead = hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_LEAD);
    flywheelFollow = hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW);

    // Set motor directions
    flywheelLead.setDirection(
        Constants.Launcher.FLYWHEEL_MOTOR_LEAD_INVERTED
            ? DcMotor.Direction.REVERSE
            : DcMotor.Direction.FORWARD
    );
    flywheelFollow.setDirection(
        Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW_INVERTED
            ? DcMotor.Direction.REVERSE
            : DcMotor.Direction.FORWARD
    );

    // Configure both motors
    flywheelLead.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheelFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    flywheelLead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flywheelLead.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    flywheelFollow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Initialize PID controller with constants
    flywheelController = new PIDFController(
        Constants.Launcher.kP,
        Constants.Launcher.kI,
        Constants.Launcher.kD,
        0
    );

    // Setup voltage compensation
    voltageSensors = hardwareMap.getAll(VoltageSensor.class);
    voltageTimer.reset();
  }

  @Override
  public void read() {
    // Read velocity from lead motor (has encoder)
    this.flywheelVelocity = flywheelLead.getVelocity();

    // Periodically update battery voltage
    if (voltageTimer.seconds() >= Constants.Launcher.VOLTAGE_UPDATE_INTERVAL_SECONDS) {
      batteryVoltage = getBatteryVoltage();
      voltageTimer.reset();
    }
  }

  @Override
  public void loop() {
    // If target is 0 and actual velocity is within deadband, stop motors to prevent oscillation
    if (flywheelTargetVelocity == 0 && Math.abs(flywheelVelocity) <= Constants.Launcher.VELOCITY_DEADBAND) {
      compensatedPower = 0.0;
      return;
    }

    // Calculate base power using PID controller + static feedforward
    double basePower = MathUtils.clamp(
        flywheelController.calculate(flywheelVelocity, flywheelTargetVelocity) + Constants.Launcher.kS,
        -1,
        1
    );

    // Apply voltage compensation
    compensatedPower = basePower;
    if (batteryVoltage > 0.1) {
      compensatedPower = basePower * (Constants.Launcher.NOMINAL_BATTERY_VOLTAGE / batteryVoltage);
      compensatedPower = MathUtils.clamp(compensatedPower, -1, 1);
    }
  }

  @Override
  public void write() {
    // Set power to both motors
    flywheelLead.setPower(compensatedPower);
    flywheelFollow.setPower(compensatedPower);
  }

  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty()) {
      return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    }
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) {
      return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    }
    return minV;
  }

  public double getFlywheelTargetVelocity() {
    return flywheelTargetVelocity;
  }

  public void setFlywheelTargetVelocity(double flywheelTargetVelocity) {
    this.flywheelTargetVelocity = flywheelTargetVelocity;
  }

  public double getFlywheelVelocity() {
    return flywheelVelocity;
  }

  public double getBatteryVoltageValue() {
    return batteryVoltage;
  }

  public double getError() {
    return flywheelTargetVelocity - flywheelVelocity;
  }

  public double getCompensatedPower() {
    return compensatedPower;
  }

  public InstantCommand LaunchFar(){
      return new InstantCommand(()->setFlywheelTargetVelocity(Constants.Launcher.FarVelocity));
  }
  public InstantCommand LaunchClose(){
    return new InstantCommand(()->setFlywheelTargetVelocity(Constants.Launcher.closeVelocity));
  }
  public InstantCommand HPIntake(){
    return new InstantCommand(()->setFlywheelTargetVelocity(Constants.Launcher.intakeVelocity));
  }
  public InstantCommand stop(){
      return new InstantCommand(()->setFlywheelTargetVelocity(0));
  }
  public InstantCommand LaunchRannge(double range){
    return new InstantCommand(()->setFlywheelTargetVelocity(FlywheelKinematics.calculateFlywheelSpeed(range)));
  }
}
