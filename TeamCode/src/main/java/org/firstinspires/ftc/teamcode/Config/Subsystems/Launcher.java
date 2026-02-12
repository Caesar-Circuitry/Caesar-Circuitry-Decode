package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.List;
import java.util.LinkedList;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Launcher extends WSubsystem {

  private DcMotorEx flywheelLead;
  private DcMotorEx flywheelFollow;

  private PIDFController flywheelController;
  private SimpleMotorFeedforward flywheelFeedforward;

  private InterpLUT flywheelSpeeds = new InterpLUT();


  private double flywheelVelocity = 0.0;
  private double flywheelTargetVelocity = 0.0; // Ticks per second
  private boolean stopPower;
  private double currentKv = Constants.Launcher.Kv;
  private boolean inSpinupMode = false;

  // Spin-up timer
  private ElapsedTime spinupTimer = new ElapsedTime();
  private double lastSpinupTime = 0.0;
  private boolean isSpinningUp = false;


  private List<VoltageSensor> voltageSensors;
  private double batteryVoltage = Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
  private ElapsedTime voltageTimer = new ElapsedTime();

  private double compensatedPower = 0.0;

  private final LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();

  public Launcher(HardwareMap hardwareMap) {
    // Initialize motors
    flywheelLead = hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_LEAD);
    flywheelFollow = hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW);

    flywheelSpeeds.add(18,1000);
    flywheelSpeeds.add(49,1100);
    flywheelSpeeds.add(57,1150);
    flywheelSpeeds.add(63,1200);
    flywheelSpeeds.add(73,1250);
    flywheelSpeeds.add(76,1275);
    flywheelSpeeds.add(114,1500);
    flywheelSpeeds.createLUT();

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

    flywheelFeedforward = new SimpleMotorFeedforward(Constants.Launcher.kS,Constants.Launcher.Kv,0);

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
      flywheelController.setPIDF(Constants.Launcher.kP,Constants.Launcher.kI,Constants.Launcher.kD,0);

    // If target is 0 and actual velocity is within deadband, stop motors to prevent oscillation
    if (flywheelTargetVelocity == 0 && Math.abs(flywheelVelocity) <= Constants.Launcher.VELOCITY_DEADBAND) {
      compensatedPower = 0.0;
      updateTelemetry();
      return;
    }
    if (stopPower){
        compensatedPower =0.0;
        updateTelemetry();
        return;
    }

    // Track spin-up time
    double velocityError = Math.abs(flywheelTargetVelocity - flywheelVelocity);
    if (isSpinningUp && velocityError <= Constants.Launcher.Detection_DeadBand) {
      lastSpinupTime = spinupTimer.seconds();
      isSpinningUp = false;
    }

    flywheelFeedforward = new SimpleMotorFeedforward(Constants.Launcher.kS, Constants.Launcher.Kv, 0);

    // Calculate base power using PID + feedforward
    double pidOutput = flywheelController.calculate(flywheelVelocity, flywheelTargetVelocity);
    double ffOutput = flywheelFeedforward.calculate(flywheelTargetVelocity);
    double basePower = MathUtils.clamp(pidOutput + ffOutput, -1, 1);

    // Apply voltage compensation
    compensatedPower = basePower;
    if (batteryVoltage > 0.1) {
      compensatedPower = basePower * (Constants.Launcher.NOMINAL_BATTERY_VOLTAGE / batteryVoltage);
      compensatedPower = MathUtils.clamp(compensatedPower, -1, 1);
    }

    updateTelemetry();
  }

  private void updateTelemetry() {
    if (Constants.Launcher.logTelemetry) {
      telemetryPackets.clear();
      telemetryPackets.add(new TelemetryPacket("Target Velocity", flywheelTargetVelocity));
      telemetryPackets.add(new TelemetryPacket("Actual Velocity", flywheelVelocity));
      telemetryPackets.add(new TelemetryPacket("Velocity Error", getError()));
      telemetryPackets.add(new TelemetryPacket("Compensated Power", compensatedPower));
      telemetryPackets.add(new TelemetryPacket("Battery Voltage", batteryVoltage));
      telemetryPackets.add(new TelemetryPacket("StopPower", stopPower));
      telemetryPackets.add(new TelemetryPacket("In Spinup Mode", inSpinupMode));
      telemetryPackets.add(new TelemetryPacket("Spin-up Time (s)", lastSpinupTime));
      telemetryPackets.add(new TelemetryPacket("Is Spinning Up", isSpinningUp));
      telemetryPackets.add(new TelemetryPacket("kP", Constants.Launcher.kP));
      telemetryPackets.add(new TelemetryPacket("kI", Constants.Launcher.kI));
      telemetryPackets.add(new TelemetryPacket("kD", Constants.Launcher.kD));
      telemetryPackets.add(new TelemetryPacket("kS", Constants.Launcher.kS));
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
    setStopPower(false);
    // Start spin-up timer when setting a new non-zero target
    if (flywheelTargetVelocity != 0 && this.flywheelTargetVelocity != flywheelTargetVelocity) {
      spinupTimer.reset();
      isSpinningUp = true;
    }
    this.flywheelTargetVelocity = flywheelTargetVelocity;
  }
  public void LaunchOnRange(double range){
      setFlywheelTargetVelocity(flywheelSpeeds.get(range));
  }
  public void LaunchOnPose(Pose robotPose, Pose targetPose){
      Pose LaunchPose = new Pose(robotPose.getX(),robotPose.getY() -Constants.Launcher.LUTDistance,robotPose.getHeading());
      double distance = LaunchPose.distanceFrom(targetPose);
      setFlywheelTargetVelocity(flywheelSpeeds.get(distance));
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
  public InstantCommand LaunchRange(double range){
    return new InstantCommand(()->LaunchOnRange(range));
  }
  public InstantCommand LaunchPose(Pose robotPose, Pose targetPose){
      return new InstantCommand(()->LaunchOnPose(robotPose,targetPose));
  }
  public InstantCommand stopPower(){
      return new InstantCommand(()->setStopPower(true));
  }
  public void setStopPower(boolean stopPower){
      this.stopPower = stopPower;
  }
  public boolean isAtDesiredSpeed(){
      if (flywheelTargetVelocity == flywheelVelocity-Constants.Launcher.Detection_DeadBand ||flywheelTargetVelocity == flywheelVelocity+Constants.Launcher.Detection_DeadBand){
          return true;
      }
      return false;
  }

  @Override
  public LinkedList<TelemetryPacket> getTelemetry() {
    return telemetryPackets;
  }
}
