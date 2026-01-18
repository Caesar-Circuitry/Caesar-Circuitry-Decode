package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

@TeleOp
@Configurable
public class launcherDualMotorPIDTuner extends LinearOpMode {
  private JoinedTelemetry Telemetry;

  // We do actually want to use KI; this Ki will be the main driving component to hold velocity
  public static double Kp = 0.01, Ki = 0.5, Kd = 0.0, Ks = 0.13, targetVelocity = 0;
  public static double velocityDeadband = 50.0; // Deadband to prevent oscillation when stopping

  DcMotorEx launcherMotorLead;   // Right motor with encoder
  DcMotorEx launcherMotorFollow; // Left motor following lead
  double actualVelocity = 0.0;
  PIDFController launcherController;

  // Voltage compensation fields (match StarterBotTeleopMecanums pattern)
  List<LynxModule> allHubs;
  private List<VoltageSensor> voltageSensors;
  private static final double NOMINAL_BATTERY_VOLTAGE = 12.0;
  private double batteryVoltage = NOMINAL_BATTERY_VOLTAGE;
  ElapsedTime voltageTimer = new ElapsedTime();

  @Override
  public void runOpMode() {

    // hardware init - lead motor (right motor with encoder)
    launcherMotorLead = hardwareMap.get(DcMotorEx.class, "Flywheel1");
    launcherMotorFollow = hardwareMap.get(DcMotorEx.class, "Flywheel2");

    Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    launcherController = new PIDFController(Kp, Ki, Kd, 0);

    // Set motor directions
    launcherMotorLead.setDirection(DcMotor.Direction.FORWARD);
    launcherMotorFollow.setDirection(DcMotor.Direction.REVERSE);

    // Configure both motors
    launcherMotorLead.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    launcherMotorFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    launcherMotorLead.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Use RUN_USING_ENCODER so getVelocity() reports correctly
    launcherMotorLead.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Follow motor runs without encoder
    launcherMotorFollow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // get hubs and voltage sensors
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    voltageSensors = hardwareMap.getAll(VoltageSensor.class);

    waitForStart();
    voltageTimer.reset();

    while (opModeIsActive()) {

      // update controller coefficients for live tuning
      launcherController.setCoefficients(new PIDFCoefficients(Kp, Ki, Kd, 0));

      // periodically sample battery voltage (every 1s)
      if (voltageTimer.seconds() >= 1.0) {
        batteryVoltage = getBatteryVoltage();
        voltageTimer.reset();
      }

      // Read velocity from lead motor (has encoder)
      actualVelocity = launcherMotorLead.getVelocity();

      // If target is 0 and actual velocity is within deadband, stop motors to prevent oscillation
      double basePower;
      double compensatedPower;

      if (targetVelocity == 0 && Math.abs(actualVelocity) <= velocityDeadband) {
        basePower = 0.0;
        compensatedPower = 0.0;
      } else {
        basePower = MathUtils.clamp(launcherController.calculate(actualVelocity, targetVelocity) + Ks, -1, 1);

        compensatedPower = basePower;
        if (batteryVoltage > 0.1) {
          compensatedPower = basePower * (NOMINAL_BATTERY_VOLTAGE / batteryVoltage);
          // ensure final power stays in [-1, 1]
          compensatedPower = MathUtils.clamp(compensatedPower, -1, 1);
        }
      }

      // Set power to both motors
      launcherMotorLead.setPower(compensatedPower);
      launcherMotorFollow.setPower(compensatedPower);

      Telemetry.addData("Target Velocity", "%.2f", targetVelocity);
      Telemetry.addData("Actual Velocity", "%.2f", actualVelocity);
      Telemetry.addData("Velocity Deadband", "%.2f", velocityDeadband);
      Telemetry.addData("Within Deadband", (targetVelocity == 0 && Math.abs(actualVelocity) <= velocityDeadband) ? "YES" : "NO");
      Telemetry.addData("Base Power", "%.3f", basePower);
      Telemetry.addData("Compensated Power", "%.3f", compensatedPower);
      Telemetry.addData("Error", "%.2f", targetVelocity - actualVelocity);
      Telemetry.addData("Lead Motor Current (A)", "%.2f", launcherMotorLead.getCurrent(CurrentUnit.AMPS));
      Telemetry.addData("Follow Motor Current (A)", "%.2f", launcherMotorFollow.getCurrent(CurrentUnit.AMPS));
      Telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);

      Telemetry.update();

      // clear hub caches
      for (LynxModule hub : allHubs) {
        hub.clearBulkCache();
      }
    }
  }

  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty()) return 0.0;
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) {
      return 0.0;
    }
    return minV;
  }
}

