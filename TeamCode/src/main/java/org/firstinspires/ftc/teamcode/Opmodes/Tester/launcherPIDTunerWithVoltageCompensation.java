package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class launcherPIDTunerWithVoltageCompensation extends LinearOpMode {
  PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

  // We do actually want to use KI; this Ki will be the main driving component to hold velocity
  public static double Kp = 0.01, Ki = 0.4, Kd = 0.0, Ks = 0.0431, targetVelocity = 1125;

  DcMotorEx launcherMotor;
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

    // hardware init
    launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
    launcherController = new PIDFController(Kp, Ki, Kd, 0);
    launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Use RUN_USING_ENCODER so getVelocity() reports correctly
    launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

      // periodically sample battery voltage (every 5s to match StarterBot behavior)
      if (voltageTimer.seconds() >= 1.0) {
        batteryVoltage = getBatteryVoltage();
        voltageTimer.reset();
      }

      actualVelocity = launcherMotor.getVelocity();

      double basePower =
          MathUtils.clamp(launcherController.calculate(actualVelocity, targetVelocity) + Ks, -1, 1);

      double compensatedPower = basePower;
      if (batteryVoltage > 0.1) {
        compensatedPower = basePower * (NOMINAL_BATTERY_VOLTAGE / batteryVoltage);
        // ensure final power stays in [-1, 1]
        compensatedPower = MathUtils.clamp(compensatedPower, -1, 1);
      }

      launcherMotor.setPower(compensatedPower);

      panelsTelemetry.getTelemetry().addData("Target Velocity", targetVelocity);
      panelsTelemetry.getTelemetry().addData("Actual Velocity", actualVelocity);
      panelsTelemetry.getTelemetry().addData("Base Power", basePower);
      panelsTelemetry.getTelemetry().addData("Compensated Power", compensatedPower);
      panelsTelemetry.getTelemetry().addData("Error", targetVelocity - actualVelocity);
      panelsTelemetry
          .getTelemetry()
          .addData("currentDraw", launcherMotor.getCurrent(CurrentUnit.AMPS));
      panelsTelemetry.getTelemetry().addData("batteryV", batteryVoltage);

      panelsTelemetry.getTelemetry().update();

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
