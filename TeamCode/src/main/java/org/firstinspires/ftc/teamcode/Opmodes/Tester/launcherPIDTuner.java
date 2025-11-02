package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

@TeleOp
@Configurable
public class launcherPIDTuner extends LinearOpMode {
  PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

  // We do actually want to use KI in fact this Ki will be main driving component to hold velocity
  public static double Kp = 0.01, Ki = 0.4, Kd = 0.0, Ks = 0.0431, targetVelocity = 1125;

  DcMotorEx launcherMotor;
  double actualVelocity = 0.0;
  PIDFController launcherController;

  @Override
  public void runOpMode() throws InterruptedException {

    launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
    launcherController = new PIDFController(Kp, Ki, Kd, 0);
    launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();
    while (opModeIsActive()) {

      launcherController.setCoefficients(new PIDFCoefficients(Kp, Ki, Kd, 0));
      actualVelocity = launcherMotor.getVelocity();

      double Power =
          MathUtils.clamp(launcherController.calculate(actualVelocity, targetVelocity) + Ks, -1, 1);

      launcherMotor.setPower(Power);

      panelsTelemetry.getTelemetry().addData("Target Velocity", targetVelocity);
      panelsTelemetry.getTelemetry().addData("Actual Velocity", actualVelocity);
      panelsTelemetry.getTelemetry().addData("Motor Power", Power);
      panelsTelemetry.getTelemetry().addData("Error", targetVelocity - actualVelocity);
      panelsTelemetry
          .getTelemetry()
          .addData("currentDraw", launcherMotor.getCurrent(CurrentUnit.AMPS));

      panelsTelemetry.getTelemetry().update();
    }
  }
}
