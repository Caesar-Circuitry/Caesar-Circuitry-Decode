package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

@TeleOp
@Configurable
public class launcherPIDTuner extends LinearOpMode {

    public static double
            Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 0.0,
            targetVelocity = 1125;
    private DcMotorEx launcherMotor;
    private double actualVelocity = 0.0;
    private PIDFController launcherController;
    @Override
    public void runOpMode() throws InterruptedException {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherController = new PIDFController(Kp, Ki, Kd, Kf);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()){

        }
    }
}
