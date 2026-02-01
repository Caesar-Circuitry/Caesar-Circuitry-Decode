package org.firstinspires.ftc.teamcode.Config.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

@Configurable
@TeleOp(name = "Motor Tester", group = "Teleop Test")
public class MotorTester extends OpMode {
    public static double leftFrontMotorDirection = 0;
    public static double leftRearMotorDirection = 0;
    public static double rightFrontMotorDirection = 0;
    public static double rightRearMotorDirection = 0;
    public static double test = 1;

    private JoinedTelemetry telemetryM;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftRear.setDirection(direction(leftRearMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightRear.setDirection(direction(rightRearMotorDirection));

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryM = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telemetryM.addLine("This will allow you to test the directions of your motors. You can change the directions in FTCDashboard -> FollowerConstants.");
        telemetryM.update();
    }

    @Override
    public void loop() {
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftRear.setDirection(direction(leftRearMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightRear.setDirection(direction(rightRearMotorDirection));

        if(gamepad1.a)
            leftFront.setPower(1);
        else
            leftFront.setPower(0);

        if(gamepad1.y)
            leftRear.setPower(1);
        else
            leftRear.setPower(0);

        if(gamepad1.b)
            rightFront.setPower(1);
        else
            rightFront.setPower(0);

        if(gamepad1.x)
            rightRear.setPower(1);
        else
            rightRear.setPower(0);

        telemetryM.addLine("Press A to spin the left front motor at 100% power");
        telemetryM.addLine("Press Y to spin the left rear motor at 100% power");
        telemetryM.addLine("Press B to spin the right front motor at 100% power");
        telemetryM.addLine("Press X to spin the right rear motor at 100% power");
        telemetryM.addLine("Left Front Motor Direction: " + leftFrontMotorDirection);
        telemetryM.addLine("Left Rear Motor Direction: "+ leftRearMotorDirection);
        telemetryM.addLine("Right Front Motor Direction: "+ rightFrontMotorDirection);
        telemetryM.addLine("Right Rear Motor Direction: "+ rightRearMotorDirection);
        telemetryM.update();
    }

    public DcMotorSimple.Direction direction(double d) {
        if (d == 0)
            return DcMotorSimple.Direction.FORWARD;
        else
            return DcMotorSimple.Direction.REVERSE;
    }
}