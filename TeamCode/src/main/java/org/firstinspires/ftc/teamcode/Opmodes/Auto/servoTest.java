package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class servoTest extends LinearOpMode {
private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo3");
        servo.setPosition(0.5);
        sleep(1000);
        servo.setPosition(0.63);
        waitForStart();
        while (opModeIsActive()){}
    }
}
