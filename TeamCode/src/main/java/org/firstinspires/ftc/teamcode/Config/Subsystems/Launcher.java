package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher extends WSubsystem{

    private DcMotorEx FlywheelMotorLead;
    private DcMotorEx FlywheelMotorFollow;

    private CRServo turretServoLead;
    private CRServo turretServoFollow;

    private AnalogInput turretServoLeadEncoder;
    private AnalogInput turretServoFollowEncoder;

    private double FlywheelMotorLeadPower = 0.0;
    private double FlywheelMotorFollowPower = 0.0;
    private double turretServoLeadPower = 0.0;
    private double turretServoFollowPower = 0.0;

    public enum Direction {
        FORWARD,
        REVERSE
    }


    public Launcher(HardwareMap hardwareMap) {
        FlywheelMotorLead = hardwareMap.get(DcMotorEx.class, "FlywheelMotorLead");
        FlywheelMotorFollow = hardwareMap.get(DcMotorEx.class, "FlywheelMotorFollow");

        turretServoLead = hardwareMap.get(CRServo.class, "turretServoLead");
        turretServoFollow = hardwareMap.get(CRServo.class, "turretServoFollow");

        turretServoLeadEncoder = hardwareMap.get(AnalogInput.class, "turretServoLeadEncoder");
        turretServoFollowEncoder = hardwareMap.get(AnalogInput.class, "turretServoFollowEncoder");

        FlywheelMotorFollow.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServoFollow.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void read() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void write() {
        FlywheelMotorLead.setPower(FlywheelMotorLeadPower);
        FlywheelMotorFollow.setPower(FlywheelMotorFollowPower);
        turretServoLead.setPower(turretServoLeadPower);
        turretServoFollow.setPower(turretServoFollowPower);
    }

    public double getCurrentAngle(AnalogInput servoEncoder) {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(DcMotorSimple.Direction.REVERSE) ? -360 : 360);
    }
}
