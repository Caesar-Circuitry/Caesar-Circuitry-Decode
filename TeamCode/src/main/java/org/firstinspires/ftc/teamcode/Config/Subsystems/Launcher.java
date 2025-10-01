package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Config.Constants;

public class Launcher extends WSubsystem {
    private final DcMotor Launchy;

    private double power = 0;
    private SimpleMotorFeedforward feedforward;
    private double feedforwardpower = 0;

    public void calculate(){
        feedforwardpower = feedforward.calculate(10, 20);

    }
    public Launcher(final HardwareMap hMap, final String name) {
        Launchy = hMap.get(DcMotor.class, name);
        feedforward = new SimpleMotorFeedforward(Constants.Launcher.Ks, Constants.Launcher.Kv, Constants.Launcher.Ka);

    }
    public void calculate(double v, double a){
        feedforwardpower = feedforward.calculate(v,a);
    }
    @Override
    public void read() {

    }

    @Override
    public void loop() {
    power=feedforwardpower;
    }

    @Override
    public void write() {
        Launchy.setPower(feedforwardpower);
    }
}
