package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Launcher extends WSubsystem {
    private final DcMotor Launchy;
    

    public Launcher(final HardwareMap hMap, final String name) {
        Launchy = hMap.get(DcMotor.class, name);
    }

    @Override
    public void read() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void write() {

    }
}
