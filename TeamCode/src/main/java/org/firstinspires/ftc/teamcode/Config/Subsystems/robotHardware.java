package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

public class robotHardware extends WSubsystem{
    private Follower m_follower;
    private HardwareMap hardwareMap;
    public robotHardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.m_follower = PedroConstants.createFollower(hardwareMap);
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
    public Follower getFollower() {
        return m_follower;
    }
}
