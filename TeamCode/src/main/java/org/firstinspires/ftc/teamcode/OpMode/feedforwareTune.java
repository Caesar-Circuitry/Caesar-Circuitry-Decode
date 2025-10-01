package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Launcher;

public class feedforwareTune extends LinearOpMode {

    private Launcher launch;
    @Override
    public void runOpMode() throws InterruptedException {
        launch = new Launcher(hardwareMap, "LMoter");
        waitForStart();
        while(opModeIsActive()){
            launch.calculate(Constants.Launcher.A, Constants.Launcher.V);
            launch.write();
        }

    }

}
