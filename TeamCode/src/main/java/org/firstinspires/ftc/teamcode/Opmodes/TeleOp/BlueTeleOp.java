package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BlueTeleOp extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Constants.Robot.alliance = Constants.Robot.Alliance.BLUE;
        Constants.Robot.Goal = Constants.Robot.BlueGoal;
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap,gamepad1,gamepad2,Telemetry);
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write));
    }

    @Override
    public void run(){
        super.run();
    }
}
