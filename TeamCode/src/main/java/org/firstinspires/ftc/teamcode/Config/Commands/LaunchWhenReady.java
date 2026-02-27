package org.firstinspires.ftc.teamcode.Config.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Launcher;

public class LaunchWhenReady extends CommandBase {
    private final Intake intake;
    private final Launcher launcher;
    public LaunchWhenReady(Intake intake, Launcher launcher){
        this.intake = intake;
        this.launcher = launcher;
        addRequirements(intake,launcher);
    }
    @Override
    public void execute(){
        if (launcher.isAtDesiredSpeed()){
            intake.Launch();
        }
    }
    @Override
    public boolean isFinished(){
        return launcher.isAtDesiredSpeed();
    }

}
