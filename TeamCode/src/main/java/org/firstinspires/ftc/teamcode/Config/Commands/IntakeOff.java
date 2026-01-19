package org.firstinspires.ftc.teamcode.Config.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;

public class IntakeOff extends CommandBase {
    private final Intake intake;
    public IntakeOff(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize(){
        intake.Hold();
    }

}
