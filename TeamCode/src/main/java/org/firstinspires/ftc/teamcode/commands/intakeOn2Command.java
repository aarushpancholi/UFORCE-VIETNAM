package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class intakeOn2Command extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public intakeOn2Command(Intake subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.intake2On();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isIntake2On();
    }
}