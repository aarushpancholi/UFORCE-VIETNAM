package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class transfer extends CommandBase {
    boolean on = false;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public transfer(Intake subsystem, boolean on) {
        intakeSubsystem = subsystem;
        this.on = on;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (on) {
            intakeSubsystem.setStopper(0.4);
            intakeSubsystem.intake2On();
        }
        else {
            intakeSubsystem.setStopper(0.6);
            intakeSubsystem.intake2Off();
        }
    }

    @Override
    public boolean isFinished() {
        if (on) {
            return (intakeSubsystem.getStopper() == 0.4 && intakeSubsystem.isIntake2On());
        }
        else {
            return (intakeSubsystem.getStopper() == 0.6 && !intakeSubsystem.isIntake2On());
        }

    }
}