package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class turretStraight extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret turretSubsystem;


    public turretStraight(Turret subsystem) {
        turretSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        turretSubsystem.straight();
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.isStraight();
    }
}