package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class turretAutoAim extends CommandBase {
    boolean setAutoAim = false;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret turretSubsystem;


    public turretAutoAim(Turret subsystem, boolean setAutoAim) {
        turretSubsystem = subsystem;
        this.setAutoAim = setAutoAim;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.setAutoAim(setAutoAim);
    }

    @Override
    public boolean isFinished() {
        if (setAutoAim) {
            return turretSubsystem.autoAimEnabled;
        } else {
            return !turretSubsystem.autoAimEnabled;
        }
    }
}