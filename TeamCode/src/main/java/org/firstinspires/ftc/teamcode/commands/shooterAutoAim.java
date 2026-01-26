package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class shooterAutoAim extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret turretSubsytem;


    public shooterAutoAim(Turret subsystem) {
        turretSubsytem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        turretSubsytem.setAutoAim(true);
    }

    @Override
    public boolean isFinished() {
        return turretSubsytem.autoAimEnabled;
    }
}