package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class setShooter extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter shooterSubsystem;
    private int target = 0;
    private double pos = 0.0;


    public setShooter(Shooter subsystem, int target, double pos) {
        shooterSubsystem = subsystem;
        this.target = target;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        shooterSubsystem.setTargetEPT(target);
        shooterSubsystem.setHood(pos);
    }

    @Override
    public boolean isFinished() {
        return (target - 10 <= shooterSubsystem.getVelA()) && (shooterSubsystem.getVelA() <= target+10);
    }
}