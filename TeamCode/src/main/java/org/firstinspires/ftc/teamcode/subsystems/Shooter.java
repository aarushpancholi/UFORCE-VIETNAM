package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Shooter extends SubsystemBase {
    private final MotorEx shooterA;
    private final MotorEx shooterB;

    private double kP = 0.05;     // tune
    private double targetRpm = 0; // tune

    private TelemetryManager telemetry;
    private MotorGroup shooterMotors;

    InterpLUT speedLut = new InterpLUT();
    InterpLUT angleLut = new InterpLUT();


    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager) {

        shooterA = new MotorEx(hardwareMap, "sh", Motor.GoBILDA.BARE);
        shooterA.setRunMode(Motor.RunMode.VelocityControl);
        shooterA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterA.setPositionCoefficient(kP);
        shooterA.setInverted(false);
        shooterA.stopAndResetEncoder();
        shooterA.setVeloCoefficients(0.05, 0.01, 0.31);

        shooterB = new MotorEx(hardwareMap, "sh2", Motor.GoBILDA.BARE);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterB.setPositionCoefficient(kP);
        shooterB.stopAndResetEncoder();
        shooterB.setVeloCoefficients(0.05, 0.01, 0.31);

        shooterMotors = new MotorGroup(shooterA, shooterB);

        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void periodic() {
        double radPerSec = targetRpm * 2.0 * Math.PI / 60.0;

        shooterA.set(radPerSec);
        shooterB.set(radPerSec);

        telemetry.addData("target rpm", targetRpm);
        telemetry.addData("velA", shooterA.getVelocity());
        telemetry.addData("velB", shooterB.getVelocity());
    }

    public void setSpeedRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
    }
    public List<Double> getSpeed() {
        return shooterMotors.getVelocities();
    }


}
