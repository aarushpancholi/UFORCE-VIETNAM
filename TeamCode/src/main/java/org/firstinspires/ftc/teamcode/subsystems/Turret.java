package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getBlueHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minTurretPos;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;

public class Turret extends SubsystemBase {
    private final MotorEx turret;

    private boolean autoAimEnabled = false;

    private double kP = 0.05;     // tune
    private double maxPower = 0.8; // tune
    

//    private double kP = 0;
//    PIDFController turretPIDFController = new PIDFController(kP, kI, kD, kF);

    InterpLUT lut = new InterpLUT();
    InterpLUT inverseLut = new InterpLUT();

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        turret = new MotorEx(hardwareMap, "turret");
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setPositionCoefficient(kP);
        turret.setPositionTolerance(10);
        turret.setInverted(true);
        turret.stopAndResetEncoder();

//        turretPos = turret.getCurrentPosition();
//        robotHeading = Pinpoint.getHeading();
//        fieldCentricHeading = robotHeading + encoderToTurretHeading();

        lut.add(minTurretPos, r(-135));
        lut.add(maxTurretPos, r(135));
        inverseLut.add(r(-135), minTurretPos);
        inverseLut.add(r(135), maxTurretPos);
        lut.createLUT();
        inverseLut.createLUT();
    }

    public void straight() {

    }

    public void resetTurretEncoder() {
        turret.stopAndResetEncoder();
    }

    public double getPos() {
        return turret.getCurrentPosition();
    }
    public void setAutoAim(boolean enabled) {
        autoAimEnabled = enabled;
        if (!enabled) turret.set(0);
    }

    @Override
    public void periodic() {
        if (!autoAimEnabled) return;

        double robotHeading = Localization.getHeading();              // radians :contentReference[oaicite:5]{index=5}
        double turretRelHeading = encoderToTurretHeading();       // radians (after LUT fix)
        double turretAbsHeading = AngleUnit.normalizeRadians(robotHeading + turretRelHeading);

        double headingDiffToGoal = getBlueHeadingDiff(turretAbsHeading); // should be radians :contentReference[oaicite:6]{index=6}
        double targetAbsHeading = AngleUnit.normalizeRadians(turretAbsHeading + headingDiffToGoal);

        double targetRelHeading = AngleUnit.normalizeRadians(targetAbsHeading - robotHeading);
//        if (targetRelHeading > r(135) || targetRelHeading < r(-135)) {
//            targetRelHeading = ((targetRelHeading + r(135))%r(135))-r(135);
//        }
        double targetTurretPos = headingToEncoder(targetRelHeading);

        targetTurretPos = Range.clip(targetTurretPos, minTurretPos, maxTurretPos);

        turret.setTargetPosition((int) targetTurretPos);
        turret.set(maxPower); // always apply “speed cap” so it can hold/correct

    }
    
    public double r(double val) {
        return Math.toRadians(val);
    }

    public boolean isAimed() {
        return turret.atTargetPosition();
    }

    public double encoderToTurretHeading() {
        return lut.get(getPos());
    }

    public double headingToEncoder(double heading) {
        return inverseLut.get(heading);
    }
}
