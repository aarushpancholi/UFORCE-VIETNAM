package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getRedHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minTurretPos;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;

public class Turret extends SubsystemBase {
    private final MotorEx turret;

    private boolean autoAimEnabled = false;

    // Turret mechanical range in degrees: [-135, +135]
    private static final double MIN_TURRET_RAD = Math.toRadians(-135);
    private static final double MAX_TURRET_RAD = Math.toRadians(135);

    // Lead compensation (Fix 2): predicts heading slightly forward
    private static final double HEADING_LEAD_SEC = 0.08; // tune 0.03–0.12

    // PIDF (tune these)
    private final PIDFController turretPID = new PIDFController(
            0.012,   // kP
            0.0,    // kI
            0.001,  // kD
            0.0     // kF
    );

    private static final int TICKS_TOLERANCE = 7;
    private double maxPower = 1;

    private int targetTicks = 168;

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        turret = new MotorEx(hardwareMap, "turret");
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setInverted(true);
    }

    public void straight() {
        setTargetTicks(168);
    }

    public void setTargetTicks(int ticks) {
        targetTicks = (int) Range.clip(ticks, minTurretPos, maxTurretPos);
    }

    public void resetTurretEncoder() {
        turret.stopAndResetEncoder();
    }

    public double getPos() {
        return turret.getCurrentPosition();
    }

    public void setAutoAim(boolean enabled) {
        autoAimEnabled = enabled;
        if (!enabled) {
            turret.set(0);
            turretPID.reset();
        }
    }

    @Override
    public void periodic() {
        if (autoAimEnabled) {
            // Robot heading + lead prediction (Fix 2)
            double robotHeading = Localization.getHeading();           // rad
            double omega = Localization.getHeadingVelocity();          // rad/s
            double robotHeadingPred = AngleUnit.normalizeRadians(robotHeading + omega * HEADING_LEAD_SEC);

            // Current turret headings
            double turretRelHeading = posToHeading(getPos());          // rad in [-135,+135]
            double turretAbsHeading = AngleUnit.normalizeRadians(robotHeadingPred + turretRelHeading);

            // Target absolute heading on field
            double headingDiffToGoal = getRedHeadingDiff(turretAbsHeading); // rad
            double targetAbsHeading = AngleUnit.normalizeRadians(turretAbsHeading + headingDiffToGoal);

            // Convert to desired turret-relative angle, handle wrap + bounds behavior
            double relToTarget = targetAbsHeading - robotHeadingPred;  // intentionally not normalized first
            double chosenRel = chooseTurretRelHeading(relToTarget, turretRelHeading);

            // Convert to ticks and clamp
            targetTicks = (int) Range.clip(headingToPos(chosenRel), minTurretPos, maxTurretPos);
        }

        // PIDF to targetTicks
        turretPID.setSetPoint(targetTicks);

        double current = turret.getCurrentPosition();
        double power = turretPID.calculate(current);
        power = Range.clip(power, -maxPower, maxPower);

        // Grinding protection: don't push further into a hard limit
        int pos = (int) current;
        boolean atMin = pos <= (minTurretPos + 2);
        boolean atMax = pos >= (maxTurretPos - 2);
        if ((atMin && power < 0) || (atMax && power > 0)) {
            power = 0;
            turretPID.reset();
        }

        turret.set(power);
    }

    public boolean isAimed() {
        return Math.abs(targetTicks - turret.getCurrentPosition()) <= TICKS_TOLERANCE;
    }

    // -----------------------------
    // Linear mapping (ticks <-> radians)
    // -----------------------------

    private double posToHeading(double posTicks) {
        double pos = Range.clip(posTicks, minTurretPos, maxTurretPos);
        double t = (pos - minTurretPos) / (maxTurretPos - minTurretPos); // 0..1
        return MIN_TURRET_RAD + t * (MAX_TURRET_RAD - MIN_TURRET_RAD);
    }

    private double headingToPos(double headingRad) {
        double h = Range.clip(headingRad, MIN_TURRET_RAD, MAX_TURRET_RAD);
        double t = (h - MIN_TURRET_RAD) / (MAX_TURRET_RAD - MIN_TURRET_RAD); // 0..1
        return minTurretPos + t * (maxTurretPos - minTurretPos);
    }

    // -----------------------------
    // Angle selection / bounds behavior
    // -----------------------------

    private double chooseTurretRelHeading(double relRad, double currentTurretRelRad) {
        double base = AngleUnit.normalizeRadians(relRad);

        double[] candidates = new double[] { base, base + 2.0 * Math.PI, base - 2.0 * Math.PI };

        // If reachable, pick the in-range candidate closest to current turret angle
        double bestInRange = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            if (c >= MIN_TURRET_RAD && c <= MAX_TURRET_RAD) {
                double d = Math.abs(c - currentTurretRelRad);
                if (d < bestDist) {
                    bestDist = d;
                    bestInRange = c;
                }
            }
        }
        if (!Double.isNaN(bestInRange)) return bestInRange;

        // Otherwise "stick" to the closest bound (clamp each candidate, pick closest to current)
        double bestClamped = 0.0;
        bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            double clamped = Range.clip(c, MIN_TURRET_RAD, MAX_TURRET_RAD);
            double d = Math.abs(clamped - currentTurretRelRad);
            if (d < bestDist) {
                bestDist = d;
                bestClamped = clamped;
            }
        }
        return bestClamped;
    }
}
