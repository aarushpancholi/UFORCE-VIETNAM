// Turret.java
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minTurretPos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.OptionalDouble;

@Configurable
public class Turret extends SubsystemBase {
    private final MotorEx turret;
    private final AprilTagTracking vision;

    public boolean autoAimEnabled = false;

    // Turret mechanical range in degrees: [-135, +135]
    private static final double MIN_TURRET_RAD = Math.toRadians(-135);
    private static final double MAX_TURRET_RAD = Math.toRadians(135);

    // Heading prediction (helps while rotating)
    private static final double HEADING_LEAD_SEC = 0.12; // tune 0.03–0.15

    // Limelight deadband to prevent oscillation (±1°)
    private static final double VISION_DEADBAND_RAD = Math.toRadians(1.0);
    public static double kP = 0.011;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kF = 0.0002;
    // PIDF (tune these)
    public static PIDFController turretPID = new PIDFController(
            kP, kI, kD, kF
    );

    private static final int TICKS_TOLERANCE = 5;
    private double maxPower = 1;

    public static int targetTicks = 168;

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        turret = new MotorEx(hardwareMap, "turret");
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setInverted(true);

        // Limelight vision (safe init, no start here)
        vision = new AprilTagTracking(hardwareMap);
    }

    /** Call once from OpMode init() after constructing Turret */
    public void startVision() {
        vision.start();
    }

    public void straight() {
        setTargetTicks(168);
    }

    public boolean isStraight() {
        return (turret.getCurrentPosition() > 165 && turret.getCurrentPosition() < 171);
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
        if (!autoAimEnabled) return;

        // Robot heading + lead prediction
        double robotHeading = Localization.getHeading();          // rad
        double omega = Localization.getHeadingVelocity();         // rad/s
        double robotHeadingPred = normalizeRadians(robotHeading + omega * HEADING_LEAD_SEC);

        // Current turret headings
        double turretRelHeading = posToHeading(getPos());         // rad in [-135,+135]
        double turretAbsHeading = normalizeRadians(robotHeadingPred + turretRelHeading);

        // Localization-based aim error (field-space)
        double localErr = getGoalHeadingDiff(turretAbsHeading, chosenAlliance);    // rad, +CCW

        // Vision-based aim error (camera-space) as +CCW rad
        OptionalDouble visionErrOpt = vision.getYawErrorRadToGoal(chosenAlliance);

        // If tag visible -> use ONLY limelight error (with deadband)
        // Else -> use localization
        double aimErr;
        if (visionErrOpt.isPresent()) {
            double visionErr = visionErrOpt.getAsDouble();
            aimErr = (Math.abs(visionErr) <= VISION_DEADBAND_RAD) ? 0.0 : visionErr;
        } else {
            aimErr = localErr;
        }

        // Convert aim error to a desired absolute heading
        double targetAbsHeading = normalizeRadians(turretAbsHeading + aimErr);

        // Convert to desired turret-relative angle, handle wrap + bounds behavior
        double relToTarget = targetAbsHeading - robotHeadingPred; // intentionally not normalized first
        double chosenRel = chooseTurretRelHeading(relToTarget, turretRelHeading);

        // Convert to ticks and clamp
        targetTicks = (int) Range.clip(headingToPos(chosenRel), minTurretPos, maxTurretPos);

        targetTicks=168;

        // PIDF to targetTicks
        turretPID.setSetPoint(targetTicks);

        double current = turret.getCurrentPosition();
        double power = turretPID.calculate(current);
        power = Range.clip(power, -maxPower, maxPower);

//        double err = targetTicks - current;
//        if (Math.abs(err) > TICKS_TOLERANCE && Math.abs(power) < 0.2) {
//            power = Math.copySign(0.2, power);
//        }
//
//
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
        double base = normalizeRadians(relRad);

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

        // Otherwise clamp each candidate, pick closest to current
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
