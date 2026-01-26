package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxEPT;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodPos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Iterator;

public class Shooter extends SubsystemBase {

    private final MotorEx shooterA;
    private final MotorEx shooterB;
    private final MotorGroup shooterMotors;
    private final ServoEx hood;
    private final VoltageSensor battery;

    // Start F at ~1/maxEPT, then tune P. D optional, I usually 0 for flywheels.
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(
            0.0002,            // P (start small)
            0.0,               // I
            0.0,               // D
            0.0003      // F (feedforward baseline)
    );

    private final PIDFController flywheelController = new PIDFController(SHOOTER_PIDF);

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double VEL_TOLERANCE_TPS = 60.0;

    private boolean activeControl = true;
    private double targetEPT = 0.0;

    private boolean voltageCompOutput = true;

    private final TelemetryManager telemetry;

    InterpLUT distToSpeed = new InterpLUT();
    InterpLUT distToAngle = new InterpLUT();

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        shooterA = new MotorEx(hardwareMap, "rsh", Motor.GoBILDA.BARE);
        shooterB = new MotorEx(hardwareMap, "lsm", Motor.GoBILDA.BARE);

        // External PIDF outputs POWER -> use RawPower run mode. :contentReference[oaicite:2]{index=2}
        shooterA.setRunMode(Motor.RunMode.RawPower);
        shooterB.setRunMode(Motor.RunMode.RawPower);

        shooterA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Make sure both spin the flywheel the SAME physical direction.
        shooterA.setInverted(false);
        shooterB.setInverted(false);

        shooterA.stopAndResetEncoder();
        shooterB.stopAndResetEncoder();

        shooterMotors = new MotorGroup(shooterA, shooterB);

        hood = new ServoEx(hardwareMap, "hood");

        battery = getBestVoltageSensor(hardwareMap);

        telemetry = (telemetryManager != null) ? telemetryManager : PanelsTelemetry.INSTANCE.getTelemetry();

        flywheelController.setTolerance(VEL_TOLERANCE_TPS);
        flywheelController.setSetPoint(0.0);
        distToAngle.add(23.69, 0.8);
        distToAngle.add(32.77, 0.72);
        distToAngle.add(40.55, 0.72);
        distToAngle.add(49.225, 0.659);
        distToAngle.add(55.81, 0.619);
        distToAngle.add(65.63, 0.599);
        distToAngle.add(78.92, 0.539);
        distToAngle.add(94.46, 0.539);
        distToAngle.add(98.77, 0.519);
        distToAngle.add(110.69, 0.518);
        distToAngle.add(118.22, 0.518);
        distToAngle.add(131.76, 0.478);

        distToSpeed.add(23.69, 1000);
        distToSpeed.add(32.77, 1050);
        distToSpeed.add(40.55, 1050);
        distToSpeed.add(49.225, 1100);
        distToSpeed.add(55.81, 1100);
        distToSpeed.add(65.63, 1150);
        distToSpeed.add(78.92, 1225);
        distToSpeed.add(94.46, 1275);
        distToSpeed.add(98.77, 1300);
        distToSpeed.add(110.69, 1375);
        distToSpeed.add(118.22, 1425);
        distToSpeed.add(131.76, 1475);

        distToSpeed.createLUT();
        distToAngle.createLUT();
    }

    @Override
    public void periodic() {
        update();
    }

    public void setTargetEPT(double ept) {
        targetEPT = clamp(ept, 0.0, maxEPT);
        flywheelController.setSetPoint(targetEPT);
    }

    public void stop() {
        targetEPT = 0.0;
        flywheelController.setSetPoint(0.0);
        shooterMotors.set(0.0);
    }

    public void setHood(double pos) {
        hood.set(pos); // immediate update + update() will maintain it
    }

    public double getHoodPos() {
        return hood.get();
    }

    // Use the documented MotorEx velocity getter. getVelocity() is ticks/sec by default. :contentReference[oaicite:3]{index=3}
    public double getVelA() { return shooterA.getVelocity(); }
    public double getVelB() { return shooterB.getVelocity(); }

    public double getFlywheelEPT() {
        return 0.5 * (getVelA() + getVelB());
    }

    public boolean atSpeed() {
        return activeControl && flywheelController.atSetPoint();
    }

    private void update() {
        // Maintain hood target (this is why setHood() must update targetHoodPos)

        if (activeControl) {
            // If one motor shows negative velocity while the other is positive, fix inversion.
            double vel = getFlywheelEPT();

            double out = flywheelController.calculate(vel); // power output (unbounded)

            if (voltageCompOutput) {
                double v = getBatteryVoltage();
                if (v > 1.0) out *= (NOMINAL_VOLTAGE / v);
            }


            double power = clamp(out, 0.0, 1.0);
            shooterMotors.set(power);

//            telemetry.addDat4a("Shooter/targetEPT", flywheelController.getSetPoint());
//            telemetry.addData("Shooter/velA", getVelA());
//            telemetry.addData("Shooter/velB", getVelB());
//            telemetry.addData("Shooter/velAvg", vel);
//            telemetry.addData("Shooter/power", power);
//            telemetry.addData("Shooter/Vbat", getBatteryVoltage());
        } else {
            shooterMotors.set(0.0);
        }

        telemetry.addData("Shooter/hood", getHoodPos());
        telemetry.addData("Shooter/atSpeed", atSpeed());
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static VoltageSensor getBestVoltageSensor(HardwareMap hardwareMap) {
        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        VoltageSensor best = it.hasNext() ? it.next() : null;

        while (it.hasNext()) {
            VoltageSensor s = it.next();
            if (best == null) { best = s; continue; }
            double bv = best.getVoltage();
            double sv = s.getVoltage();
            if (sv > 0.0 && (bv <= 0.0 || sv < bv)) best = s;
        }
        return best;
    }

    private double getBatteryVoltage() {
        return (battery != null) ? battery.getVoltage() : NOMINAL_VOLTAGE;
    }

    public double getSpeedFromDist(double dist) {
        return distToSpeed.get(dist);
    }

    public double getAngleFromDist(double dist) {
        return distToAngle.get(dist);
    }

    public static double angleFromDistance(double d) {
        d = clamp(d, 23.69, 131.76);
        return 0.93846278 + (-0.0070167848)*d + (0.000027995891)*d*d;
    }

    public static double speedFromDistance(double d) {
        d = clamp(d, 23.69, 131.76);
        return 941.55280 + 2.3652614*d + 0.0134337116*d*d;
    }
}
