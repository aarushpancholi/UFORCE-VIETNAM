package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intake extends SubsystemBase {

    // ------------------------------
    // Ranger in 15° FOV mode = ANALOG VOLTAGE
    // ------------------------------
    private final AnalogInput s1;
    private final AnalogInput s2;
    private final AnalogInput s3;

    // ------------------------------
    // Hardware
    // ------------------------------
    private final DcMotorEx intakeMotor;
    private final CRServo servoA;

    private final CRServo servoB;
    private final ServoEx stopper;

    private final TelemetryManager telemetry;

    // ------------------------------
    // TUNING (YOU MUST CALIBRATE)
    // ------------------------------
    // Pick these from telemetry:
    // - Measure voltage with NO ball (V_empty)
    // - Measure voltage with ball present (V_ball)
    // - Set threshold between them (average is a good start)
    //
    // If you discover voltage goes DOWN when ball is present, flip the comparisons (see NOTE below).
    private static final double S1_DETECT_V = 0.18;  // placeholder
    private static final double S2_DETECT_V = 0.18;  // placeholder
    private static final double S3_DETECT_V = 0.18;  // placeholder

    // Hysteresis band (reduces flicker at threshold)
    private static final double HYST_V = 0.05;

    // If true: "detected" means voltage >= threshold (common)
    // If false: "detected" means voltage <= threshold (some sensors behave like this)
    private static final boolean DETECT_IS_HIGHER_V = false;

    // Latched states (memory) for hysteresis
    private boolean s1Detected = false;
    private boolean s2Detected = false;
    private boolean s3Detected = false;

    private boolean all3 = false;

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {

        // Analog inputs (make sure Ranger analog out is plugged into REV ANALOG ports)
        s1 = hardwareMap.get(AnalogInput.class, "s1");
        s2 = hardwareMap.get(AnalogInput.class, "s2");
        s3 = hardwareMap.get(AnalogInput.class, "s3");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        stopper = new ServoEx(hardwareMap, "stopper");

        servoA = hardwareMap.get(CRServo.class, "sA");
        servoB = hardwareMap.get(CRServo.class, "sB");


        servoA.setDirection(DcMotorSimple.Direction.FORWARD);

        servoB.setDirection(DcMotorSimple.Direction.FORWARD
        );

        telemetry = telemetryManager;
    }

    @Override
    public void periodic() {
        // Optional: you could call testSensors() here if you want constant telemetry,
        // but it may spam. Usually call it from your OpMode loop.
    }

    // ------------------------------
    // Actuators
    // ------------------------------
    public void setStopper(double pos) {
        stopper.set(pos);
    }

    public double getStopper() {
        return stopper.get();
    }

    public void intakeOff() {
        intakeMotor.setPower(0.0);
//        servoA.setPower(0.0);
//        servoB.setPower(0.0);
    }

    public void intake1On() {
        intakeMotor.setPower(1.0);
    }

    public void intakeOpposite() {
        intakeMotor.setPower(-1.0);
//        servoA.setPower(-1.0);
//        servoB.setPower(-1.0);

    }

    public void intake2On() {
//        servoA.setPower(1.0);
//        servoB.setPower(1.0);
        intakeMotor.setPower(1);

    }


    public void intake2Off() {
//        servoA.setPower(0.0);
//        servoB.setPower(0.0);
//        intakeMotor.setPower(1.0);
    }

    // ------------------------------
    // Detection helpers (voltage -> boolean) with hysteresis
    // ------------------------------
    private boolean detectWithHysteresisHigherIsDetected(double v, double threshold, boolean prevState) {
        // ON when v >= threshold + HYST
        // OFF when v <  threshold - HYST
        if (prevState) return v >= (threshold - HYST_V);
        return v >= (threshold + HYST_V);
    }

    private boolean detectWithHysteresisLowerIsDetected(double v, double threshold, boolean prevState) {
        // ON when v <= threshold - HYST
        // OFF when v >  threshold + HYST
        if (prevState) return v <= (threshold + HYST_V);
        return v <= (threshold - HYST_V);
    }

    private boolean computeDetected(double v, double threshold, boolean prevState) {
        return DETECT_IS_HIGHER_V
                ? detectWithHysteresisHigherIsDetected(v, threshold, prevState)
                : detectWithHysteresisLowerIsDetected(v, threshold, prevState);
    }

    // ------------------------------
    // Public detection API (same logic as your digital version)
    // ------------------------------
    public boolean isBallDetected01() {
        return ((s1.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean isBallDetected02() {
        return ((s2.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean isBallDetected03() {
        return ((s3.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean areAllBallsDetected() {
        return isBallDetected01() && isBallDetected02() && isBallDetected03();
    }

    public boolean noBalls() {
        return !isBallDetected01() && !isBallDetected02() && !isBallDetected03();
    }

    public boolean canIntake() {
        return !areAllBallsDetected();
    }

    // ------------------------------
    // Telemetry / Debug
    // ------------------------------
    public void testSensors() {
        double v1 = s1.getVoltage();
        double v2 = s2.getVoltage();
        double v3 = s3.getVoltage();

        boolean d1 = isBallDetected01();
        boolean d2 = isBallDetected02();
        boolean d3 = isBallDetected03();

//        telemetry.addData("Mode", "Ranger 15° FOV (Analog)");
//        telemetry.addData("Detect higher V?", DETECT_IS_HIGHER_V);
//
//        telemetry.addData("S1 V", v1);
//        telemetry.addData("S1 Det", d1 ? 1 : 0);
//
//        telemetry.addData("S2 V", v2);
//        telemetry.addData("S2 Det", d2 ? 1 : 0);
//
//        telemetry.addData("S3 V", v3);
//        telemetry.addData("S3 Det", d3 ? 1 : 0);

        telemetry.addData("S1", d1);
        telemetry.addData("S2", d2);
        telemetry.addData("S3", d3);

        telemetry.update();
    }

    // ------------------------------
    // Intake status helpers
    // ------------------------------
    public boolean isIntake1On() {
        return intakeMotor.getPower() > 0.4;
    }

    public boolean isIntake2On() {
        return intakeMotor.getPower() > 0.4; // assumes both are driven the same
    }

    public boolean isIntakeOff() {
//        return intakeMotor.getPower() < 0.1 && Math.abs(servoA.getPower()) < 0.1 && Math.abs(servoB.getPower()) < 0.1;
        return intakeMotor.getPower() < 0.1;
    }

    // ------------------------------
    // Auto intake logic (same as your original)
    // ------------------------------

    public void intakeReset() {
        all3 = false;
    }
    public void autoIntake() {
        if (!all3) {

            // If all 3 slots are full, stop everything
            if (areAllBallsDetected()) {
                intakeOff();
                all3 = true;
                return;
            }

            // If slot 3 is occupied, disable Intake2 (CRServos), but Intake1 can keep running
            if (isBallDetected03()) {
                intakeMotor.setPower(1.0);   // Intake1 ON
                servoA.setPower(0.0);        // Intake2 OFF
                servoB.setPower(0.0);        // Intake2 OFF
                return;
            }

            // Otherwise, run both intakes to collect balls
            intakeMotor.setPower(1.0);       // Intake1 ON
            servoA.setPower(1.0);            // Intake2 ON
            servoB.setPower(1.0);
        }
    }
}
