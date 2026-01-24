package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intake extends SubsystemBase {
    private final DigitalChannel sensor01;
    private final DigitalChannel sensor02;
    private final DigitalChannel sensor03;

    private final DcMotorEx IntakeMotor;
    private final CRServo ServoMotor1;
    private final CRServo ServoMotor2;

    private TelemetryManager telemetry;

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        sensor01 = hardwareMap.get(DigitalChannel.class, "sensor01");
        sensor01.setMode(DigitalChannel.Mode.INPUT);

        sensor02 = hardwareMap.get(DigitalChannel.class, "sensor02");
        sensor02.setMode(DigitalChannel.Mode.INPUT);

        sensor03 = hardwareMap.get(DigitalChannel.class, "sensor03");
        sensor03.setMode(DigitalChannel.Mode.INPUT);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        IntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        ServoMotor1 = hardwareMap.get(CRServo.class, "servoMotor1");
        ServoMotor2 = hardwareMap.get(CRServo.class, "servoMotor2");

        ServoMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoMotor2.setDirection(DcMotorSimple.Direction.FORWARD);



        telemetry = telemetryManager;
    }

    @Override
    public void periodic() {

    }

    public void intakeOff(){
        IntakeMotor.setPower(0.0);
        ServoMotor1.setPower(0.0);
        ServoMotor2.setPower(0.0);
    }

    public void Intake1On(){
        IntakeMotor.setPower(1.0);
    }

    public void Intake2On(){
        ServoMotor1.setPower(1.0);
        ServoMotor2.setPower(1.0);
    }
    public boolean isBallDetected01() {
        return !sensor01.getState();
    }

    public boolean isBallDetected02() {
        return !sensor02.getState();
    }

    public boolean isBallDetected03() {
        return !sensor03.getState();
    }

    public boolean areAllBallsDetected() {
        return isBallDetected01() && isBallDetected02() && isBallDetected03();
    }

    public boolean canIntake() {
        return !areAllBallsDetected();
    }

    public void testSensors() {
        int status01 = isBallDetected01() ? 1 : 0;
        int status02 = isBallDetected02() ? 1 : 0;
        int status03 = isBallDetected03() ? 1 : 0;

        System.out.println("Sensor 01: " + status01);
        System.out.println("Sensor 02: " + status02);
        System.out.println("Sensor 03: " + status03);
    }

    public void autoIntake() {
        // If all 3 slots are full, stop everything
        if (areAllBallsDetected()) {
            intakeOff();
            return;
        }

        // If slot 3 is occupied, disable Intake2 (CRServos), but Intake1 can keep running
        if (isBallDetected03()) {
            IntakeMotor.setPower(1.0);     // Intake1 ON
            ServoMotor1.setPower(0.0);     // Intake2 OFF
            ServoMotor2.setPower(0.0);     // Intake2 OFF
            return;
        }

        // Otherwise, we can run both intakes to collect balls
        IntakeMotor.setPower(1.0);         // Intake1 ON
        ServoMotor1.setPower(1.0);         // Intake2 ON
        ServoMotor2.setPower(1.0);         // Intake 2 ON
    }

}