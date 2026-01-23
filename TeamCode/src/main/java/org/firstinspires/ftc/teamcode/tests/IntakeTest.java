package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTest {
    private final DcMotorEx intakeR;
    private final CRServo servoA;
    private final CRServo servoB;
    private final Telemetry telemetry;

    public IntakeTest(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeR = hardwareMap.get(DcMotorEx.class, "intake");
        intakeR.setDirection(DcMotorEx.Direction.REVERSE);
        this.telemetry = telemetry;

        servoA = hardwareMap.get(CRServo.class, "sA");
        servoB = hardwareMap.get(CRServo.class, "sB");
        servoA.setDirection(DcMotorSimple.Direction.FORWARD);
        servoB.setDirection(DcMotorSimple.Direction.FORWARD);
//        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void intakeOn() {
        intakeR.setPower(1);
    }

    public void intakeOff() {
        intakeR.setPower(0);
    }

    public void intakeReverse() {
        intakeR.setPower(-1);
    }

    public void secondIntakeOn() {
        servoA.setPower(1);
        servoB.setPower(1);
    }

    public void secondIntakeOff() {
        servoA.setPower(0);
        servoB.setPower(0);
    }


    public void secondIntakeReverse() {
        servoA.setPower(-1);
        servoB.setPower(-1);
    }

}
