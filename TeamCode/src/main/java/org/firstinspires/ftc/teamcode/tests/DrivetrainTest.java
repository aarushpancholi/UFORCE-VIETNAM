package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainTest {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private final ElapsedTime timer = new ElapsedTime();

    public DrivetrainTest(HardwareMap hardwareMap, TelemetryManager telemetry) {
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");   // CONTROL: port 0
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");     // CONTROL: port 1
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");       // CONTROL: port 2
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");     // CONTROL: port 3

        // Set motor directions (adjust if needed)
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reset timer
        timer.reset();
    }

    public void drive(double leftY, double leftX, double rightX,double drive_sens) {
        // Forward/Backward and Strafing
        double frontLeftPower = leftY + leftX + rightX;
        double frontRightPower = leftY - leftX - rightX;
        double backLeftPower = leftY - leftX + rightX;
        double backRightPower = leftY + leftX - rightX;

        // Apply sensitivity scaling
        frontLeftDrive.setPower(frontLeftPower / drive_sens);
        backLeftDrive.setPower(backLeftPower / drive_sens);
        frontRightDrive.setPower(frontRightPower / drive_sens);
        backRightDrive.setPower(backRightPower / drive_sens);
    }
}
