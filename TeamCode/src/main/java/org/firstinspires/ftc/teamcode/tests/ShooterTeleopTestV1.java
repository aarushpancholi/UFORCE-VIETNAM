package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTest;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter test v1", group = "TeleOp")
public class ShooterTeleopTestV1 extends OpMode {
    private Shooter shooter;
    private double speed;
    private IntakeTest intake;
    private Follower follower;
    private Turret turret;
    private TelemetryManager telemetry;
    private List<Double> speeds;

    @Override
    public void init() {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, null);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(90)));
        intake = new IntakeTest(hardwareMap, null);
        Localization.init(follower, telemetry);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setSpeedRpm(speed);
        turret.setAutoAim(true);
        follower.startTeleOpDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
//        if (gamepad1.a) {
//            intake.intakeOn();
//        }
//        if (gamepad1.b) {
//            intake.intakeReverse();
//        }

        Localization.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
//        turret.periodic();

//        if (gamepad1.dpad_left) {
//            intake.secondIntakeReverse();
//        }
//        if (gamepad1.x) {
//            intake.secondIntakeOff();
//        }
        if(gamepad1.dpad_right) {intake.secondIntakeOn();}
        if (gamepad1.dpadUpWasReleased()) {
            speed += 0.5;
        }
        if (gamepad1.dpadDownWasReleased()) {
            speed -= 0.5;
        }
        shooter.setSpeedRpm(speed);

//        telemetry.addData("encoder", turret.getPos());
        shooter.periodic();
        speeds = shooter.getSpeed();
        telemetry.addData("speeds", speeds);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
