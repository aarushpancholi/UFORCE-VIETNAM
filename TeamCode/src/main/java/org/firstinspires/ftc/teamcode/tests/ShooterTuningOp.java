package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTest;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter tuning", group = "TeleOp")
public class ShooterTuningOp extends OpMode {
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
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
        intake = new IntakeTest(hardwareMap, null);
        Localization.init(follower, telemetry);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setTargetEPT(speed);
        shooter.setHood(minHoodPos);
        follower.startTeleOpDrive();
        turret.straight();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        telemetry.addData("encoder", turret.getPos());
        if (gamepad1.a) {
            intake.intakeOn();
            intake.secondIntakeOn();
        }
        if (gamepad1.b) {
            intake.intakeOff();
            intake.secondIntakeOff();
        }


        Localization.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.right_bumper) {
            follower.holdPoint(follower.getPose());
        }
        if (gamepad1.left_bumper) {
            follower.startTeleOpDrive();
        }


        if (gamepad1.dpadUpWasReleased()) {
            speed += 25;
        }
        if (gamepad1.dpadDownWasReleased()) {
            speed -= 25;
        }

        if (gamepad1.dpadLeftWasReleased()) {
            shooter.setHood(shooter.getHoodPos() + 0.02);
        }
        if (gamepad1.dpadRightWasReleased()) {
            shooter.setHood(shooter.getHoodPos() - 0.02);
        }
        shooter.setTargetEPT(speed);

        telemetry.addData("shooter speed", speed);
        telemetry.addData("hood pos", shooter.getHoodPos());
        telemetry.addData("distance from red", getRedDistance());

        turret.periodic();
        shooter.periodic();
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setHood(minHoodPos);
    }
}
