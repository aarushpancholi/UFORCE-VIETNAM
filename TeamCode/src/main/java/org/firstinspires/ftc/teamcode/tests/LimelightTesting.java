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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTest;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@Configurable
@TeleOp(name = "ll testing", group = "TeleOp")
public class LimelightTesting extends OpMode {

    private TelemetryManager telemetry;
    private AprilTagTracking vision;

    @Override
    public void init() {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        vision = new AprilTagTracking(hardwareMap);


        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        telemetry.addData("limelight", vision.getYawErrorRadToGoal("red"));
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
