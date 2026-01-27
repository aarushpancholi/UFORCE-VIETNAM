package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.autoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.intakeOn2Command;
import org.firstinspires.ftc.teamcode.commands.setShooter;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretStraight;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;

@Configurable
@TeleOp(name = "TeleopMain", group = "TeleOp")
public class TeleopMain extends CommandOpMode {
    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private TelemetryManager telemetry;
    private Intake intake;

    @Override
    public void initialize() {
        super.reset();

        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(140,0,Math.toRadians(90)));
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        Localization.init(follower, telemetry);
        turret.setAutoAim(true);

        super.register(turret);
        super.register(shooter);


        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new autoIntakeCommand(intake))
                .whenReleased(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::intakeOff),
                                new InstantCommand(intake::intakeReset)
                        )
                );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new transfer(intake, true),
                                new ParallelCommandGroup(
                                        new intakeOn1Command(intake),
                                        new intakeOn2Command(intake)
                                )
                        )
                )
                        .whenReleased(
                                new ParallelCommandGroup(
                                        new transfer(intake, false),
                                        new intakeOn1Command(intake),
                                        new intakeOn2Command(intake)
                                )

                        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new turretStraight(turret));

        toolOp.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new intakeOn1Command(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new intakeOn2Command(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOpposite))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );



        telemetry.update();

    }

    @Override
    public void run() {
        super.run();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    }



