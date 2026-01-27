//package org.firstinspires.ftc.teamcode.tests;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeTest;
//
//
//@Configurable
//@TeleOp(name = "Servo Test", group = "TeleOp")
//public class servoContinuousTest extends OpMode {
//
//    private TelemetryManager telemetryManager;
//    private Timer opModeTimer;
//
//    private Double speed = 0.0;
//    private IntakeTest intake;
//
//    @Override
//    public void init() {
//        opModeTimer = new Timer();
//        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//        intake = new IntakeTest(hardwareMap, telemetry);
//    }
//
//    @Override
//    public void start() {
//        opModeTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        intake.secondIntakeOn();
//
//        telemetry.update();
////        shooter.start();
//        // normal drive
////        follower.setTeleOpDrive(
////            -gamepad1.left_stick_y,
////            -gamepad1.left_stick_x,
////            -gamepad1.right_stick_x,
////            true);
//    }
//}
