package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class AutoTestV1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        MSTART_INTAKE1,
        INTAKE1_INTAKE1END,
        INTAKE1END_SHOOT,
        SHOOT_INTAKE2,
        INTAKE2_EINTAKE2,
        EINTAKE2_SHOOT2,
        SHOOT2_INTAKE3,
        INTAKE3_PREINTAKE3,
        PREINTAKE3_EINTAKE3,
        EINTAKE3_SHOOTFINAL,
        SHOOTFINAL_HPINTAKE,
        HPINTAKE_SHOOT,
        END
    }

    PathState pathState;

    // Poses extracted from the visualizer video
    // Note: Converted degrees to radians for headings
    private final Pose startPose = new Pose(58.769543147208125, 7.89441624365482, Math.toRadians(90));
    private final Pose intake1 = new Pose(41.66497461928934, 35.086294416243646, Math.toRadians(180));
    private final Pose eIntake1 = new Pose(9.648730964467006, 35.37868020304568, Math.toRadians(180));
    private final Pose shoot = new Pose(58.769543147208125, 8.917766497461933, Math.toRadians(180));

    // New Poses (Paths 4-10)
    private final Pose intake2 = new Pose(42.542131979695434, 59.20812182741117, Math.toRadians(180));
    private final Pose eIntake2 = new Pose(15.350253807106599, 59.06192893401016, Math.toRadians(180));
    private final Pose shoot2 = new Pose(51.16751269035533, 83.4761421319797, Math.toRadians(180));
    private final Pose shoot2CP1 = new Pose(45.61218274111675, 59.06192893401016);
    private final Pose intake3 = new Pose(15.78883241, 83.91472081, Math.toRadians(180));
    private final Pose preIntake3 = new Pose(22.6598984, 69.8802030, Math.toRadians(90));
    private final Pose eIntake3 = new Pose(15.0578680, 69.1492385, Math.toRadians(90));
    private final Pose shootFinal = new Pose(58.47715736040609, 10.964467005076143, Math.toRadians(180));

    private final Pose hpIntake = new Pose(11.110659898477158, 9.063959390862943, Math.toRadians(180));

    // PathChains
    private PathChain startPose_intake1, intake1_eIntake1, eIntake1_shoot;
    private PathChain shoot_intake2, intake2_eIntake2, eIntake2_shoot2;
    private PathChain shoot2_intake3, intake3_preIntake3, preIntake3_eIntake3, eIntake3_shootFinal;

    private PathChain shootFinal_hpIntake, hpIntake_shootHp;

    public void buildPaths() {
        startPose_intake1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intake1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intake1.getHeading())
                .build();

        intake1_eIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, eIntake1))
                .setLinearHeadingInterpolation(intake1.getHeading(), eIntake1.getHeading())
                .build();

        eIntake1_shoot = follower.pathBuilder()
                .addPath(new BezierLine(eIntake1, shoot))
                .setLinearHeadingInterpolation(eIntake1.getHeading(), shoot.getHeading())
                .build();

        // New Paths
        shoot_intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot, intake2))
                .setLinearHeadingInterpolation(shoot.getHeading(), intake2.getHeading())
                .build();

        intake2_eIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, eIntake2))
                .setLinearHeadingInterpolation(intake2.getHeading(), eIntake2.getHeading())
                .build();

        eIntake2_shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(eIntake2, shoot2CP1, shoot2))
                .setLinearHeadingInterpolation(eIntake2.getHeading(), shoot2.getHeading())
                .build();

        shoot2_intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, intake3))
                .setLinearHeadingInterpolation(shoot2.getHeading(), intake3.getHeading())
                .build();

        intake3_preIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, preIntake3))
                .setLinearHeadingInterpolation(intake3.getHeading(), preIntake3.getHeading())
                .build();

        preIntake3_eIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(preIntake3, eIntake3))
                .setLinearHeadingInterpolation(preIntake3.getHeading(), eIntake3.getHeading())
                .build();

        eIntake3_shootFinal = follower.pathBuilder()
                .addPath(new BezierLine(eIntake3, shootFinal))
                .setLinearHeadingInterpolation(eIntake3.getHeading(), shootFinal.getHeading())
                .build();

        shootFinal_hpIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootFinal, hpIntake))
                .setLinearHeadingInterpolation(shootFinal.getHeading(), hpIntake.getHeading())
                .build();

        hpIntake_shootHp = follower.pathBuilder()
                .addPath(new BezierLine(hpIntake, shoot))
                .setLinearHeadingInterpolation(hpIntake.getHeading(), shoot.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case MSTART_INTAKE1:
                follower.followPath(startPose_intake1, true);
                setPathState(PathState.INTAKE1_INTAKE1END);
                break;
            case INTAKE1_INTAKE1END:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(intake1_eIntake1, true);
                    setPathState(PathState.INTAKE1END_SHOOT);
                }
                break;
            case INTAKE1END_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(eIntake1_shoot, true);
                    setPathState(PathState.SHOOT_INTAKE2);
                }
                break;
            case SHOOT_INTAKE2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(shoot_intake2, true);
                    setPathState(PathState.INTAKE2_EINTAKE2);
                }
                break;
            case INTAKE2_EINTAKE2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(intake2_eIntake2, true);
                    setPathState(PathState.EINTAKE2_SHOOT2);
                }
                break;
            case EINTAKE2_SHOOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(eIntake2_shoot2, true);
                    setPathState(PathState.SHOOT2_INTAKE3);
                }
                break;
            case SHOOT2_INTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(shoot2_intake3, true);
                    setPathState(PathState.INTAKE3_PREINTAKE3);
                }
                break;
            case INTAKE3_PREINTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(intake3_preIntake3, true);
                    setPathState(PathState.PREINTAKE3_EINTAKE3);
                }
                break;
            case PREINTAKE3_EINTAKE3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(preIntake3_eIntake3, true);
                    setPathState(PathState.EINTAKE3_SHOOTFINAL);
                }
                break;
            case EINTAKE3_SHOOTFINAL:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(eIntake3_shootFinal, true);
                    setPathState(PathState.SHOOTFINAL_HPINTAKE);
                }
                break;
            case SHOOTFINAL_HPINTAKE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(shootFinal_hpIntake, true);
                    setPathState(PathState.HPINTAKE_SHOOT);
                }
                break;
            case HPINTAKE_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    follower.followPath(hpIntake_shootHp, true);
                    setPathState(PathState.END);
                }
                break;
            case END:
                telemetry.addLine("Paths done!");
                break;
            default:
                telemetry.addLine("Switch default.");
        }
    }

    public void setPathState(PathState endState) {
        pathState = endState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        // Set initial pose
        follower.setPose(startPose);
        pathState = PathState.MSTART_INTAKE1;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.MSTART_INTAKE1);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}