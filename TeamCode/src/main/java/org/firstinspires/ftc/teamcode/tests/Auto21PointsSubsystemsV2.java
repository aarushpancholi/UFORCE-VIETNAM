package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.setShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Auto21PointsV2")
public class Auto21PointsSubsystemsV2 extends CommandOpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Small delay after finishing a path before starting the next (tune as needed)
    private static final double STATE_DELAY_S = 0.2;

    // Headings (radians)
    private static final double H0  = Math.toRadians(0);
    private static final double H30 = Math.toRadians(30);

    // Path13 heading was "undefined" in your snippet; set to 0 by default.
    // Change this if Path13 needs a different heading.
    private static final double H13 = H0;


    public enum PathState {
        PATH1,
        PATH2,
        PATH3,
        PATH4,
        PATH5,
        PATH6,
        PATH7,
        PATH8,
        PATH9,
        PATH10,
        PATH11,
        PATH12,
        PATH13,
        END
    }

    private PathState pathState;

    // Reused poses (use the same variables for repeated points)
    private final Pose startPose = new Pose(85.385, 9.316, H0);

    // Path1 control points (no heading needed)
    private final Pose p1CP1 = new Pose(108.018, 67.244);
    private final Pose p1CP2 = new Pose(95.323, 59.203);

    private final Pose pEnd1 = new Pose(129.974, 59.829, H0);

    private final Pose pA = new Pose(81.260, 69.933, H0);     // used in Path2/3
    private final Pose pB = new Pose(131.458, 59.593, H30);    // used in Path3-10
    private final Pose pC = new Pose(81.188, 70.001, H30);     // used in Path4-9

    private final Pose pD = new Pose(76.948, 80.527, H30);     // Path10 start/end
    private final Pose pE = new Pose(127.006, 83.684, H0);     // Path11/12
    private final Pose pF = new Pose(92.171, 77.649, H0);      // Path12/13 start
    private final Pose pG = new Pose(92.105, 71.771, H13);     // Path13 end

    // PathChains
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, p1CP1, p1CP2, pEnd1))
                .setLinearHeadingInterpolation(H0, H0)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pEnd1, pA))
                .setLinearHeadingInterpolation(H0, H0)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pA, pB))
                .setLinearHeadingInterpolation(H0, H30)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pB, pC))
                .setConstantHeadingInterpolation(H30)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(pC, pB))
                .setConstantHeadingInterpolation(H30)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pB, pC))
                .setConstantHeadingInterpolation(H30)
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pC, pB))
                .setConstantHeadingInterpolation(H30)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pB, pC))
                .setConstantHeadingInterpolation(H30)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pC, pB))
                .setConstantHeadingInterpolation(H30)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pB, pD))
                .setConstantHeadingInterpolation(H30)
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pD, pE))
                .setLinearHeadingInterpolation(H30, H0)
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pE, pF))
                .setLinearHeadingInterpolation(H0, H0)
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierLine(pF, pG))
                .setConstantHeadingInterpolation(H13)
                .build();
    }




    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
