package org.firstinspires.ftc.teamcode.globals;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class Localization {
    private static Follower follower;

    public static void init(Follower f) {
        follower = f;
    }

    public static void update() {
        follower.update();
    }

    public static double getHeading() {
        return follower.getHeading();
    }

    public static double getX() {
        return follower.getPose().getX();
    }

    public static double getZLateral() {
        return follower.getPose().getY();
    }

    public static double getRedDistance() {
        return follower.getPose().distanceFrom(redGoalPose);
    }

    public static double getBlueDistance() {
        return follower.getPose().distanceFrom(blueGoalPose);
    }

    public static double getBlueHeadingDiff(double turretAbsHeading) {
        Pose robot = follower.getPose();
        double goalBearing = blueGoalPose.minus(robot).getAsVector().getTheta();
        return AngleUnit.normalizeRadians(goalBearing - turretAbsHeading);
    }

    public static double getRedHeadingDiff(double turretAbsHeading) {
        Pose robot = follower.getPose();
        double goalBearing = redGoalPose.minus(robot).getAsVector().getTheta();
        return AngleUnit.normalizeRadians(goalBearing - turretAbsHeading);
    }
}
