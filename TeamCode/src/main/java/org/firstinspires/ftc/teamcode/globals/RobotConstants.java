package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RobotConstants {
    public static double maxHoodPos = 0.4;
    public static double minHoodPos = 0.8;

    public static double maxTurretPos = 336;
    public static double minTurretPos = 0;

    public MotorEx frontRight;
    public MotorEx frontLeft;
    public MotorEx backRight;
    public MotorEx backLeft;

    public ServoEx sIntakeA;
    public ServoEx sIntakeB;

    public SensorDistanceEx R1;
    public SensorDistanceEx R2;
    public SensorDistanceEx R3;

    public MotorEx intake;

    public MotorEx shooterA;
    public MotorEx shooterB;

    public static int maxEPT = 1900;

    public static Pose blueGoalPose = new Pose(10, 136, Math.toRadians(90));
    public static Pose redGoalPose  = new Pose(138, 136, Math.toRadians(90));


    public static final int RED_GOAL_TAG_ID = 24;
    public static final int BLUE_GOAL_TAG_ID = 20;

    public static String chosenAlliance = "RED";

    public static Pose intakeBlueRamp = new Pose(12.580710659898466, 59.021319796954295, Math.toRadians(150));
    public static Pose blueRampCP = new Pose(30.68223350253807, 58.12994923857866);
    public static Pose intakeRedRamp = new Pose(130.9969543147208, 59.45989847715734, Math.toRadians(30));
    public static Pose redRampCP = new Pose(103.19390862944162, 58.12994923857866);
    public static Pose bluePark = new Pose(53.076142131979694, 23.350253807106576, Math.toRadians(180));
    public static Pose redPark = new Pose(53.076142131979694, 23.350253807106576, Math.toRadians(0));



}
