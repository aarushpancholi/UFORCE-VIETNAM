package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RobotConstants {
    public static double maxHoodPos = 1;
    public static double minHoodPos = 0;

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

    public static Pose blueGoalPose = new Pose(6, 130, Math.toRadians(90));
    public static Pose redGoalPose  = new Pose(135, 150, Math.toRadians(90));



}
