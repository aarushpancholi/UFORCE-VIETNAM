package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxEPT;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.Iterator;

public class Shooter extends SubsystemBase {

    private PIDFController controller1, controller2;
    private TelemetryManager telemetry;
    private boolean autoShoot = false;
    private double pos = 0.5;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    private ServoEx hood;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;
    private static boolean isAuto;

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager, boolean isAuto) {
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = new ServoEx(hardwareMap, "hood");
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        I = 0.2;
        Shooter.isAuto = isAuto;
        P = 1.3;
        kS = 0.06;
        kV = 0.00039;
        controller1 = new org.firstinspires.ftc.teamcode.util.PIDFController(P,I,0.0, 0);
        controller2 = new org.firstinspires.ftc.teamcode.util.PIDFController(P,I,0.0, 0);

    }

    @Override
    public void periodic() {
        if (autoShoot) {
            targetVelocity = speedFromDistance(getGoalDistance(chosenAlliance));
            pos = angleFromDistance(getGoalDistance(chosenAlliance));
            setHood(pos);
        }
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel1", velocity1);
        telemetry.addData("CurrentVel2", velocity2);
        controller1.setPID(P,I, 0.0);
        controller1.setFeedforward(kV, 0.0, kS);
        controller2.setPID(P,I, 0.0);
        controller2.setFeedforward(kV, 0.0, kS);
        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
        sh.setPower(controller1.calculate(targetVelocity - velocity1, targetVelocity, 0.0));
        sh2.setPower(controller2.calculate(targetVelocity - velocity2, targetVelocity, 0.0));
        telemetry.update();
    }

    public void setAutoShoot(boolean on) {
        autoShoot = on;
    }

    public void setTargetEPT(double ept) {
        targetVelocity = ept;
    }

    public void setASpeed(double speed) {
        sh.setPower(speed);
    }
    public void setBSpeed(double speed) {
        sh2.setPower(speed);
    }


    public void setHood(double pos) {
        hood.set(pos); // immediate update + update() will maintain it
    }

    public double getHoodPos() {
        return hood.get();
    }

    // Use the documented MotorEx velocity getter. getVelocity() is ticks/sec by default. :contentReference[oaicite:3]{index=3}
    public int getVelA() { return (int) sh.getVelocity(); }
    public int getVelB() { return (int) sh2.getVelocity(); }


    public static double speedFromDistance(double d) {
        if (d > 0 && d < 47.8) {
            return (int) 1.41862*d+1041.18911;
        }
        else if (d>=47.8 && d < 70) {
            return (int) 7.70289*d+740.79915;
        }
        else if (d>=70) {
            return (int) (0.00122932*(Math.pow(d, 3)))-(0.318188*(Math.pow(d, 2)))+(30.10425*d+403.64484);
        }
        return 0;
    }

    public static double angleFromDistance(double d) {
        if (isAuto) {
            if (d > 0 && d < 47.8) {
                return 0.63;
            }
            else if (d>=47.8 && d < 70) {
                return 0.57;
            }
            else if (d>=70) {
                return 0.4;
            }
        }
        else {
            if (d > 0 && d < 47.8) {
                return 0.63;
            }
            else if (d>=47.8 && d < 70) {
                return 0.575;
            }
            else if (d>=70) {
                return 0.4;
            }
        }
        return 0.6;
    }
}
