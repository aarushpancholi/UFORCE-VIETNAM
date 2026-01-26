package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.PIDFController;

@Configurable
public class ShooterNewPIDTest extends SubsystemBase {
    private PIDFController controller1, controller2;
    private TelemetryManager telemetry;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    private ServoEx hood;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;


    public ShooterNewPIDTest(HardwareMap hardwareMap) {
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = new ServoEx(hardwareMap, "hood");
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        I = 0.2;
        P = 1.3;
        kS = 0.06;
        kV = 0.00039;
        controller1 = new PIDFController(P,I,0.0, 0);
        controller2 = new PIDFController(P,I,0.0, 0);
    }

    @Override
    public void periodic() {
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

    public void setShooterTarget(int targetSpeed) {
        targetVelocity = targetSpeed;
    }

    public void setHood(double pos) {
        hood.set(pos);
    }
}
