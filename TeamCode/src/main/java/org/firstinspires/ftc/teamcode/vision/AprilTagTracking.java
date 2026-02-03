// AprilTagTracking.java  (Limelight-based, crash-proof)
package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.OptionalDouble;

public class AprilTagTracking {

    private static final String LIMELIGHT_NAME = "limelight";

    private static final int RED_GOAL_TAG_ID = 24;
    private static final int BLUE_GOAL_TAG_ID = 20;

    private static final double TX_DEADBAND_DEG = 1.0;

    private final TelemetryManager telemetry;

    private Limelight3A limelight;
    private boolean available = false;
    private boolean started = false;

    public AprilTagTracking(HardwareMap hardwareMap) {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(60);
        limelight.start();
        limelight.pipelineSwitch(1);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void start() {

    }

    public OptionalDouble getYawErrorRadToGoal(String goal) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            if (result.getFiducialResults().get(0).getFiducialId() == 24 && goal.equals("RED")) {
                telemetry.addData("limelight", result.getTx());
                return OptionalDouble.of(-Math.toRadians(result.getTx()));
            }
            else if (result.getFiducialResults().get(0).getFiducialId() == 20 && goal.equals("BLUE")) {
                telemetry.addData("limelight", result.getTx());
                return OptionalDouble.of(-Math.toRadians(result.getTx()));
            }
            else {
                return OptionalDouble.empty();
            }
        }
        else {
            return OptionalDouble.empty();
        }
    }
    
//    public Pose getLLPose() {
//        Pose3D botPose;
//        Pose llPose = null;
//        LLResult result = limelight.getLatestResult();
//        if (result.isValid()) {
//            botPose = result.getBotpose();
//            llPose = new Pose(botPose.getPosition().x, botPose.getPosition().y);
//        }
//
//        return llPose;
//    }
}
