// AprilTagTracking.java  (Limelight-based, crash-proof)
package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.OptionalDouble;

public class AprilTagTracking {

    // Must match the device name in the RC configuration exactly
    private static final String LIMELIGHT_NAME = "limelight";

    // Adjust if your target IDs differ
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
            telemetry.addData("limelight", result.getTx());
            return OptionalDouble.of(-Math.toRadians(result.getTx()));
        }
        else {
            return OptionalDouble.empty();
        }
    }
}
