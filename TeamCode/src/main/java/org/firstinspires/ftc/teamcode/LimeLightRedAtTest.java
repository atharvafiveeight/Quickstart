package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.vision.LimeLight;

import java.util.List;

/**
 * Test class for LimeLight AprilTag detection
 * Specifically detects REDAT tags and prints telemetry
 */
@Autonomous
public class LimeLightRedAtTest extends OpMode {

    private LimeLight limeLight;

    @Override
    public void init() {
        // Initialize LimeLight with hardwareMap and pipeline ID
        limeLight = new LimeLight(hardwareMap, "limelight", 0);

        telemetry.addData("Status", "LimeLight initialized for REDAT detection");
        telemetry.addData("Pipeline ID", limeLight.getPipelineId());
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "LimeLight OpMode started");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Version", 3);
        telemetry.addData("Pipeline ID", limeLight.getPipelineId());

        // Get fiducial results
        List<LLResultTypes.FiducialResult> fiducialResults = limeLight.getAllFiducialResults();

        // Display all detected fiducials
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            telemetry.addData("Fiducials Detected", "None");
        } else {
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }

        // Check if REDAT target is detected using the new overloaded method
        double[] redatCoordinates = limeLight.getCoordinates(LimeLight.AprilTagType.REDAT);

        if (limeLight.isTargetDetected() && redatCoordinates[0] != 0.0 && redatCoordinates[1] != 0.0) {
            telemetry.addData("Status", "REDAT tag detected!");
            telemetry.addData("Detection Status", 1);

            // Get current coordinates for REDAT
            double coordX = redatCoordinates[0];
            double coordY = redatCoordinates[1];

            // Print telemetry information
            telemetry.addData("REDAT Target X offset", coordX);
            telemetry.addData("REDAT Target Y offset", coordY);
            telemetry.addData("REDAT Coordinates",
                    String.format("[X: %.2f, Y: %.2f]", coordX, coordY));

            // Print detailed fiducial information for REDAT
            LLResultTypes.FiducialResult redatFiducial =
                    limeLight.getFiducialResultById(LimeLight.AprilTagType.REDAT);
            if (redatFiducial != null) {
                telemetry.addData("REDAT Details", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        redatFiducial.getFiducialId(), redatFiducial.getFamily(),
                        redatFiducial.getTargetXDegrees(), redatFiducial.getTargetYDegrees());
            }
        } else {
            // No REDAT AprilTag detected - searching
            telemetry.addData("Status", "Searching for REDAT AprilTag...");
            telemetry.addData("Detection Status", -1);
            telemetry.addData("REDAT Target X offset", 0.0);
            telemetry.addData("REDAT Target Y offset", 0.0);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limeLight.stop();
        super.stop();
    }
}