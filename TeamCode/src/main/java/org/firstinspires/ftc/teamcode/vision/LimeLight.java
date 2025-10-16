package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * LimeLight class to handle AprilTag detection and coordinate retrieval
 * Includes robot pose estimation relative to AprilTags for PedroPathing integration
 */
public class LimeLight {

    /**
     * ENUM for different AprilTag families with their IDs
     */
    public enum AprilTagType {
        BLUEAT(20),
        PGP(21),
        GPG(22),
        PPG(23),
        REDAT(24);

        private final int tagId;

        AprilTagType(int tagId) {
            this.tagId = tagId;
        }

        public int getTagId() {
            return tagId;
        }
    }

    private Limelight3A limelight3A;
    private AprilTagType currentTagType;
    private int pipelineId;

    // Robot dimensions - adjust based on your robot's measurements
    private static final double ROBOT_LENGTH_INCHES = 18.0;  // Front to back
    private static final double ROBOT_WIDTH_INCHES = 18.0;   // Left to right
    private static final double LIMELIGHT_OFFSET_X = 0.0;    // X offset from robot center (inches)
    private static final double LIMELIGHT_OFFSET_Y = 0.0;    // Y offset from robot center (inches)
    private static final double LIMELIGHT_HEIGHT = 6.0;      // Height from ground (inches)

    // Field reference points - AprilTag positions (in inches, adjust for your field)
    private static final double TAG_HEIGHT = 5.5;             // Height of AprilTag center (inches)

    /**
     * Initialize LimeLight with hardware map and pipeline ID
     */
    public LimeLight(HardwareMap hardwareMap, String deviceName, int pipelineId) {
        limelight3A = hardwareMap.get(Limelight3A.class, deviceName);
        this.pipelineId = pipelineId;
        limelight3A.pipelineSwitch(pipelineId);
        limelight3A.start();
    }

    /**
     * Set pipeline ID
     */
    public void setPipelineId(int newPipelineId) {
        this.pipelineId = newPipelineId;
        limelight3A.pipelineSwitch(newPipelineId);
    }

    /**
     * Get current pipeline ID
     */
    public int getPipelineId() {
        return pipelineId;
    }

    /**
     * Get coordinates as an array [x, y, area]
     */
    public double[] getCoordinates(AprilTagType aTag) {
        LLResultTypes.FiducialResult result = getFiducialResultById(aTag);
        if (result != null) {
            return new double[]{result.getTargetXDegrees(),
                    result.getTargetYDegrees(), result.getTargetArea()};
        }
        return new double[]{0.0, 0.0, 0.0};
    }

    /**
     * Calculate distance from robot to AprilTag in inches
     * Uses the 3D distance from Limelight to tag
     */
    public double getDistanceToAprilTag(AprilTagType aTag) {
        LLResultTypes.FiducialResult result = getFiducialResultById(aTag);
        if (result != null) {
            double tx = result.getTargetXDegrees();
            double ty = result.getTargetYDegrees();
            double area = result.getTargetArea();

            // Estimate distance using area (calibration dependent)
            // This is a rough approximation - adjust coefficients based on your calibration
            double distance = Math.sqrt(25000.0 / area);  // Rough estimate

            return distance;
        }
        return -1.0;  // Tag not detected
    }

    /**
     * Get robot coordinates relative to a detected AprilTag
     * Returns coordinates suitable for PedroPathing [x, y, theta]
     *
     * @param aTag The AprilTag to use as reference
     * @return Array [robotX, robotY, robotTheta] in inches and degrees, or null if tag not detected
     */
    public double[] getRobotCoordinatesRelativeToAprilTag(AprilTagType aTag) {
        LLResultTypes.FiducialResult result = getFiducialResultById(aTag);

        if (result == null) {
            return null;  // Tag not detected
        }

        // Get target offset angles from Limelight
        double tx = result.getTargetXDegrees();      // Horizontal offset
        double ty = result.getTargetYDegrees();      // Vertical offset
        double targetArea = result.getTargetArea();

        // Estimate distance to tag using area (requires calibration)
        double distanceToTag = estimateDistanceFromArea(targetArea);

        if (distanceToTag <= 0) {
            return null;  // Invalid distance
        }

        // Convert angle offsets to horizontal and vertical distances
        // Using small angle approximation or trigonometry
        double horizontalDistanceFromCenter = distanceToTag * Math.tan(Math.toRadians(tx));
        double verticalDistanceFromCenter = distanceToTag * Math.tan(Math.toRadians(ty));

        // Account for Limelight mounting offset from robot center
        double robotToTagX = horizontalDistanceFromCenter - LIMELIGHT_OFFSET_X;
        double robotToTagY = verticalDistanceFromCenter - LIMELIGHT_OFFSET_Y;

        // Calculate robot position relative to AprilTag
        // AprilTag serves as origin for this calculation
        double robotX = -robotToTagX;  // Negate based on coordinate system
        double robotY = robotToTagY;

        // Estimate robot heading based on target position
        // If tag is centered (tx = 0), robot is roughly aligned
        double robotTheta = tx;  // Raw heading offset from tag

        return new double[]{robotX, robotY, robotTheta};
    }

    /**
     * Estimate distance from Limelight to AprilTag based on detected area
     * This requires calibration specific to your setup and lighting conditions
     *
     * @param area The target area detected by Limelight (0-100)
     * @return Distance in inches, or -1 if invalid
     */
    private double estimateDistanceFromArea(double area) {
        // Calibration values - adjust these based on your testing
        // Formula: distance = k / sqrt(area)
        // k should be calibrated by measuring distance at known areas

        if (area <= 0) {
            return -1;
        }

        final double CALIBRATION_CONSTANT = 500.0;  // Adjust based on your calibration
        double distance = CALIBRATION_CONSTANT / Math.sqrt(area);

        return distance;
    }

    /**
     * Get all fiducial results (AprilTag detections)
     */
    public List<LLResultTypes.FiducialResult> getAllFiducialResults() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            return result.getFiducialResults();
        }
        return new ArrayList<>();
    }

    /**
     * Get fiducial result by specific ID
     */
    public LLResultTypes.FiducialResult getFiducialResultById(AprilTagType aprilTagType) {
        List<LLResultTypes.FiducialResult> results = getAllFiducialResults();
        for (LLResultTypes.FiducialResult fr : results) {
            if (fr.getFiducialId() == aprilTagType.getTagId()) {
                return fr;
            }
        }
        return null;
    }

    /**
     * Check if a valid target is detected
     */
    public boolean isTargetDetected() {
        LLResult result = limelight3A.getLatestResult();
        return result != null && result.isValid();
    }

    /**
     * Check if any fiducials are detected
     */
    public boolean hasFiducials() {
        return !getAllFiducialResults().isEmpty();
    }

    /**
     * Stop the limelight
     */
    public void stop() {
        if (limelight3A != null) {
            limelight3A.stop();
        }
    }

    /**
     * Get the latest result for advanced operations
     */
    public LLResult getLatestResult() {
        return limelight3A.getLatestResult();
    }
}