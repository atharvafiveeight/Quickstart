package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Drive with Limelight", group = "Autonomous")
public class AprilTagDriveExample extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Limelight and Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Drive constants
    private static final double DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;
    private static final double TARGET_DISTANCE = 12.0; // inches (1 foot)
    private static final double DISTANCE_TOLERANCE = 2.0; // inches
    private static final double HEADING_TOLERANCE = 2.0; // degrees

    // PID constants for smooth control
    private static final double KP_DISTANCE = 0.02;
    private static final double KP_HEADING = 0.01;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware
        initializeHardware();

        // Initialize vision
        initializeVision();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Look for AprilTag and drive to it");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Get AprilTag detections
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.size() > 0) {
                // Process the first detected AprilTag
                AprilTagDetection detection = currentDetections.get(0);
                driveToAprilTag(detection);
            } else {
                // No AprilTag detected - stop and search
                stopAllMotors();
                telemetry.addData("Status", "Searching for AprilTag...");
            }

            // Display telemetry
            updateTelemetry(currentDetections);

            // Safety timeout
            if (runtime.seconds() > 30) {
                telemetry.addData("Status", "Timeout reached");
                break;
            }
        }

        // Stop all motors when done
        stopAllMotors();
    }

    private void initializeHardware() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions (adjust based on your robot configuration)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeVision() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create the vision portal using the Limelight camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Change to your Limelight camera name
                .addProcessor(aprilTag)
                .build();
    }

    private void driveToAprilTag(AprilTagDetection detection) {
        // Get position data from the AprilTag detection
        double range = detection.ftcPose.range; // Distance to tag
        double bearing = detection.ftcPose.bearing; // Angle to tag (-180 to +180)
        double yaw = detection.ftcPose.yaw; // Tag rotation

        // Calculate distance error (how far we are from target distance)
        double distanceError = range - TARGET_DISTANCE;

        // Calculate heading error (how much we need to turn)
        double headingError = bearing;

        // Check if we're close enough to the target
        if (Math.abs(distanceError) < DISTANCE_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE) {

            // We're at the target position - stop
            stopAllMotors();
            telemetry.addData("Status", "Target Reached!");
            return;
        }

        // Calculate drive powers using simple proportional control
        double drivePower = distanceError * KP_DISTANCE;
        double turnPower = headingError * KP_HEADING;

        // Limit drive power
        drivePower = Math.max(-DRIVE_SPEED, Math.min(DRIVE_SPEED, drivePower));
        turnPower = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turnPower));

        // Calculate individual motor powers for tank drive with turning
        double leftPower = drivePower - turnPower;
        double rightPower = drivePower + turnPower;

        // Apply motor powers
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void updateTelemetry(List<AprilTagDetection> detections) {
        telemetry.addData("# AprilTags Detected", detections.size());

        if (detections.size() > 0) {
            AprilTagDetection detection = detections.get(0);
            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("Range", "%.2f inches", detection.ftcPose.range);
            telemetry.addData("Bearing", "%.1f degrees", detection.ftcPose.bearing);
            telemetry.addData("Yaw", "%.1f degrees", detection.ftcPose.yaw);

            double distanceError = detection.ftcPose.range - TARGET_DISTANCE;
            telemetry.addData("Distance Error", "%.2f inches", distanceError);

            if (Math.abs(distanceError) < DISTANCE_TOLERANCE &&
                    Math.abs(detection.ftcPose.bearing) < HEADING_TOLERANCE) {
                telemetry.addData("Status", "AT TARGET!");
            } else {
                telemetry.addData("Status", "Approaching target...");
            }
        }

        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.update();
    }
}