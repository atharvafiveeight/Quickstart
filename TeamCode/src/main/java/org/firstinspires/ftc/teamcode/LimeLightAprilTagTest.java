package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vision.LimeLight;

/**
 * Test OpMode for LimeLight AprilTag detection and navigation
 * Press GamePad1 Button A to start autonomous navigation to detected AprilTag
 * Robot will drive towards the tag and stop 2 feet (24 inches) away
 */
@TeleOp(name = "LimeLight AprilTag Navigation Test", group = "Vision")
public class LimeLightAprilTagTest extends LinearOpMode {

    // Hardware
    private LimeLight limeLight;
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Navigation parameters
    private static final double STOP_DISTANCE_INCHES = 24.0;  // 2 feet
    private static final double TARGET_TOLERANCE_INCHES = 2.0; // Stop within 2 inches
    private static final double MAX_SPEED = 0.8;
    private static final double MIN_SPEED = 0.2;
    private static final double KP = 0.02;  // Proportional gain for distance control
    private static final double MAX_TIMEOUT_SECONDS = 10.0;   // Max time to reach target

    private boolean isNavigating = false;
    private ElapsedTime navigationTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeMotors();

        // Initialize LimeLight with pipeline 0 for AprilTags
        limeLight = new LimeLight(hardwareMap, "Limelight", 0);

        navigationTimer = new ElapsedTime();

        telemetry.addLine("LimeLight AprilTag Navigation Test");
        telemetry.addLine("Press GamePad1 Button A to start navigation");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check for Button A press to start navigation
            if (gamepad1.a && !isNavigating) {
                isNavigating = true;
                navigationTimer.reset();
                telemetry.addLine("Starting navigation to AprilTag...");
                telemetry.update();
            }

            // Reset navigation flag if Button A is released
            if (!gamepad1.a) {
                isNavigating = false;
                stopRobot();
            }

            // Execute navigation if active
            if (isNavigating) {
                navigateToAprilTag();
            } else {
                // Manual control when not navigating
                manualDrive();
            }

            updateTelemetry();
            sleep(20);  // 50 Hz loop
        }

        // Cleanup
        limeLight.stop();
        stopRobot();
    }

    /**
     * Navigate robot towards detected AprilTag
     */
    private void navigateToAprilTag() {
        // Try to detect target tag (using REDAT as example - adjust as needed)
        LimeLight.AprilTagType targetTag = LimeLight.AprilTagType.REDAT;

        // Get robot coordinates relative to AprilTag
        double[] robotCoords = limeLight.getRobotCoordinatesRelativeToAprilTag(targetTag);

        if (robotCoords == null) {
            telemetry.addLine("ERROR: AprilTag not detected!");
            stopRobot();
            return;
        }

        double robotX = robotCoords[0];
        double robotY = robotCoords[1];
        double robotTheta = robotCoords[2];

        // Calculate distance to tag
        double distanceToTag = limeLight.getDistanceToAprilTag(targetTag);

        if (distanceToTag < 0) {
            telemetry.addLine("ERROR: Invalid distance calculation");
            stopRobot();
            return;
        }

        telemetry.addData("Distance to Tag", "%.2f inches", distanceToTag);
        telemetry.addData("Robot Position", "X: %.2f, Y: %.2f", robotX, robotY);
        telemetry.addData("Robot Heading", "%.2f degrees", robotTheta);

        // Check if we've reached target distance
        double remainingDistance = distanceToTag - STOP_DISTANCE_INCHES;

        if (remainingDistance <= TARGET_TOLERANCE_INCHES) {
            telemetry.addLine("TARGET REACHED!");
            stopRobot();
            isNavigating = false;
            return;
        }

        // Check timeout
        if (navigationTimer.seconds() > MAX_TIMEOUT_SECONDS) {
            telemetry.addLine("TIMEOUT: Navigation took too long");
            stopRobot();
            isNavigating = false;
            return;
        }

        // Calculate speed based on remaining distance
        double speed = Math.max(MIN_SPEED, remainingDistance * KP);
        speed = Math.min(speed, MAX_SPEED);

        // Correct heading if needed
        double headingError = robotTheta;  // Error from straight ahead

        if (Math.abs(headingError) > 5.0) {
            // Strafe to correct heading
            double strafeSpeed = headingError * 0.01;
            strafeSpeed = Math.max(-0.5, Math.min(0.5, strafeSpeed));
            moveRobotStrafe(speed, strafeSpeed);
        } else {
            // Drive forward towards tag
            moveRobotForward(speed);
        }

        telemetry.addData("Speed", "%.2f", speed);
        telemetry.addData("Remaining Distance", "%.2f inches", remainingDistance);
        telemetry.addData("Navigation Time", "%.2f seconds", navigationTimer.seconds());
    }

    /**
     * Manual drive control using gamepad left and right sticks
     */
    private void manualDrive() {
        double forward = -gamepad1.left_stick_y;   // Forward/backward
        double strafe = gamepad1.left_stick_x;      // Strafe left/right
        double rotate = gamepad1.right_stick_x;     // Rotate

        moveRobot(forward, strafe, rotate);
    }

    /**
     * Move robot forward
     */
    private void moveRobotForward(double speed) {
        moveRobot(speed, 0, 0);
    }

    /**
     * Move robot with strafe correction
     */
    private void moveRobotStrafe(double forward, double strafe) {
        moveRobot(forward, strafe, 0);
    }

    /**
     * Universal robot movement using mecanum drive
     */
    private void moveRobot(double forward, double strafe, double rotate) {
        // Mecanum drive calculations
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize if any value exceeds 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Stop all motors
     */
    private void stopRobot() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Initialize drive motors
     * Adjust names to match your robot configuration
     */
    private void initializeMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions (adjust if needed for your robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.addLine("=== LimeLight AprilTag Test ===");
        telemetry.addLine();

        if (limeLight.hasFiducials()) {
            telemetry.addLine("Status: AprilTag DETECTED");
            double[] coords = limeLight.getCoordinates(LimeLight.AprilTagType.REDAT);
            telemetry.addData("Tag X Degrees", "%.2f", coords[0]);
            telemetry.addData("Tag Y Degrees", "%.2f", coords[1]);
            telemetry.addData("Tag Area", "%.2f", coords[2]);
        } else {
            telemetry.addLine("Status: No AprilTag detected");
        }

        telemetry.addLine();
        telemetry.addData("Navigation Active", isNavigating);
        telemetry.addLine("Press GamePad1 Button A to navigate");
        telemetry.update();
    }
}