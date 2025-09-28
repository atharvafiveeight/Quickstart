package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot Teleop Template", group="Linear Opmode")
public class RobotTeleopTemplate extends LinearOpMode {

    // Runtime timer
    private ElapsedTime runtime = new ElapsedTime();

    // Drive train motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Additional motors
    private DcMotor shooterMotor1 = null;
    private DcMotor shooterMotor2 = null;

    // Servos
    private Servo pushServo1 = null;
    private Servo pushServo2 = null;

    // Servo positions
    private double pushServo1Position = 0.5;
    private double pushServo2Position = 0.5;

    // Drive power variables
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;

    // Limelight variables
    private double limelightTx = 0; // Horizontal offset
    private double limelightTy = 0; // Vertical offset
    private double limelightTa = 0; // Target area
    private boolean limelightHasTarget = false;

    // Button press tracking for toggle functionality
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousX = false;
    private boolean previousY = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        telemetry.addData("Status", "Initialized - Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Update Limelight data
            updateLimelightData();

            // Handle drivetrain movement
            handleDriveMovement();

            // Handle gamepad button logic with switch
            handleGamepadButtons();

            // Handle servo controls
            handleServoControls();

            // Handle shooter motor controls
            handleShooterMotors();

            // Handle Limelight interactions
            handleLimelightControls();

            // Apply motor powers
            applyMotorPowers();

            // Update servo positions
            updateServoPositions();

            // Display telemetry
            displayTelemetry();

            // Small delay to prevent excessive CPU usage
            sleep(10);
        }

        // Stop all hardware when opmode ends
        stopAllHardware();
    }

    // ==================== INITIALIZATION METHODS ====================

    private void initializeHardware() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        // Initialize drive train motors
        initializeDriveMotors();

        // Initialize shooter motors
        initializeShooterMotors();

        // Initialize servos
        initializeServos();

        // Initialize Limelight
        initializeLimelight();

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();
    }

    private void initializeDriveMotors() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions (adjust based on your robot configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeShooterMotors() {
        // Initialize shooter motors
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooter_motor_1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter_motor_2");

        // Set motor directions (adjust based on your robot configuration)
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeServos() {
        // Initialize push servos (adjust names based on your hardware configuration)
        pushServo1 = hardwareMap.get(Servo.class, "push_servo_1");
        pushServo2 = hardwareMap.get(Servo.class, "push_servo_2");

        // Set initial servo positions
        pushServo1.setPosition(pushServo1Position);
        pushServo2.setPosition(pushServo2Position);
    }

    private void initializeLimelight() {
        // TODO: Initialize Limelight connection and settings
        // This depends on your specific Limelight integration method
        // Common initialization tasks:
        // - Set pipeline
        // - Configure LED mode
        // - Set camera mode

        // Stub for Limelight initialization
        setLimelightPipeline(0);
        setLimelightLEDMode(3); // Force on
        setLimelightCameraMode(0); // Vision processor
    }

    // ==================== GAMEPAD BUTTON LOGIC ====================

    private void handleGamepadButtons() {
        // Handle single gamepad button presses with switch logic and edge detection

        // Handle button presses (with edge detection for toggles)
        if (gamepad1.a) handleButtonAction("a");
        if (gamepad1.b) handleButtonAction("b");

        // Toggle buttons - only trigger on press, not hold
        if (gamepad1.x && !previousX) handleButtonAction("x");
        if (gamepad1.y && !previousY) handleButtonAction("y");

        if (gamepad1.dpad_up) handleButtonAction("dpad_up");
        if (gamepad1.dpad_down) handleButtonAction("dpad_down");
        if (gamepad1.dpad_left) handleButtonAction("dpad_left");
        if (gamepad1.dpad_right) handleButtonAction("dpad_right");

        // Handle bumpers (continuous action while held)
        if (gamepad1.left_bumper && !previousLeftBumper) handleButtonAction("left_bumper");
        if (gamepad1.right_bumper) handleButtonAction("right_bumper");

        // Update previous button states for edge detection
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
    }

    private void handleButtonAction(String buttonId) {
        switch (buttonId) {
            case "a":
                activatePushServo1();
                break;
            case "b":
                activatePushServo2();
                break;
            case "x":
                toggleLimelightLED();
                break;
            case "y":
                if (limelightHasTarget) {
                    performAutoAlign();
                }
                break;
            case "dpad_up":
                adjustPushServo1Position(0.1);
                break;
            case "dpad_down":
                adjustPushServo1Position(-0.1);
                break;
            case "dpad_left":
                setLimelightPipeline(0);
                break;
            case "dpad_right":
                setLimelightPipeline(1);
                break;
            case "left_bumper":
                // Reset servos to neutral
                resetPushServos();
                break;
            case "right_bumper":
                // Control both servos and shooter motors
                activateServosAndShooters();
                break;
            default:
                // No action for unhandled buttons
                break;
        }
    }

    // ==================== DRIVE MOVEMENT METHODS ====================

    private void handleDriveMovement() {
        // Get gamepad inputs - using gamepad1 joysticks for movement
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;   // Left/right
        double turn = gamepad1.right_stick_x;    // Rotation

        // Apply drive modes (precision mode when left bumper is held)
        if (gamepad1.left_bumper) {
            // Precision mode - reduced speed
            handlePrecisionDrive(drive * 0.3, strafe * 0.3, turn * 0.3);
        } else {
            // Normal mecanum drive
            handleMecanumDrive(drive, strafe, turn);
        }
    }

    private void handleMecanumDrive(double drive, double strafe, double turn) {
        // Calculate wheel powers for mecanum drive
        leftFrontPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        leftBackPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        rightBackPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
    }

    private void handlePrecisionDrive(double drive, double strafe, double turn) {
        // Precision drive with reduced sensitivity
        handleMecanumDrive(drive, strafe, turn);
    }

    private void handleTankDrive(double leftPower, double rightPower) {
        // Tank drive mode (alternative to mecanum)
        leftFrontPower = leftPower;
        leftBackPower = leftPower;
        rightFrontPower = rightPower;
        rightBackPower = rightPower;
    }

    private void applyMotorPowers() {
        // Apply calculated powers to motors
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void stopAllMotors() {
        // Emergency stop - set all motor powers to zero
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Stop shooter motors
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    // ==================== SHOOTER MOTOR METHODS ====================

    private void handleShooterMotors() {
        // Shooter motor control integrated with right bumper (handled in button logic)
        // Stop motors when right bumper is not pressed
        if (!gamepad1.right_bumper) {
            stopShooterMotors();
        }
    }

    private void activateShooterMotors() {
        // Activate shooter motors at full speed
        shooterMotor1.setPower(1.0);
        shooterMotor2.setPower(1.0);
    }

    private void setShooterMotorsPower(double power) {
        // Set shooter motors to specific power level
        power = Range.clip(power, -1.0, 1.0);
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    private void stopShooterMotors() {
        // Stop shooter motors
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    // ==================== SERVO CONTROL METHODS ====================

    private void handleServoControls() {
        // Push servo control (handled primarily in button logic)
        // Additional continuous control can be added here if needed

        // Fine control with gamepad1 triggers for servo 2
        if (gamepad1.right_trigger > 0.1) {
            adjustPushServo2Position(gamepad1.right_trigger * 0.02);
        } else if (gamepad1.left_trigger > 0.1) {
            adjustPushServo2Position(-gamepad1.left_trigger * 0.02);
        }
    }

    private void activateServosAndShooters() {
        // Right bumper activates both servos and shooter motors
        activatePushServo1();
        activatePushServo2();
        activateShooterMotors();
    }

    private void activatePushServo1() {
        // Activate push servo 1 (extend)
        pushServo1Position = 1.0;
    }

    private void activatePushServo2() {
        // Activate push servo 2 (extend)
        pushServo2Position = 1.0;
    }

    private void retractPushServo1() {
        // Retract push servo 1
        pushServo1Position = 0.0;
    }

    private void retractPushServo2() {
        // Retract push servo 2
        pushServo2Position = 0.0;
    }

    private void resetPushServos() {
        // Reset both push servos to neutral position
        pushServo1Position = 0.5;
        pushServo2Position = 0.5;
    }

    private void adjustPushServo1Position(double delta) {
        pushServo1Position = Range.clip(pushServo1Position + delta, 0.0, 1.0);
    }

    private void adjustPushServo2Position(double delta) {
        pushServo2Position = Range.clip(pushServo2Position + delta, 0.0, 1.0);
    }

    private void setPushServo1Position(double position) {
        pushServo1Position = Range.clip(position, 0.0, 1.0);
    }

    private void setPushServo2Position(double position) {
        pushServo2Position = Range.clip(position, 0.0, 1.0);
    }

    private void updateServoPositions() {
        // Apply servo positions
        pushServo1.setPosition(pushServo1Position);
        pushServo2.setPosition(pushServo2Position);
    }

    private void resetServos() {
        // Reset servos to default positions
        pushServo1Position = 0.5;
        pushServo2Position = 0.5;
        updateServoPositions();
    }

    // ==================== LIMELIGHT METHODS ====================

    private void handleLimelightControls() {
        // Limelight controls moved to button logic switch statement
        // Additional continuous Limelight control can be added here if needed
    }

    private void updateLimelightData() {
        // TODO: Implement Limelight data retrieval
        // This depends on your specific Limelight integration
        // Common data to retrieve:
        // - tx (horizontal offset)
        // - ty (vertical offset)
        // - ta (target area)
        // - tv (valid target)

        // Stub implementation
        limelightTx = getLimelightTx();
        limelightTy = getLimelightTy();
        limelightTa = getLimelightTa();
        limelightHasTarget = getLimelightHasTarget();
    }

    private void performAutoAlign() {
        // Auto-align robot with Limelight target
        double steeringAdjust = limelightTx * 0.05; // Proportional steering
        double distanceAdjust = (limelightTa < 5.0) ? 0.3 : 0.0; // Move closer if target is small

        // Apply auto-align movement
        handleMecanumDrive(distanceAdjust, 0, -steeringAdjust);

        // Apply the calculated movement immediately
        applyMotorPowers();

        // Brief pause for alignment movement
        sleep(100);
    }

    private void performDistanceControl() {
        // Control robot distance based on target area
        double targetArea = 10.0; // Desired target area
        double driveAdjust = (targetArea - limelightTa) * 0.02;

        handleMecanumDrive(driveAdjust, 0, 0);
    }

    // ==================== LIMELIGHT STUB METHODS ====================
    // TODO: Implement these methods based on your Limelight integration

    private void setLimelightPipeline(int pipeline) {
        // TODO: Set Limelight pipeline
        // Implementation depends on your Limelight integration method
        telemetry.addData("Limelight", "Setting pipeline to %d", pipeline);
    }

    private void setLimelightLEDMode(int mode) {
        // TODO: Set Limelight LED mode
        // 0 - use pipeline default
        // 1 - force off
        // 2 - force blink
        // 3 - force on
        telemetry.addData("Limelight", "Setting LED mode to %d", mode);
    }

    private void setLimelightCameraMode(int mode) {
        // TODO: Set Limelight camera mode
        // 0 - vision processor
        // 1 - driver camera (increases exposure, disables vision processing)
        telemetry.addData("Limelight", "Setting camera mode to %d", mode);
    }

    private void toggleLimelightLED() {
        // TODO: Toggle Limelight LED on/off
        telemetry.addData("Action", "Toggling Limelight LED");
    }

    private double getLimelightTx() {
        // TODO: Get horizontal offset from Limelight
        // Return value in degrees
        return 0.0; // Stub return
    }

    private double getLimelightTy() {
        // TODO: Get vertical offset from Limelight
        // Return value in degrees
        return 0.0; // Stub return
    }

    private double getLimelightTa() {
        // TODO: Get target area from Limelight
        // Return value as percentage of image (0-100)
        return 0.0; // Stub return
    }

    private boolean getLimelightHasTarget() {
        // TODO: Check if Limelight has valid target
        // Return true if target is detected
        return false; // Stub return
    }

    // ==================== UTILITY METHODS ====================

    private void stopAllHardware() {
        // Stop all motors and reset servos when opmode ends
        stopAllMotors();
        resetServos();
        telemetry.addData("Status", "All hardware stopped");
        telemetry.update();
    }

    // ==================== TELEMETRY METHODS ====================

    private void displayTelemetry() {
        // Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("OpMode", "Active: %s", opModeIsActive());

        // Drive telemetry
        telemetry.addData("Drive", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        // Shooter motor telemetry
        telemetry.addData("Shooter Motors", "Motor1: %.2f, Motor2: %.2f",
                shooterMotor1.getPower(), shooterMotor2.getPower());

        // Servo telemetry
        telemetry.addData("Push Servos", "Servo1 (%.2f), Servo2 (%.2f)",
                pushServo1Position, pushServo2Position);

        // Limelight telemetry
        telemetry.addData("Limelight", "Target: %s", limelightHasTarget ? "YES" : "NO");
        if (limelightHasTarget) {
            telemetry.addData("Limelight Data", "Tx: %.2f, Ty: %.2f, Ta: %.2f",
                    limelightTx, limelightTy, limelightTa);
        }

        // Control instructions
        telemetry.addData("Drive Controls", "Left stick: drive/strafe, Right stick: turn");
        telemetry.addData("Right Bumper", "Activate servos AND shooter motors");
        telemetry.addData("Push Servos", "A: servo1, B: servo2, Left bumper: reset");
        telemetry.addData("Servo Adjust", "Dpad up/down: servo1, Triggers: servo2");
        telemetry.addData("Limelight", "X: LED toggle, Y: auto-align, Dpad L/R: pipeline");

        telemetry.update();
    }
}