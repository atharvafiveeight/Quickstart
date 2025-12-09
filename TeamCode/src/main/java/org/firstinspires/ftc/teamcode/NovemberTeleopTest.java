package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * HariODTeleOP - Field-Oriented Control TeleOp Program
 *
 * This program provides smooth, field-oriented control  using the GoBilda PinPoint IMU.
 * Field-oriented control means the robot moves relative to the field, not relative to the robot's current heading.
 *
 * Key Features:
 * - Field-Oriented Control (FOC) using GoBilda PinPoint IMU
 * - Same motor setup as MainOverdriveTeleOp for consistency
 * - Anti-drift measures to prevent unwanted movement
 * - Bulk reading for faster performance
 * - Easy-to-understand code with lots of comments
 *
 * Controls:
 * - Left Stick Y: Move forward/backward (field-relative)
 * - Left Stick X: Strafe left/right (field-relative)
 * - Right Stick X: Rotate left/right
 * - Right Bumper: Launch game piece (state machine controlled)
 *
 * @author Hari
 * @version 1.0
 */
@TeleOp(name = "NovemberTeleop", group = "TeleOp")
public class NovemberTeleopTest extends LinearOpMode {

    // Motor declarations - same as MainOverdriveTeleOp for consistency
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Launcher hardware declarations
    private DcMotorEx shooterMotor;  // Shooter motor with velocity control
    private CRServo leftServo;       // Left feeder servo
    private CRServo rightServo;      // Right feeder servo

    // GoBilda PinPoint IMU - named "odo" as requested
    private GoBildaPinpointDriver odo;

    // Bulk reading setup for faster performance
    private List<LynxModule> allHubs;

    // Anti-drift constants - these help prevent unwanted movement
    private static final double JOYSTICK_DEADZONE = 0.15; // Ignore small joystick movements
    private static final double MIN_MOTOR_POWER = 0.08;   // Minimum power to overcome motor friction

    // Launcher constants - adjust these values for your robot
    private final double FEED_TIME_SECONDS = 0.08;        // How long to feed game pieces (reduced for single ball)
    private final double FEED_DELAY_SECONDS = 1.5;        // Delay between feeds for continuous shooting (1 second)
    private final double STOP_SPEED = 0.0;                // Stop speed for shooter
    private final double FULL_SPEED = 1.0;                // Full speed for servos
    private final double LAUNCHER_TARGET_VELOCITY = 1125; // Target velocity for shooter
    private final double LAUNCHER_MIN_VELOCITY = 1075;    // Minimum velocity before launching

    // Field-oriented control is always enabled in this program
    private static final boolean FIELD_CENTRIC = true;

    // Launcher state machine
    private enum LaunchState {
        IDLE,        // Shooter is stopped, waiting for launch command
        SPIN_UP,     // Shooter is spinning up to target velocity
        LAUNCH,      // Ready to launch, start feeding
        LAUNCHING,   // Currently feeding game pieces
        CONTINUOUS   // Continuous shooting mode (bumper held)
    }

    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();

    // Control variables for edge detection
    private boolean prevRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize launcher state machine
        launchState = LaunchState.IDLE;

        // ========================================
        // STEP 1: SETUP BULK READING FOR SPEED
        // ========================================
        // Bulk reading makes the robot respond faster by reading all sensors at once
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ========================================
        // STEP 2: INITIALIZE MOTORS
        // ========================================
        // Map motor names from hardware configuration to code variables
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Initialize launcher hardware
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");

            // DEBUG: Verify hardware mapping
            telemetry.addData("DEBUG", "Hardware mapping successful:");
            telemetry.addData("DEBUG", "  Shooter Motor: " + (shooterMotor != null ? "FOUND" : "MISSING"));
            telemetry.addData("DEBUG", "  Left Servo: " + (leftServo != null ? "FOUND" : "MISSING"));
            telemetry.addData("DEBUG", "  Right Servo: " + (rightServo != null ? "FOUND" : "MISSING"));
        } catch (Exception e) {
            telemetry.addData("ERROR", "Hardware mapping failed: " + e.getMessage());
        }

        // Set motor directions - same as MainOverdriveTeleOp
        // This ensures wheels spin in the correct direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set launcher motor and servo directions (FIXED)
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Changed from FORWARD to REVERSE
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);   // Changed from FORWARD to REVERSE
        leftServo.setDirection(DcMotorSimple.Direction.FORWARD);    // Changed from REVERSE to FORWARD

        // Set motors to brake when no power is applied - prevents drift
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TRYING TO NOT USE ENCODERS FOR NOW
        // Reset encoders and set to use them for smooth control
        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up shooter motor with encoder-based velocity control
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set custom PIDF coefficients for better launcher control
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        // Stop servos initially
        leftServo.setPower(0);
        rightServo.setPower(0);

        // ========================================
        // STEP 3: INITIALIZE PINPOINT IMU
        // ========================================
        // Initialize the GoBilda PinPoint IMU named "odo"
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(107.5, -98.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        // Wait for the IMU to be ready before starting
        telemetry.addData("Status", "Initializing PinPoint IMU...");
        telemetry.update();

        // Wait for IMU to be ready (check device status)
        while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY && opModeIsActive()) {
            telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
            telemetry.addData("Waiting", "Please wait for IMU to be ready...");
            telemetry.update();
            sleep(100);
        }

        // ========================================
        // STEP 4: READY TO START
        // ========================================
        telemetry.addData("Status", "HariODTeleOP Ready!");
        telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
        telemetry.addData("Control Mode", "Field-Oriented Control");
        telemetry.addData("Instructions", "Use left stick to move, right stick X to rotate");
        telemetry.addData("Launcher", "Right bumper to launch");
        telemetry.update();

        waitForStart();

        // ========================================
        // STEP 5: MAIN CONTROL LOOP
        // ========================================
        while (opModeIsActive() && !isStopRequested()) {

            // Clear cached data from control hubs for fresh readings
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // Update IMU data - this gets the latest heading information
            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

            // ========================================
            // STEP 6: PRE-WARM LAUNCHER SYSTEM
            // ========================================
            // Detect right bumper press and hold
            boolean rightBumperPressed = gamepad1.right_bumper && !prevRightBumper;
            boolean rightBumperHeld = gamepad1.right_bumper;
            boolean rightBumperReleased = !gamepad1.right_bumper && prevRightBumper;
            prevRightBumper = gamepad1.right_bumper;

            // Manual pre-warm with X button
            if (gamepad1.x) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("DEBUG", "MANUAL PRE-WARM: Setting velocity to " + LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("DEBUG", "MANUAL PRE-WARM: Current velocity: " + String.format("%.1f", shooterMotor.getVelocity()));
            } else if (!rightBumperHeld && launchState == LaunchState.IDLE) {
                // Only stop if not using bumper and in idle state
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(0);
                telemetry.addData("DEBUG", "STOPPING: Setting velocity to 0");
            }

            // Run the launch state machine
            launch(rightBumperPressed, rightBumperReleased, rightBumperHeld);

            // ========================================
            // STEP 7: GET JOYSTICK INPUT
            // ========================================
            // Read joystick values and apply deadzone to prevent drift
            double y = applyAdvancedDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x = applyAdvancedDeadzone(gamepad1.left_stick_x);  // Strafing
            double rx = applyAdvancedDeadzone(gamepad1.right_stick_x); // Rotation

            // Square inputs for smoother control while preserving direction
            // This makes small movements more precise and large movements more responsive
            y = Math.copySign(y * y, y);
            x = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);

            // ========================================
            // STEP 9: FIELD-ORIENTED CONTROL CALCULATION
            // ========================================
            // This is the magic of field-oriented control!
            // We get the robot's current heading from the IMU
            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            // Transform joystick input to field coordinates
            // This makes the robot move relative to the field, not the robot's current direction
            double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
            x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
            y = temp;

            // ========================================
            // STEP 10: MECANUM WHEEL CALCULATIONS
            // ========================================
            // Convert field-oriented movement into individual motor powers
            // This is the standard mecanum wheel formula
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator; // front left
            double blPower = (y - x + rx) / denominator; // back left
            double frPower = (y - x - rx) / denominator; // front right
            double brPower = (y + x - rx) / denominator; // back right

            // Apply minimum power threshold to prevent stalling/drift
            flPower = applyMinPower(flPower);
            blPower = applyMinPower(blPower);
            frPower = applyMinPower(frPower);
            brPower = applyMinPower(brPower);

            // ========================================
            // STEP 11: APPLY MOTOR POWERS
            // ========================================
            // Send the calculated powers to each motor
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // ========================================
            // STEP 12: UPDATE TELEMETRY
            // ========================================
            // Show important information on the driver station screen
            updateTelemetry(flPower, frPower, blPower, brPower, botHeading);
        }
    }

    /**
     * Launcher state machine - controls the shooting sequence
     * This method handles the complete launch process from spin-up to feeding
     *
     * @param shotRequested True when right bumper is pressed (edge detected)
     * @param shotReleased True when right bumper is released (edge detected)
     * @param bumperHeld True when right bumper is currently held down
     */
    void launch(boolean shotRequested, boolean shotReleased, boolean bumperHeld) {
        // DEBUG: Log state machine transitions
        LaunchState previousState = launchState;

        switch (launchState) {
            case IDLE:
                // When idle, stop the shooter and servos
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(0);    // Use power control to stop
                leftServo.setPower(0);       // Use 0 instead of STOP_SPEED for servos
                rightServo.setPower(0);      // Use 0 instead of STOP_SPEED for servos

                // DEBUG: Log idle state hardware status
                telemetry.addData("DEBUG", "IDLE: Setting shooter velocity to 0");
                telemetry.addData("DEBUG", "IDLE: Setting servos to 0");
                telemetry.addData("DEBUG", "IDLE: Current shooter velocity: " + String.format("%.1f", shooterMotor.getVelocity()));
                telemetry.addData("DEBUG", "IDLE: Current shooter power: " + String.format("%.3f", shooterMotor.getPower()));
                telemetry.addData("DEBUG", "IDLE: Left servo power: " + String.format("%.3f", leftServo.getPower()));
                telemetry.addData("DEBUG", "IDLE: Right servo power: " + String.format("%.3f", rightServo.getPower()));

                // If launch is requested, start spinning up
                if (shotRequested) {
                    telemetry.addData("DEBUG", "IDLE -> SPIN_UP: Launch requested");
                    feederTimer.reset(); // Reset timer for spin-up
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Use power control for more reliable speed control
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(0.7); // 70% power for target velocity

                // DEBUG: Log detailed hardware status
                double currentVelocity = shooterMotor.getVelocity();
                double shooterPower = shooterMotor.getPower();
                telemetry.addData("DEBUG", "SPIN_UP: Motor mode: " + shooterMotor.getMode().toString());
                telemetry.addData("DEBUG", "SPIN_UP: Setting power to 0.7");
                telemetry.addData("DEBUG", "SPIN_UP: Current velocity: " + String.format("%.1f", currentVelocity));
                telemetry.addData("DEBUG", "SPIN_UP: Shooter power: " + String.format("%.3f", shooterPower));
                telemetry.addData("DEBUG", "SPIN_UP: Target velocity: " + LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("DEBUG", "SPIN_UP: Min velocity: " + LAUNCHER_MIN_VELOCITY);

                // Wait a fixed time for motor to spin up (more reliable than velocity checking)
                if (feederTimer.seconds() > 2.0) { // Wait 2 seconds for spin-up
                    telemetry.addData("DEBUG", "SPIN_UP -> LAUNCH: Spin-up time complete");
                    launchState = LaunchState.LAUNCH;
                } else {
                    telemetry.addData("DEBUG", "SPIN_UP: Spinning up... " + String.format("%.1f", feederTimer.seconds()) + " / 2.0 seconds");
                }
                break;

            case LAUNCH:
                // Start feeding the game pieces
                telemetry.addData("DEBUG", "LAUNCH -> LAUNCHING: Starting feeder servos");
                telemetry.addData("DEBUG", "LAUNCH: Setting left servo to " + FULL_SPEED);
                telemetry.addData("DEBUG", "LAUNCH: Setting right servo to " + FULL_SPEED);

                leftServo.setPower(FULL_SPEED);
                rightServo.setPower(FULL_SPEED);

                // DEBUG: Check if servos are actually responding
                telemetry.addData("DEBUG", "LAUNCH: Left servo power reading: " + String.format("%.3f", leftServo.getPower()));
                telemetry.addData("DEBUG", "LAUNCH: Right servo power reading: " + String.format("%.3f", rightServo.getPower()));

                feederTimer.reset(); // Start timing the feed
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // Continue feeding for the specified time
                double feedTime = feederTimer.seconds();
                telemetry.addData("DEBUG", "LAUNCHING: Feed time: " + String.format("%.2f", feedTime) +
                        " / Target: " + FEED_TIME_SECONDS);
                telemetry.addData("DEBUG", "LAUNCHING: Left servo power: " + String.format("%.3f", leftServo.getPower()));
                telemetry.addData("DEBUG", "LAUNCHING: Right servo power: " + String.format("%.3f", rightServo.getPower()));

                if (feedTime > FEED_TIME_SECONDS) {
                    // Feeding complete, check if bumper is still held
                    if (bumperHeld) {
                        // Go to continuous mode for repeated shooting
                        telemetry.addData("DEBUG", "LAUNCHING -> CONTINUOUS: Starting continuous mode");
                        launchState = LaunchState.CONTINUOUS;
                        leftServo.setPower(0);   // Stop servos
                        rightServo.setPower(0);  // Stop servos
                        feederTimer.reset();     // Reset timer for next feed
                    } else {
                        // Return to idle
                        telemetry.addData("DEBUG", "LAUNCHING -> IDLE: Feed complete, bumper released");
                        launchState = LaunchState.IDLE;
                        leftServo.setPower(0);   // Use 0 instead of STOP_SPEED
                        rightServo.setPower(0);  // Use 0 instead of STOP_SPEED
                    }
                }
                break;

            case CONTINUOUS:
                // Continuous shooting mode - feed balls at intervals
                double continuousTime = feederTimer.seconds();
                telemetry.addData("DEBUG", "CONTINUOUS: Time since last feed: " + String.format("%.2f", continuousTime));
                telemetry.addData("DEBUG", "CONTINUOUS: Bumper held: " + bumperHeld);

                if (bumperHeld) {
                    // Check if it's time for next feed
                    if (continuousTime > FEED_DELAY_SECONDS) {
                        // Time to feed another ball
                        telemetry.addData("DEBUG", "CONTINUOUS -> LAUNCHING: Feeding next ball");
                        leftServo.setPower(FULL_SPEED);
                        rightServo.setPower(FULL_SPEED);
                        feederTimer.reset(); // Start timing the feed
                        launchState = LaunchState.LAUNCHING;
                    }
                } else {
                    // Bumper released, return to idle
                    telemetry.addData("DEBUG", "CONTINUOUS -> IDLE: Bumper released");
                    launchState = LaunchState.IDLE;
                    leftServo.setPower(0);
                    rightServo.setPower(0);
                }
                break;
        }

        // DEBUG: Log state changes
        if (previousState != launchState) {
            telemetry.addData("DEBUG", "State changed: " + previousState.toString() + " -> " + launchState.toString());
        }
    }

    /**
     * Advanced deadzone function that scales remaining input
     * This prevents small joystick movements from causing unwanted robot movement
     *
     * @param value The raw joystick input value
     * @return The processed joystick value with deadzone applied
     */
    private double applyAdvancedDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0.0; // Ignore small movements
        }
        // Scale the remaining input to maintain full range
        return Math.signum(value) * ((Math.abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE));
    }

    /**
     * Apply minimum power threshold to prevent motor stalling
     * This ensures motors have enough power to overcome friction
     *
     * @param power The calculated motor power
     * @return The adjusted motor power with minimum threshold applied
     */
    private double applyMinPower(double power) {
        if (Math.abs(power) > 0 && Math.abs(power) < MIN_MOTOR_POWER) {
            return Math.signum(power) * MIN_MOTOR_POWER;
        }
        return power;
    }

    /**
     * Update telemetry with important robot information
     * This helps the driver understand what the robot is doing
     *
     * @param flPower Front left motor power
     * @param frPower Front right motor power
     * @param blPower Back left motor power
     * @param brPower Back right motor power
     * @param botHeading Robot's current heading in radians
     */
    private void updateTelemetry(double flPower, double frPower, double blPower, double brPower, double botHeading) {
        telemetry.clear();

        // Header information
        telemetry.addLine("=== HARI OD TELEOP ===");
        telemetry.addData("Control Mode", "Field-Oriented Control");
        telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
        telemetry.addLine();

        // Robot heading information
        telemetry.addLine("--- ROBOT STATUS ---");
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(botHeading)));
        telemetry.addData("Heading (rad)", String.format("%.3f", botHeading));
        telemetry.addLine();

        // Motor power information
        telemetry.addLine("--- MOTOR POWERS ---");
        telemetry.addData("Front Left", String.format("%.3f", flPower));
        telemetry.addData("Front Right", String.format("%.3f", frPower));
        telemetry.addData("Back Left", String.format("%.3f", blPower));
        telemetry.addData("Back Right", String.format("%.3f", brPower));
        telemetry.addLine();

        // Launcher information
        telemetry.addLine("--- LAUNCHER STATUS ---");
        telemetry.addData("Launch State", launchState.toString());
        telemetry.addData("Shooter Velocity", String.format("%.1f", shooterMotor.getVelocity()));
        telemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Min Velocity", LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Right Bumper", gamepad1.right_bumper ? "PRESSED" : "RELEASED");
        telemetry.addData("Shooter Power", String.format("%.3f", shooterMotor.getPower()));
        telemetry.addData("Left Servo Power", String.format("%.3f", leftServo.getPower()));
        telemetry.addData("Right Servo Power", String.format("%.3f", rightServo.getPower()));
        telemetry.addData("Motor Mode", shooterMotor.getMode().toString());
        telemetry.addData("Current Position", shooterMotor.getCurrentPosition());
        telemetry.addLine();

        // Control instructions
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Left Stick Y", "Move Forward/Backward");
        telemetry.addData("Left Stick X", "Strafe Left/Right");
        telemetry.addData("Right Stick X", "Rotate Left/Right");
        telemetry.addData("Right Bumper", "Launch Game Piece");
        telemetry.addLine();

        // Performance information
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("IMU Frequency", String.format("%.1f Hz", odo.getFrequency()));
        telemetry.addData("Loop Time", odo.getLoopTime() + " μs");
        telemetry.addData("Deadzone", JOYSTICK_DEADZONE);
        telemetry.addLine();

        // Debug information section
        telemetry.addLine("--- DEBUG INFO ---");
        // Debug messages are already added above in the main loop

        telemetry.update();
    }
}
