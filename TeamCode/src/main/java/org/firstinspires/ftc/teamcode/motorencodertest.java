package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * motorencodertest - Test program for shooter motor encoder and velocity control
 * 
 * This program allows you to test your shooter motor at different speeds and durations
 * to verify encoder functionality and find optimal settings.
 * 
 * Controls:
 * - A Button: Test at 1000 RPM for 3 seconds
 * - B Button: Test at 1500 RPM for 3 seconds  
 * - X Button: Test at 2000 RPM for 3 seconds
 * - Y Button: Test at 2500 RPM for 3 seconds
 * - Right Bumper: Custom speed test (adjust speed with right stick Y)
 * - Left Bumper: Stop motor immediately
 * - Right Stick Y: Adjust custom speed (-1.0 to 1.0, displayed as RPM)
 * 
 * @author Sahas Kumar
 */
@TeleOp(name = "motorencodertest", group = "Test")
public class motorencodertest extends LinearOpMode {

    // Shooter motor
    private DcMotorEx shooterMotor;
    
    // Test parameters
    private final double TEST_DURATION_SECONDS = 3.0; // How long each test runs
    private final double RPM_TO_VELOCITY = 28.0; // Conversion factor (adjust based on your motor)
    
    // Test speeds in RPM
    private final double SPEED_1000_RPM = 1000.0;
    private final double SPEED_1500_RPM = 1500.0;
    private final double SPEED_2000_RPM = 2000.0;
    private final double SPEED_2500_RPM = 2500.0;
    
    // State variables
    private boolean isTestRunning = false;
    private ElapsedTime testTimer = new ElapsedTime();
    private double currentTargetRPM = 0.0;
    private double customSpeed = 0.0; // -1.0 to 1.0 from right stick
    
    // Right bumper test variables
    private boolean rightBumperPressed = false;
    private boolean rightBumperTestActive = false;
    private ElapsedTime rightBumperTimer = new ElapsedTime();
    private final double RIGHT_BUMPER_TEST_DURATION = 2.0; // 2 seconds
    private final double RIGHT_BUMPER_TARGET_RPM = 1500.0; // Target RPM for right bumper test
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize shooter motor
        initializeShooterMotor();
        
        // Display instructions
        telemetry.addData("Status", "Shooter Motor Test Ready!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A Button", "Test 1000 RPM for 3 seconds");
        telemetry.addData("B Button", "Test 1500 RPM for 3 seconds");
        telemetry.addData("X Button", "Test 2000 RPM for 3 seconds");
        telemetry.addData("Y Button", "Test 2500 RPM for 3 seconds");
        telemetry.addData("Right Bumper", "Run 1500 RPM for 2 seconds then BRAKE");
        telemetry.addData("Left Bumper", "Stop motor");
        telemetry.addData("Right Stick Y", "Adjust custom speed");
        telemetry.addLine();
        telemetry.addData("Note", "Motor will run for " + TEST_DURATION_SECONDS + " seconds per test");
        telemetry.update();
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            
            // Update custom speed from right stick
            updateCustomSpeed();
            
            // Handle button presses for different test speeds
            handleTestButtons();
            
            // Handle right bumper test (separate from other tests)
            handleRightBumperTest();
            
            // Handle stop button
            if (gamepad1.left_bumper) {
                stopMotor();
            }
            
            // Run current test if active
            if (isTestRunning) {
                runCurrentTest();
            }
            
            // Run right bumper test if active
            if (rightBumperTestActive) {
                runRightBumperTest();
            }
            
            // Update telemetry
            updateTelemetry();
        }
        
        // Stop motor when program ends
        stopMotor();
    }
    
    /**
     * Initialize the shooter motor with proper settings
     */
    private void initializeShooterMotor() {
        try {
            // Get motor from hardware map
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            
            // Set motor direction
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            
            // Set zero power behavior
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            
            // Reset encoder and set to velocity control mode
            shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            
            // Set PIDF coefficients for better velocity control
            shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(300, 0, 0, 10));
            
            // Stop motor initially
            shooterMotor.setVelocity(0);
            
            telemetry.addData("Status", "Shooter motor initialized successfully");
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor: " + e.getMessage());
        }
    }
    
    /**
     * Update custom speed from right stick input
     */
    private void updateCustomSpeed() {
        // Get right stick Y input (-1.0 to 1.0)
        double stickInput = -gamepad1.right_stick_y; // Invert so up is positive
        
        // Convert to RPM range (0 to 3000 RPM)
        customSpeed = stickInput;
        double customRPM = (stickInput + 1.0) * 1500.0; // Maps -1 to 1 -> 0 to 3000 RPM
        
        // Display custom speed in telemetry
        telemetry.addData("Custom Speed", String.format("%.1f%% (%.0f RPM)", 
            customSpeed * 100, customRPM));
    }
    
    /**
     * Handle button presses for different test speeds
     */
    private void handleTestButtons() {
        // Only start new test if not currently running
        if (!isTestRunning && !rightBumperTestActive) {
            if (gamepad1.a) {
                startTest(SPEED_1000_RPM, "1000 RPM");
            } else if (gamepad1.b) {
                startTest(SPEED_1500_RPM, "1500 RPM");
            } else if (gamepad1.x) {
                startTest(SPEED_2000_RPM, "2000 RPM");
            } else if (gamepad1.y) {
                startTest(SPEED_2500_RPM, "2500 RPM");
            }
        }
    }
    
    /**
     * Start a new test with specified RPM
     */
    private void startTest(double targetRPM, String testName) {
        currentTargetRPM = targetRPM;
        isTestRunning = true;
        testTimer.reset();
        
        // Convert RPM to velocity (adjust conversion factor as needed)
        double targetVelocity = targetRPM * RPM_TO_VELOCITY;
        
        // Set motor to target velocity
        shooterMotor.setVelocity(targetVelocity);
        
        telemetry.addData("TEST STARTED", testName + " for " + TEST_DURATION_SECONDS + " seconds");
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Target Velocity", String.format("%.0f", targetVelocity));
    }
    
    /**
     * Run the current test and check if it should stop
     */
    private void runCurrentTest() {
        double elapsedTime = testTimer.seconds();
        double remainingTime = TEST_DURATION_SECONDS - elapsedTime;
        
        // Check if test duration is complete
        if (elapsedTime >= TEST_DURATION_SECONDS) {
            // Test complete, stop motor
            stopMotor();
        } else {
            // Test still running, show remaining time
            telemetry.addData("Test Running", String.format("%.1f seconds remaining", remainingTime));
        }
    }
    
    /**
     * Handle right bumper test - runs motor to target speed then brakes completely
     */
    private void handleRightBumperTest() {
        // Detect right bumper press (edge detection)
        boolean currentRightBumper = gamepad1.right_bumper;
        boolean rightBumperJustPressed = currentRightBumper && !rightBumperPressed;
        rightBumperPressed = currentRightBumper;
        
        // Start right bumper test if pressed and no other test is running
        if (rightBumperJustPressed && !isTestRunning && !rightBumperTestActive) {
            startRightBumperTest();
        }
    }
    
    /**
     * Start the right bumper test
     */
    private void startRightBumperTest() {
        rightBumperTestActive = true;
        rightBumperTimer.reset();
        
        // Convert RPM to velocity
        double targetVelocity = RIGHT_BUMPER_TARGET_RPM * RPM_TO_VELOCITY;
        
        // Set motor to target velocity
        shooterMotor.setVelocity(targetVelocity);
        
        telemetry.addData("RIGHT BUMPER TEST", "Started - " + RIGHT_BUMPER_TARGET_RPM + " RPM for " + RIGHT_BUMPER_TEST_DURATION + " seconds");
    }
    
    /**
     * Run the right bumper test and handle completion
     */
    private void runRightBumperTest() {
        double elapsedTime = rightBumperTimer.seconds();
        double remainingTime = RIGHT_BUMPER_TEST_DURATION - elapsedTime;
        
        // Check if test duration is complete
        if (elapsedTime >= RIGHT_BUMPER_TEST_DURATION) {
            // Test complete, brake motor completely
            brakeMotor();
        } else {
            // Test still running, show remaining time
            telemetry.addData("Right Bumper Test", String.format("%.1f seconds remaining", remainingTime));
        }
    }
    
    /**
     * Brake the motor completely (set to 0 velocity and ensure it stops)
     */
    private void brakeMotor() {
        // Set velocity to 0
        shooterMotor.setVelocity(0);
        
        // Set to brake mode to ensure complete stop
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        // Reset test state
        rightBumperTestActive = false;
        isTestRunning = false;
        currentTargetRPM = 0.0;
        
        telemetry.addData("Motor Status", "BRAKED - Complete Stop");
    }
    
    /**
     * Stop the motor and reset test state
     */
    private void stopMotor() {
        shooterMotor.setVelocity(0);
        isTestRunning = false;
        rightBumperTestActive = false;
        currentTargetRPM = 0.0;
        telemetry.addData("Motor Status", "STOPPED");
    }
    
    /**
     * Update telemetry with motor information
     */
    private void updateTelemetry() {
        telemetry.clear();
        
        // Header
        telemetry.addLine("=== SHOOTER MOTOR TEST ===");
        telemetry.addLine();
        
        // Motor status
        telemetry.addLine("--- MOTOR STATUS ---");
        telemetry.addData("Test Running", isTestRunning ? "YES" : "NO");
        telemetry.addData("Right Bumper Test", rightBumperTestActive ? "YES" : "NO");
        telemetry.addData("Current Velocity", String.format("%.1f", shooterMotor.getVelocity()));
        telemetry.addData("Current RPM", String.format("%.0f", shooterMotor.getVelocity() / RPM_TO_VELOCITY));
        telemetry.addData("Target RPM", String.format("%.0f", currentTargetRPM));
        telemetry.addData("Motor Power", String.format("%.3f", shooterMotor.getPower()));
        telemetry.addData("Motor Mode", shooterMotor.getMode().toString());
        telemetry.addData("Current Position", shooterMotor.getCurrentPosition());
        telemetry.addLine();
        
        // Test information
        if (isTestRunning) {
            telemetry.addLine("--- CURRENT TEST ---");
            telemetry.addData("Elapsed Time", String.format("%.1f", testTimer.seconds()));
            telemetry.addData("Remaining Time", String.format("%.1f", TEST_DURATION_SECONDS - testTimer.seconds()));
            telemetry.addLine();
        }
        
        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A", "1000 RPM Test");
        telemetry.addData("B", "1500 RPM Test");
        telemetry.addData("X", "2000 RPM Test");
        telemetry.addData("Y", "2500 RPM Test");
        telemetry.addData("Right Bumper", "1500 RPM for 2s then BRAKE");
        telemetry.addData("Left Bumper", "Stop Motor");
        telemetry.addData("Right Stick Y", "Adjust Custom Speed");
        telemetry.addLine();
        
        // Custom speed display
        double customRPM = (customSpeed + 1.0) * 1500.0;
        telemetry.addData("Custom Speed", String.format("%.1f%% (%.0f RPM)", 
            customSpeed * 100, customRPM));
        
        telemetry.update();
    }
}
