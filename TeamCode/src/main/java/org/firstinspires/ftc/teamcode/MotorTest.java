package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * MotorTest - Comprehensive motor and encoder testing program
 * 
 * Features:
 * - Test all four drive motors individually
 * - Verify wheel direction and port mapping
 * - Monitor encoder ticks in real-time
 * - Debug motor configuration issues
 * 
 * Controls:
 * - Left stick Y: Test backLeft motor
 * - Right stick Y: Test backRight motor  
 * - Left stick X: Test frontLeft motor
 * - Right stick X: Test frontRight motor
 * - A button: Reset all encoders
 * - B button: Toggle encoder mode (with/without encoder)
 */
@TeleOp(name = "MotorTest", group = "Test")
public class MotorTest extends LinearOpMode {

    // Motor declarations for all four drive wheels
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // Encoder mode tracking
    private boolean useEncoders = true;
    private boolean encodersReset = false;

    @Override
    public void runOpMode() {
        
        // Initialize all drive motors with hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions (adjust these based on your robot's configuration)
        // Note: These directions may need to be adjusted based on your specific robot setup
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure motors for encoder usage
        configureMotorsForEncoders();

        // Initial telemetry
        telemetry.addData("Status", "MotorTest initialized - Ready to test!");
        telemetry.addData("Instructions", "Use sticks to test individual motors");
        telemetry.addData("A Button", "Reset encoders");
        telemetry.addData("B Button", "Toggle encoder mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            // Handle encoder reset (A button)
            if (gamepad1.a && !encodersReset) {
                resetAllEncoders();
                encodersReset = true;
            } else if (!gamepad1.a) {
                encodersReset = false;
            }
            
            // Handle encoder mode toggle (B button)
            if (gamepad1.b) {
                toggleEncoderMode();
                sleep(200); // Debounce
            }

            // Test individual motors with gamepad sticks
            testMotorWithStick(frontLeft, gamepad1.left_stick_x, "Front Left");
            testMotorWithStick(frontRight, gamepad1.right_stick_x, "Front Right");
            testMotorWithStick(backLeft, gamepad1.left_stick_y, "Back Left");
            testMotorWithStick(backRight, gamepad1.right_stick_y, "Back Right");

            // Update comprehensive telemetry
            updateTelemetry();
        }
    }
    
    /**
     * Configure all motors for encoder usage
     */
    private void configureMotorsForEncoders() {
        DcMotor.RunMode mode = useEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
        
        // Set zero power behavior to brake for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Test a specific motor with stick input
     */
    private void testMotorWithStick(DcMotor motor, double stickInput, String motorName) {
        // Apply deadzone to prevent drift
        if (Math.abs(stickInput) < 0.1) {
            stickInput = 0;
        }
        
        motor.setPower(stickInput);
    }
    
    /**
     * Reset all motor encoders
     */
    private void resetAllEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Return to encoder mode after reset
        configureMotorsForEncoders();
        
        telemetry.addData("Encoders", "Reset complete!");
        telemetry.update();
    }
    
    /**
     * Toggle between encoder and non-encoder modes
     */
    private void toggleEncoderMode() {
        useEncoders = !useEncoders;
        configureMotorsForEncoders();
    }
    
    /**
     * Update comprehensive telemetry for debugging
     */
    private void updateTelemetry() {
        // Clear previous telemetry
        telemetry.clear();
        
        // Header information
        telemetry.addLine("=== MOTOR TEST DEBUG ===");
        telemetry.addData("Encoder Mode", useEncoders ? "ENABLED" : "DISABLED");
        telemetry.addLine();
        
        // Motor power and encoder data
        telemetry.addLine("--- MOTOR STATUS ---");
        addMotorTelemetry(frontLeft, "Front Left", "Port: Check config");
        addMotorTelemetry(frontRight, "Front Right", "Port: Check config");
        addMotorTelemetry(backLeft, "Back Left", "Port: Check config");
        addMotorTelemetry(backRight, "Back Right", "Port: Check config");
        
        telemetry.addLine();
        telemetry.addLine("--- GAMEPAD CONTROLS ---");
        telemetry.addData("Left Stick X", "Front Left Motor (Left/Right)");
        telemetry.addData("Right Stick X", "Front Right Motor (Left/Right)");
        telemetry.addData("Left Stick Y", "Back Left Motor (Up/Down)");
        telemetry.addData("Right Stick Y", "Back Right Motor (Up/Down)");
        telemetry.addLine();
        telemetry.addData("A Button", "Reset All Encoders to Zero");
        telemetry.addData("B Button", "Toggle Encoder Mode (ON/OFF)");
        telemetry.addLine();
        telemetry.addLine("--- WHEEL TESTING GUIDE ---");
        telemetry.addData("Front Left", "Move LEFT STICK LEFT/RIGHT");
        telemetry.addData("Front Right", "Move RIGHT STICK LEFT/RIGHT");
        telemetry.addData("Back Left", "Move LEFT STICK UP/DOWN");
        telemetry.addData("Back Right", "Move RIGHT STICK UP/DOWN");
        
        telemetry.addLine();
        telemetry.addLine("--- DEBUGGING TIPS ---");
        telemetry.addData("Direction Test", "Positive stick = forward wheel rotation");
        telemetry.addData("Encoder Test", "Watch ticks increase when motor runs");
        telemetry.addData("Port Check", "Verify motor names match REV Hub config");
        
        telemetry.update();
    }
    
    /**
     * Add telemetry for a specific motor
     */
    private void addMotorTelemetry(DcMotor motor, String name, String portInfo) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData("Power", String.format("%.2f", motor.getPower()));
        telemetry.addData("Direction", motor.getDirection().toString());
        telemetry.addData("Mode", motor.getMode().toString());
        
        if (useEncoders) {
            telemetry.addData("Encoder Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", motor.getTargetPosition());
            telemetry.addData("Busy", motor.isBusy());
        }
        
        telemetry.addData("Port Info", portInfo);
        telemetry.addLine();
    }
}


