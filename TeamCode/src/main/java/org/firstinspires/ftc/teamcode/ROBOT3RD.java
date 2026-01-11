package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * ROBOT3RD - Simple Field-Centric TeleOp Program
 * 
 * This is a simple field-centric drive program using the default REV IMU.
 * Field-centric control means the robot moves relative to the field, not relative to the robot's current heading.
 * 
 * Key Features:
 * - Field-Oriented Control (FOC) using default REV IMU
 * - Simple mecanum drive with joystick control
 * - IMU reset with A button
 * 
 * Controls:
 * - Left Stick Y: Move forward/backward (field-relative)
 * - Left Stick X: Strafe left/right (field-relative)  
 * - Right Stick X: Rotate left/right
 * - A Button: Reset IMU yaw to 0°
 * 
 * Hardware:
 * - 4 Drivetrain motors: frontLeft, frontRight, backLeft, backRight
 * - IMU: imu (default REV IMU)
 * 
 * @author Team
 * @version 1.0 - Simple field-centric drive
 */
@TeleOp(name = "ROBOT3RD", group = "TeleOp")
public class ROBOT3RD extends LinearOpMode {

    // ========================================
    // DRIVETRAIN MOTORS
    // ========================================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    // ========================================
    // IMU FOR FIELD-CENTRIC CONTROL
    // ========================================
    private IMU imu;  // Default REV IMU
    
    // ========================================
    // DRIVE CONTROL CONSTANTS
    // ========================================
    private static final double JOYSTICK_DEADZONE = 0.10;  // Ignore small joystick movements to prevent drift
    private static final double DRIVE_POWER_MULTIPLIER = 0.5;  // Overall speed control (1.0 = 100% speed)
    
    // Track previous button state for IMU reset
    private boolean prevAButton = false;

    @Override
    public void runOpMode() {
        // ========================================
        // STEP 1: INITIALIZE HARDWARE
        // ========================================
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        
        // Set motor directions (adjust if needed for your robot)
        // Typically, left motors are reversed
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // ========================================
        // STEP 2: INITIALIZE IMU
        // ========================================
        telemetry.addData("Status", "Initializing REV IMU...");
        telemetry.update();
        
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            
            // Configure IMU orientation (adjust these values to match your robot's orientation)
            // Logo facing UP and USB facing FORWARD is a common default
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            
            RevHubOrientationOnRobot orientationOnRobot = 
                    new RevHubOrientationOnRobot(logoDirection, usbDirection);
            
            // Initialize IMU with orientation parameters
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
            imu.initialize(parameters);
            
            // Reset yaw to 0° (robot should be facing away from driver toward field)
            imu.resetYaw();
            
            telemetry.addData("Status", "REV IMU initialized successfully");
            telemetry.addData("IMU", "Ready - Press A to reset yaw");
        } catch (Exception e) {
            telemetry.addData("ERROR", "REV IMU initialization failed: " + e.getMessage());
            telemetry.addData("WARNING", "Field-centric control will be disabled");
            imu = null;
        }
        
        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("Press A to reset IMU yaw");
        telemetry.update();
        
        // Wait for start button
        waitForStart();
        
        // ========================================
        // STEP 3: MAIN CONTROL LOOP
        // ========================================
        while (opModeIsActive() && !isStopRequested()) {
            
            // ========================================
            // STEP 4: HANDLE IMU RESET (A BUTTON)
            // ========================================
            // Reset IMU yaw to 0° when A button is pressed
            boolean aButtonCurrentlyPressed = gamepad1.a;
            boolean aButtonJustPressed = aButtonCurrentlyPressed && !prevAButton;
            
            if (aButtonJustPressed && imu != null) {
                try {
                    imu.resetYaw();
                    telemetry.addData("IMU RESET", "Yaw reset to 0° (A button pressed)");
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to reset IMU yaw: " + e.getMessage());
                }
            }
            prevAButton = aButtonCurrentlyPressed;
            
            // ========================================
            // STEP 5: READ JOYSTICK INPUTS
            // ========================================
            // Get joystick values (inverted Y for forward/backward)
            double y = -gamepad1.left_stick_y;  // Forward/backward (inverted)
            double x = gamepad1.left_stick_x;   // Strafe left/right
            double rx = gamepad1.right_stick_x;  // Rotate left/right
            
            // Apply deadzone to prevent drift
            if (Math.abs(y) < JOYSTICK_DEADZONE) y = 0.0;
            if (Math.abs(x) < JOYSTICK_DEADZONE) x = 0.0;
            if (Math.abs(rx) < JOYSTICK_DEADZONE) rx = 0.0;
            
            // ========================================
            // STEP 6: FIELD-CENTRIC DRIVE CONTROL
            // ========================================
            driveFieldCentric(x, y, rx);
            
            // ========================================
            // STEP 7: UPDATE TELEMETRY
            // ========================================
            updateTelemetry();
        }
        
        // Stop all motors when op mode ends
        stopAllMotors();
    }
    
    /**
     * Drive the robot using field-centric control
     * Field-centric: Forward always moves toward the field, regardless of robot orientation
     * 
     * @param x Strafe input (left/right)
     * @param y Forward input (forward/backward)
     * @param rx Rotation input (rotate left/right)
     */
    private void driveFieldCentric(double x, double y, double rx) {
        // Get the robot's current heading from REV IMU
        double botHeading = 0.0;
        boolean imuAvailable = false;
        
        if (imu != null) {
            try {
                // Get yaw angle in radians
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                imuAvailable = true;
            } catch (Exception e) {
                // If IMU read fails, use 0 heading (robot-centric mode)
                telemetry.addData("ERROR", "REV IMU read failed, using robot-centric: " + e.getMessage());
                botHeading = 0.0;
                imuAvailable = false;
            }
        } else {
            telemetry.addData("ERROR", "REV IMU is NULL - using robot-centric mode");
            botHeading = 0.0;
            imuAvailable = false;
        }
        
        // Convert joystick input to field coordinates (field-centric control)
        // This makes forward always move toward the field, not the robot's current direction
        if (imuAvailable) {
            // Rotate joystick input based on robot's heading
            // This matches the pattern used in RED_DEC_TELEOP and BLUE_DEC_TELEOP
            double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
            x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
            y = temp;
        }
        
        // Mecanum wheel calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        
        // Apply power multiplier
        flPower *= DRIVE_POWER_MULTIPLIER;
        blPower *= DRIVE_POWER_MULTIPLIER;
        frPower *= DRIVE_POWER_MULTIPLIER;
        brPower *= DRIVE_POWER_MULTIPLIER;
        
        // Clamp power to maximum of 1.0
        flPower = Math.max(-1.0, Math.min(1.0, flPower));
        blPower = Math.max(-1.0, Math.min(1.0, blPower));
        frPower = Math.max(-1.0, Math.min(1.0, frPower));
        brPower = Math.max(-1.0, Math.min(1.0, brPower));
        
        // Apply motor powers
        if (frontLeft != null) frontLeft.setPower(flPower);
        if (backLeft != null) backLeft.setPower(blPower);
        if (frontRight != null) frontRight.setPower(frPower);
        if (backRight != null) backRight.setPower(brPower);
    }
    
    /**
     * Update telemetry with current robot status
     */
    private void updateTelemetry() {
        telemetry.addLine("=== ROBOT3RD ===");
        telemetry.addLine("");
        
        // IMU status
        if (imu != null) {
            try {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addData("IMU Heading", String.format("%.1f°", heading));
                telemetry.addData("Field-Centric", "ACTIVE");
            } catch (Exception e) {
                telemetry.addData("IMU Status", "ERROR: " + e.getMessage());
                telemetry.addData("Field-Centric", "DISABLED");
            }
        } else {
            telemetry.addData("IMU Status", "NOT INITIALIZED");
            telemetry.addData("Field-Centric", "DISABLED");
        }
        
        telemetry.addLine("");
        telemetry.addData("Controls", "A = Reset IMU Yaw");
        telemetry.addData("Left Stick", "Move (field-relative)");
        telemetry.addData("Right Stick X", "Rotate");
        
        telemetry.update();
    }
    
    /**
     * Stop all motors (cleanup)
     */
    private void stopAllMotors() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }
}
