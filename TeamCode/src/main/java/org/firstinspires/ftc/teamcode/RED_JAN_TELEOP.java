package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Limelight vision sensor imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

// Drivetrain subsystem
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * RED_JAN_TELEOP - Field-Oriented Control TeleOp Program for January Robot
 * 
 * This program provides smooth, field-oriented control using the GoBilda PinPoint IMU.
 * Field-oriented control means the robot moves relative to the field, not relative to the robot's current heading.
 * 
 * Key Features:
 * - Field-Oriented Control (FOC) using GoBilda PinPoint IMU ("odo")
 * - Direct IMU-based field-centric control (no odometry required)
 * - Dual shooter motors with velocity control
 * - Intake motor with forward/reverse control
 * - Feed motor tied to shooter action
 * - Anti-drift measures to prevent unwanted movement
 * - Bulk reading for faster performance
 * - Easy-to-understand code with lots of comments of beginner 6th grader coders (Atharva, Dhruv)
 * 
 * Controls:
 * - Left Stick Y: Move forward/backward (field-relative)
 * - Left Stick X: Strafe left/right (field-relative)  
 * - Right Stick X: Rotate left/right
 * - L1 Button: Intake forward (0.95 power)
 * - L2 Trigger: Intake reverse (0.85 power) - spits out balls
 * - R1 Button: Shooter trigger - spins up shooters, feeds ball when velocity reached, runs intake at 0.85 power
 * 
 * Hardware:
 * - 4 Drivetrain motors: frontLeft, frontRight, backLeft, backRight
 * - Intake motor: intakeMotor (no encoder)
 * - Feed motor: feedMotor (no encoder, constant speed)
 * - Shooter motors: shooterMotorLeft, shooterMotorRight (velocity controlled, opposite directions)
 * 
 * @author Team
 * @version 1.0 - Initial January robot implementation
 */
@TeleOp(name = "RED_JAN_TELEOP", group = "TeleOp")
public class RED_JAN_TELEOP extends LinearOpMode {

    // ========================================
    // DRIVETRAIN MOTORS (MANUAL CONTROL)
    // ========================================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;  // Direct motor control (like BLUE_DEC_TELEOP)
    
    // ========================================
    // PEDROPATHING FOR HEADING AND POSE
    // ========================================
    private MecanumDrive drive;  // Only used for heading and pose tracking (not motor control)
    
    // ========================================
    // INTAKE AND FEEDING SYSTEM
    // ========================================
    private DcMotor intakeMotor;        // Intake motor (no encoder)
    private DcMotor feedMotor;           // Feed motor (no encoder, constant speed)
    
    // ========================================
    // SHOOTER SYSTEM
    // ========================================
    private DcMotorEx shooterMotorLeft;  // Left shooter motor (velocity controlled)
    private DcMotorEx shooterMotorRight; // Right shooter motor (velocity controlled, opposite direction)
    
    // ========================================
    // LED/RGB INDICATOR
    // ========================================
    private Servo rgb;  // RGB LED light (controlled like a servo)
    
    // ========================================
    // IMU FOR FIELD-CENTRIC CONTROL
    // ========================================
    // Note: IMU/odometry is handled by PedroPathing Follower - do not access directly to avoid conflicts
    
    // ========================================
    // LIMELIGHT VISION SENSOR
    // ========================================
    private Limelight3A limelight;      // Limelight for distance-based velocity calculation
    
    // ========================================
    // BULK READING FOR PERFORMANCE
    // ========================================
    private List<LynxModule> allHubs;
    
    // ========================================
    // ALL CONSTANTS
    // ========================================
    
    // Drive Control Constants
    private static final double JOYSTICK_DEADZONE = 0.10;  // Ignore small joystick movements to prevent drift
    private static final double MIN_MOTOR_POWER = 0.12;    // Minimum power needed to make motors move
    private static final double DRIVE_POWER_MULTIPLIER = 0.8;  // Overall speed control (0.8 = 80% speed)
    private static final double STRAFE_POWER_MULTIPLIER = 1.3;  // Strafe speed multiplier (1.3 = 30% faster strafe than forward)
    
    // Intake Motor Constants
    private static final double INTAKE_FORWARD_POWER = 0.95;  // Power when L1 button pressed (forward)
    private static final double INTAKE_REVERSE_POWER = 0.85;  // Power when L2 trigger pressed (reverse)
    private static final double INTAKE_SHOOTER_POWER = 0.85;   // Power when R1 pressed (helps feed ball)
    
    // Feed Motor Constants - now dynamic based on R1 (0.8) or R2 (0.4)
    // Feed motor power is set dynamically: R1 (right bumper) = 0.8, R2 (right trigger) = 0.4
    
    // Shooter Distance Constants
    private static final double SHORT_DISTANCE_INCHES = 36.0;   // Closest shooting distance (inches)
    private static final double LONG_DISTANCE_INCHES = 130.0;   // Farthest shooting distance (inches)
    private static final double SHORT_DISTANCE_VELOCITY = 1000.0;  // Shooter speed at close distance (RPM) - reduced by 50 RPM
    private static final double LONG_DISTANCE_VELOCITY = 1400.0;   // Shooter speed at far distance (RPM) - reduced by 50 RPM
    private static final double MIN_VELOCITY = 800.0;   // Slowest allowed shooter speed (RPM)
    private static final double MAX_VELOCITY = 1700.0;  // Fastest allowed shooter speed (RPM)
    
    // Shooter Velocity Adjustments
    private static final double SHORT_DISTANCE_RPM_REDUCTION = 100.0;  // Reduce speed by this much for close shots (RPM)
    private static final double REDUCTION_TAPER_DISTANCE = 80.0;       // Distance where reduction stops (inches)
    
    // Optional: Single distance calibration multiplier if base distance calculation needs adjustment
    // Set to 1.0 to use raw Limelight distance, or tune if distance readings are consistently off
    private static final double DISTANCE_CALIBRATION_MULTIPLIER = 1.0;  // Tune this if distance needs adjustment
    
    // Shooter Control Constants
    private static final double SHOOTER_TARGET_VELOCITY = 1000.0;  // Default speed when Limelight not working (RPM)
    private static final double SHOOTER_VELOCITY_THRESHOLD = 0.95; // Wait until 95% of target speed before feeding
    private static final double SHOOTER_PIDF_P = 100.0;  // Proportional gain (how fast to reach target speed)
    private static final double SHOOTER_PIDF_I = 0.0;    // Integral gain (not used)
    private static final double SHOOTER_PIDF_D = 0.0;    // Derivative gain (not used)
    private static final double SHOOTER_PIDF_F = 30.0;   // Feedforward gain (helps maintain speed)
    
    // RGB LED Constants
    private static final double RGB_RED = 0.2777;     // Red color position
    private static final double RGB_GREEN = 0.5;      // Green color position
    private static final double RGB_ORANGE = 0.333;   // Orange color position
    
    // Auto-Align Constants
    private static final double AUTO_ALIGN_TX_TOLERANCE = 0.5;   // How close to center is "aligned" (degrees)
    private static final double AUTO_ALIGN_ROTATION_SPEED = 0.35; // Maximum rotation speed for alignment
    private static final double AUTO_ALIGN_TX_GAIN = 0.14;       // How fast to rotate based on target position
    private static final double AUTO_ALIGN_TIMEOUT = 3.0;        // Stop trying after 3 seconds
    private static final double AUTO_ALIGN_MIN_ROTATE = 0.05;    // Minimum rotation to make robot move
    
    // Dynamic velocity calculation (updated based on limelight distance)
    private double currentCalculatedVelocity = SHOOTER_TARGET_VELOCITY; // Current target velocity
    private double currentFeedMotorPower = 0.4;  // Current feed motor power (set by R1=0.8 or R2=0.4)
    
    // ========================================
    // SHOOTER STATE MACHINE
    // ========================================
    private enum ShooterState {
        IDLE,        // Shooter is stopped, waiting for trigger
        SPIN_UP,     // Shooter is spinning up to target velocity
        READY,        // Shooter at target velocity, ready to feed
        FEEDING       // Currently feeding ball to shooter
    }
    
    private ShooterState shooterState = ShooterState.IDLE;
    private boolean r1Pressed = false;  // Track R1 button state for edge detection
    private boolean prevDpadDown = false;  // Track D-pad down state for edge detection
    
    // ========================================
    // AUTO-ALIGN VARIABLES
    // ========================================
    private boolean autoAlignMode = false;  // Is auto-align currently active?
    private double autoAlignRotationCommand = 0.0;  // How much to rotate (from auto-align)
    private ElapsedTime autoAlignTimer = new ElapsedTime();  // Timer to stop auto-align after timeout
    
    // ========================================
    // FIELD-CENTRIC DEBUG VARIABLES
    // ========================================
    private double fcRawHeading = 0.0;
    private double fcHeadingUsed = 0.0;
    private double fcYBefore = 0.0;
    private double fcXBefore = 0.0;
    private double fcYAfter = 0.0;
    private double fcXAfter = 0.0;
    private String fcStatus = "NOT RUN";
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // ========================================
        // STEP 1: SETUP BULK READING FOR SPEED
        // ========================================
        // Read all sensors at once to make robot respond faster
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ========================================
        // STEP 2: INITIALIZE DRIVETRAIN MOTORS (MANUAL CONTROL)
        // ========================================
        // Initialize drive motors directly (matching BLUE_DEC_TELEOP)
        boolean driveMotorsInitialized = true;
        
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            if (frontLeft == null) {
                telemetry.addData("ERROR", "Front left motor 'frontLeft' not found!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize front left motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            if (frontRight == null) {
                telemetry.addData("ERROR", "Front right motor 'frontRight' not found!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize front right motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            if (backLeft == null) {
                telemetry.addData("ERROR", "Back left motor 'backLeft' not found!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize back left motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");
            if (backRight == null) {
                telemetry.addData("ERROR", "Back right motor 'backRight' not found!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize back right motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        if (!driveMotorsInitialized) {
            telemetry.addData("CRITICAL ERROR", "Drive motors failed to initialize - robot cannot move!");
        }
        
        // Set motor directions (matching BLUE_DEC_TELEOP)
        if (frontRight != null) frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontLeft != null) frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motors to brake when no power is applied
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // ========================================
        // STEP 2.5: INITIALIZE PEDROPATHING FOR HEADING/POSE ONLY
        // ========================================
        // Initialize PedroPathing Follower ONLY for heading and pose tracking (not motor control)
        try {
            drive = new MecanumDrive(hardwareMap, null);
            // Don't start teleop drive mode - we're controlling motors manually
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize PedroPathing (heading/pose only): " + e.getMessage());
            drive = null;
        }
        
        // ========================================
        // STEP 3: INITIALIZE INTAKE MOTOR
        // ========================================
        boolean intakeInitialized = true;
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            if (intakeMotor == null) {
                telemetry.addData("ERROR", "Intake motor 'intakeMotor' not found in hardware map!");
                intakeInitialized = false;
            } else {
                // Set motor direction - REVERSED to change direction
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                
                // No encoder mode - run without encoder
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Changed to FLOAT for smoother operation
                intakeMotor.setPower(0.0); // Stop initially
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize intake motor: " + e.getMessage());
            intakeInitialized = false;
        }
        
        // ========================================
        // STEP 4: INITIALIZE FEED MOTOR
        // ========================================
        boolean feedInitialized = true;
        try {
            feedMotor = hardwareMap.get(DcMotor.class, "feedMotor");
            if (feedMotor == null) {
                telemetry.addData("ERROR", "Feed motor 'feedMotor' not found in hardware map!");
                feedInitialized = false;
            } else {
                // Set motor direction - REVERSED
                feedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                
                // No encoder mode - run without encoder at constant speed
                feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                feedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                feedMotor.setPower(0.0); // Stop initially
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize feed motor: " + e.getMessage());
            feedInitialized = false;
        }
        
        // ========================================
        // STEP 5: INITIALIZE SHOOTER MOTORS
        // ========================================
        boolean shooterInitialized = true;
        
        try {
            shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
            if (shooterMotorLeft == null) {
                telemetry.addData("ERROR", "Shooter motor left 'shooterMotorLeft' not found in hardware map!");
                shooterInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor left: " + e.getMessage());
            shooterInitialized = false;
        }
        
        try {
            shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
            if (shooterMotorRight == null) {
                telemetry.addData("ERROR", "Shooter motor right 'shooterMotorRight' not found in hardware map!");
                shooterInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor right: " + e.getMessage());
            shooterInitialized = false;
        }
        
        if (shooterInitialized && shooterMotorLeft != null && shooterMotorRight != null) {
            // Set opposite directions for flywheel
            // REVERSED for debugging - swapped from original FORWARD/REVERSE
            shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
            
            // Set to FLOAT (coast) mode - allows motors to coast to stop naturally
            // Changed from BRAKE so velocity reduces gradually when power is removed
            shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            
            // Configure for velocity control with encoder
            try {
                shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // Set same PIDF coefficients for both motors
                PIDFCoefficients pidfCoeffs = new PIDFCoefficients(
                    SHOOTER_PIDF_P, 
                    SHOOTER_PIDF_I, 
                    SHOOTER_PIDF_D, 
                    SHOOTER_PIDF_F
                );
                
                shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
                shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to configure shooter motors: " + e.getMessage());
                shooterInitialized = false;
            }
        }
        
        // ========================================
        // STEP 6: IMU/ODOMETRY HANDLED BY PEDROPATHING
        // ========================================
        // Note: PedroPathing Follower handles all IMU/odometry access via Constants.java
        // Do NOT access "odo" directly to avoid "multiple drivers" conflict
        // The Follower initializes and manages the PinpointDriver internally
        telemetry.addData("Status", "IMU/Odometry managed by PedroPathing Follower");
        
        // ========================================
        // STEP 6.5: INITIALIZE LIMELIGHT VISION SENSOR
        // ========================================
        // Initialize Limelight for distance-based velocity calculation
        boolean limelightInitialized = true;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight == null) {
                telemetry.addData("ERROR", "Limelight 'limelight' not found in hardware map!");
                limelightInitialized = false;
            } else {
                // Configure Limelight pipeline (use pipeline 0 for MegaTag2)
                limelight.pipelineSwitch(0);
                
                // Start Limelight
                limelight.start();
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Limelight initialization failed: " + e.getMessage());
            limelightInitialized = false;
            limelight = null;
        }
        
        if (!limelightInitialized) {
            telemetry.addData("WARNING", "Limelight not available - using default velocity");
        }
        
        // ========================================
        // STEP 6.6: INITIALIZE RGB LED INDICATOR
        // ========================================
        // Initialize RGB LED servo for visual feedback on Limelight status
        boolean rgbInitialized = true;
        try {
            rgb = hardwareMap.get(Servo.class, "rgb");
            if (rgb == null) {
                telemetry.addData("ERROR", "RGB LED 'rgb' not found in hardware map!");
                rgbInitialized = false;
            } else {
                // Default to RED on init until we have a Limelight reading
                rgb.setPosition(RGB_RED);
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "RGB LED initialization failed: " + e.getMessage());
            rgbInitialized = false;
            rgb = null;
        }
        
        if (!rgbInitialized) {
            telemetry.addData("WARNING", "RGB LED not available - no visual feedback");
        }
        
        // ========================================
        // STEP 7: READY TO START
        // ========================================
        telemetry.addData("Status", "RED_JAN_TELEOP Ready!");
        telemetry.addData("Localization", "PedroPathing Follower (Pinpoint Odometry)");
        telemetry.addData("Control Mode", "Field-Oriented Control (via Follower)");
        telemetry.addData("Drivetrain", drive != null ? "Initialized" : "FAILED");
        telemetry.addData("Instructions", "Use left stick to move, right stick X to rotate");
        telemetry.addData("Intake", "L1 forward, L2 reverse");
        telemetry.addData("Shooter", "R1 to shoot");
        telemetry.update();
        
        waitForStart();

        // ========================================
        // STEP 8: MAIN CONTROL LOOP
        // ========================================
        while (opModeIsActive() && !isStopRequested()) {
            
            // Clear cached data from control hubs for fresh readings
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            
            // Update PedroPathing for heading and pose tracking only (not motor control)
            if (drive != null) {
                try {
                    drive.periodic();  // Update follower pose (odometry tracking)
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to update PedroPathing: " + e.getMessage());
                }
            }
            
            // ========================================
            // STEP 8.5: LIMELIGHT POSE CORRECTION
            // ========================================
            // Use Limelight AprilTag data to correct PedroPathing's pose
            if (limelight != null && drive != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    // Check if result is valid and has AprilTag detections
                    boolean hasValidTargets = result.isValid() && 
                        result.getFiducialResults() != null && 
                        !result.getFiducialResults().isEmpty();
                    
                    if (hasValidTargets) {
                        // Use AprilTag data to correct PedroPathing's pose
                        Pose correctedPose = getRobotPoseFromCamera(result);
                        if (correctedPose != null) {
                            try {
                                // Access PedroPathing follower directly to set pose
                                drive.follower.setPose(correctedPose);
                            } catch (Exception e) {
                                telemetry.addData("ERROR", "Failed to apply pose correction: " + e.getMessage());
                            }
                        }
                    }
                }
            }
            
            // ========================================
            // STEP 9: HANDLE IMU RESET (D-PAD DOWN)
            // ========================================
            // Reset IMU to 0 degrees when D-pad down is pressed
            // Use this if field-centric drive stops working correctly
            boolean dpadDownCurrentlyPressed = gamepad1.dpad_down;
            boolean dpadDownJustPressed = dpadDownCurrentlyPressed && !prevDpadDown;
            
            if (dpadDownJustPressed && drive != null) {
                try {
                    // Reset heading to 0° using PedroPathing Follower (avoids multiple drivers conflict)
                    drive.resetHeading(0.0);
                    telemetry.addData("IMU RESET", "Heading reset to 0° via PedroPathing (D-pad down pressed)");
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to reset heading: " + e.getMessage());
                }
            }
            prevDpadDown = dpadDownCurrentlyPressed;
            
            // ========================================
            // STEP 10: HANDLE AUTO-ALIGN (R1 TRIGGERED)
            // ========================================
            // Auto-align is triggered when R1 is pressed and Limelight sees AprilTag
            handleAutoAlign();
            
            // ========================================
            // STEP 11: HANDLE INTAKE CONTROLS
            // ========================================
            handleIntakeControls();
            
            // ========================================
            // STEP 12: HANDLE SHOOTER CONTROLS
            // ========================================
            handleShooterControls();
            
            // ========================================
            // STEP 13: GET JOYSTICK INPUT
            // ========================================
            // Read joystick values and apply deadzone (EXACT match to BLUE_DEC_TELEOP lines 580-582)
            double y = applyAdvancedDeadzone(-gamepad1.left_stick_y); // Forward/backward (negated - matches BLUE_DEC)
            double x = applyAdvancedDeadzone(gamepad1.left_stick_x);  // Strafing (NOT negated - matches BLUE_DEC)
            double rx = applyAdvancedDeadzone(gamepad1.right_stick_x); // Rotation (NOT negated - matches BLUE_DEC)
            
            // Override rotation with auto-align command if active
            if (autoAlignMode) {
                rx = autoAlignRotationCommand;
            }

            // Square inputs to make small movements smoother
            y = Math.copySign(y * y, y);
            x = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);

            // ========================================
            // STEP 14: MANUAL DRIVE CONTROL (MATCHING BLUE_DEC_TELEOP)
            // ========================================
            // Use manual motor control matching BLUE_DEC_TELEOP exactly
            // Field-centric rotation is handled INSIDE manualDriveControl (not here)
            manualDriveControl(x, y, rx);
            
            // Get current heading for telemetry
            double botHeading = 0.0;
            if (drive != null) {
                try {
                    // Get heading from PedroPathing's pose (more accurate than IMU alone)
                    botHeading = drive.getPose().getHeading();
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to get heading from PedroPathing: " + e.getMessage());
                }
            }
            
            // Get motor powers for telemetry (from actual motors)
            double flPower = (frontLeft != null) ? frontLeft.getPower() : 0.0;
            double frPower = (frontRight != null) ? frontRight.getPower() : 0.0;
            double blPower = (backLeft != null) ? backLeft.getPower() : 0.0;
            double brPower = (backRight != null) ? backRight.getPower() : 0.0;
            
            updateTelemetry(flPower, frPower, blPower, brPower, botHeading);
        }
        
        // ========================================
        // CLEANUP ON STOP
        // ========================================
        // Stop all motors when op mode stops
        stopAllMotors();
    }
    
    /**
     * Controls the intake motor
     * L1: Forward intake
     * L2: Reverse intake (spits out balls)
     * R1: Also runs intake when shooting
     */
    private void handleIntakeControls() {
        if (intakeMotor == null) {
            return;
        }
        
        // L1 button: Forward intake
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(INTAKE_FORWARD_POWER);
        }
        // L2 trigger: Reverse intake (spit out balls)
        else if (gamepad1.left_trigger > 0.1) {
            intakeMotor.setPower(-INTAKE_REVERSE_POWER);
        }
        // If shooter is feeding (R1 or R2 active), run intake to help feed ball
        else if (shooterState == ShooterState.FEEDING && (gamepad1.right_bumper || gamepad1.right_trigger > 0.1)) {
            intakeMotor.setPower(INTAKE_SHOOTER_POWER);
        }
        // No button pressed: Stop intake
        else {
            intakeMotor.setPower(0.0);
        }
    }
    
    /**
     * Auto-align: Rotates robot to center AprilTag in Limelight view
     * Starts when R1 is pressed and Limelight sees an AprilTag
     */
    private void handleAutoAlign() {
        // Start auto-align when R1 or R2 is pressed and Limelight sees AprilTag
        boolean r1CurrentlyPressed = gamepad1.right_bumper || gamepad1.right_trigger > 0.1;
        boolean r1JustPressed = r1CurrentlyPressed && !r1Pressed;
        
        if (r1JustPressed && !autoAlignMode) {
            startAutoAlign();
        }
        
        // Handle auto-align if active
        if (autoAlignMode) {
            // Check timeout
            if (autoAlignTimer.seconds() > AUTO_ALIGN_TIMEOUT) {
                stopAutoAlign("Timeout reached (" + String.format("%.1f", AUTO_ALIGN_TIMEOUT) + "s)");
                return;
            }
            
            // Check if shooter button is still pressed
            if (!r1CurrentlyPressed) {
                stopAutoAlign("Shooter button released");
                return;
            }
            
            // Get current Limelight data
            if (limelight == null) {
                stopAutoAlign("Limelight not available");
                return;
            }
            
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                // Don't stop auto-align, just wait for tag to appear
                autoAlignRotationCommand = 0.0;
                return;
            }
            
            // Read Limelight tx (horizontal offset in degrees)
            // tx < 0: Target to the left → rotate left (negative rotation)
            // tx > 0: Target to the right → rotate right (positive rotation)
            double tx = result.getTx();
            
            // Check if heading is aligned (within tolerance)
            if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
                // Aligned! Stop rotation but keep mode active
                autoAlignRotationCommand = 0.0;
                return;
            }
            
            // Calculate rotation speed based on tx value using proportional control
            // tx < 0 (target left) → negative rotation (rotate left)
            // tx > 0 (target right) → positive rotation (rotate right)
            // Direct mapping: rotation direction matches tx direction
            double rotationCommand = tx * AUTO_ALIGN_TX_GAIN;
            
            // Clamp rotation speed to maximum
            rotationCommand = Math.max(-AUTO_ALIGN_ROTATION_SPEED, Math.min(AUTO_ALIGN_ROTATION_SPEED, rotationCommand));
            
            // Enforce minimum rotation to ensure movement
            if (Math.abs(rotationCommand) < AUTO_ALIGN_MIN_ROTATE) {
                // Use tx sign to determine rotation direction if command is too small
                rotationCommand = Math.copySign(AUTO_ALIGN_MIN_ROTATE, tx);
            }
            
            // Store rotation command for use in drive control
            autoAlignRotationCommand = rotationCommand;
        }
    }
    
    /**
     * Start auto-align mode
     * Called when R1 is pressed and Limelight sees an AprilTag
     */
    private void startAutoAlign() {
        if (limelight == null) {
            telemetry.addData("AUTO-ALIGN", "ERROR: Limelight not available - cannot use auto-align");
            return;
        }
        
        // Get current Limelight result
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        
        // Check if already aligned
        double tx = result.getTx();
        if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
            autoAlignMode = true;
            autoAlignRotationCommand = 0.0;
            autoAlignTimer.reset();
            return;
        }
        
        // Initialize auto-align mode
        autoAlignMode = true;
        autoAlignTimer.reset();
    }
    
    /**
     * Stop auto-align mode
     */
    private void stopAutoAlign(String reason) {
        autoAlignMode = false;
        autoAlignRotationCommand = 0.0;
    }
    
    /**
     * Handle shooter system controls
     * R1: Trigger shooter - spins up motors, feeds ball when velocity reached, runs intake at 0.5 power
     * Auto-align is handled separately and runs concurrently with shooting
     */
    private void handleShooterControls() {
        if (shooterMotorLeft == null || shooterMotorRight == null) return;
        
        // Detect R1 (right bumper) or R2 (right trigger) button press (edge detection)
        boolean r1CurrentlyPressed = gamepad1.right_bumper;
        boolean r2CurrentlyPressed = gamepad1.right_trigger > 0.1;  // R2 trigger pressed
        boolean shooterButtonPressed = r1CurrentlyPressed || r2CurrentlyPressed;  // Either button pressed
        boolean shooterButtonJustPressed = shooterButtonPressed && !r1Pressed;
        
        // Determine feed motor power based on which button is pressed
        if (r1CurrentlyPressed) {
            currentFeedMotorPower = 0.8;  // R1 uses 0.8 power
        } else if (r2CurrentlyPressed) {
            currentFeedMotorPower = 0.4;  // R2 uses 0.4 power
        }
        
        // Update state machine based on shooter button and shooter state
        switch (shooterState) {
            case IDLE:
                if (shooterButtonJustPressed) {
                    // Calculate dynamic velocity based on Limelight distance
                    double targetVelocity = calculateDynamicVelocity();
                    currentCalculatedVelocity = targetVelocity;
                    
                    // Start spinning up shooters using velocity control
                    // Note: Auto-align runs concurrently, so shooting can start even while aligning
                    shooterState = ShooterState.SPIN_UP;
                    try {
                        shooterMotorLeft.setVelocity(targetVelocity);
                        shooterMotorRight.setVelocity(targetVelocity);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to start shooter motors: " + e.getMessage());
                        shooterState = ShooterState.IDLE;
                    }
                }
                break;
                
            case SPIN_UP:
                // Update velocity based on current distance (in case robot moved)
                double currentTargetVelocity = calculateDynamicVelocity();
                currentCalculatedVelocity = currentTargetVelocity;
                
                // Continuously set velocity to ensure motors maintain target (important for velocity control)
                try {
                    shooterMotorLeft.setVelocity(currentTargetVelocity);
                    shooterMotorRight.setVelocity(currentTargetVelocity);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to set shooter velocity: " + e.getMessage());
                }
                
                // Check if both motors have reached threshold velocity
                try {
                    double leftVel = Math.abs(shooterMotorLeft.getVelocity());
                    double rightVel = Math.abs(shooterMotorRight.getVelocity());
                    double effectiveTarget = currentTargetVelocity * SHOOTER_VELOCITY_THRESHOLD;
                    
                    if (leftVel >= effectiveTarget && rightVel >= effectiveTarget) {
                        shooterState = ShooterState.READY;
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to read shooter velocity: " + e.getMessage());
                }
                
                // If shooter button is released, stop shooters and auto-align
                if (!shooterButtonPressed) {
                    stopShooter();
                    stopAutoAlign("Shooter button released");
                }
                break;
                
            case READY:
                // Update velocity based on current distance (in case robot moved)
                double readyTargetVelocity = calculateDynamicVelocity();
                currentCalculatedVelocity = readyTargetVelocity;
                
                // Continuously maintain velocity while ready
                try {
                    shooterMotorLeft.setVelocity(readyTargetVelocity);
                    shooterMotorRight.setVelocity(readyTargetVelocity);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to maintain shooter velocity: " + e.getMessage());
                }
                
                // Start feeding when ready
                shooterState = ShooterState.FEEDING;
                
                // Start feed motor with power based on which button is pressed
                        if (feedMotor != null) {
                            feedMotor.setPower(currentFeedMotorPower);
                        }
                        
                // Start intake at reduced power (0.5) to feed ball to feeder
                // This pushes the ball into the chamber when first feed is triggered
                        if (intakeMotor != null) {
                            intakeMotor.setPower(INTAKE_SHOOTER_POWER);
                }
                break;
                
            case FEEDING:
                // Update velocity based on current distance (in case robot moved)
                double feedingTargetVelocity = calculateDynamicVelocity();
                currentCalculatedVelocity = feedingTargetVelocity;
                
                // Continuously maintain velocity while feeding
                try {
                    shooterMotorLeft.setVelocity(feedingTargetVelocity);
                    shooterMotorRight.setVelocity(feedingTargetVelocity);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to maintain shooter velocity: " + e.getMessage());
                }
                
                // Keep feeding while shooter button is held
                // Update feed power if button changes (R1 to R2 or vice versa)
                if (r1CurrentlyPressed) {
                    currentFeedMotorPower = 0.8;
                } else if (r2CurrentlyPressed) {
                    currentFeedMotorPower = 0.4;
                }
                
                // Update feed motor power in case it changed
                if (feedMotor != null) {
                    feedMotor.setPower(currentFeedMotorPower);
                }
                
                // Intake continues running at 0.5 power to push balls into chamber
                if (!shooterButtonPressed) {
                    // Shooter button released - stop everything and return to IDLE
                    stopFeeding();
                    stopShooter();
                    stopAutoAlign("Shooter button released");
                }
                // Note: Feed motor and intake continue running while in FEEDING state and R1 is held
                // Auto-align continues to run concurrently to keep robot aligned
                break;
        }
        
        // Update R1 pressed state for next iteration (track if any shooter button was pressed)
        r1Pressed = shooterButtonPressed;
    }
    
    /**
     * Stop shooter motors
     */
    private void stopShooter() {
        if (shooterMotorLeft != null) {
            try {
                shooterMotorLeft.setVelocity(0);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to stop left shooter: " + e.getMessage());
            }
        }
        if (shooterMotorRight != null) {
            try {
                shooterMotorRight.setVelocity(0);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to stop right shooter: " + e.getMessage());
            }
        }
        shooterState = ShooterState.IDLE;
    }
    
    /**
     * Stop feeding (feed motor and intake)
     * Note: Intake stopping is handled by handleIntakeControls() based on current state
     */
    private void stopFeeding() {
        if (feedMotor != null) {
            feedMotor.setPower(0.0);
        }
        // Note: Intake is controlled by handleIntakeControls() which runs every loop
        // It will stop the intake when shooterState is no longer FEEDING
    }
    
    /**
     * Calculate dynamic shooter velocity based on actual distance to AprilTag
     * Simplified linear model: Uses Limelight ta to calculate distance, then linear interpolation
     * 
     * Velocity Range:
     * - Short distance (36"): 1050 RPM - 100 RPM reduction = 950 RPM (for accuracy)
     * - Long distance (130"): 1450 RPM (linear interpolation)
     * 
     * How it works:
     * 1. Calculate base distance from Limelight ta (target area)
     * 2. Apply optional calibration multiplier if distance needs adjustment
     * 3. Clamp distance to working range [36", 130"]
     * 4. Linear interpolation: velocity = 1000 + (1400-1000) × normalizedDistance
     * 5. Apply close-range reduction (< 80") for better accuracy at short distances
     * 6. Clamp to safe limits [800, 1700] RPM
     * 
     * Tuning:
     * - DISTANCE_CALIBRATION_MULTIPLIER: Adjust if Limelight distance readings are consistently off
     * - SHORT_DISTANCE_VELOCITY: Base velocity at close range (36")
     * - LONG_DISTANCE_VELOCITY: Base velocity at far range (130")
     * - SHORT_DISTANCE_RPM_REDUCTION: How much to reduce for close shots (helps accuracy)
     * - REDUCTION_TAPER_DISTANCE: Distance where reduction stops (80")
     * 
     * @return Calculated target velocity in RPM
     */
    private double calculateDynamicVelocity() {
        // Fallback to default velocity if Limelight not available
        if (limelight == null) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
        // Get target area (ta) from Limelight to calculate distance
        double ta = result.getTa();
        if (ta <= 0.0) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
        // Step 1: Calculate base distance from target area
        // Formula: distance ∝ 1/√(ta) - inverse square root relationship
        double distanceCalibrationFactor = 50.0;
        double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
        
        // Step 2: Apply optional calibration multiplier (set to 1.0 if not needed)
        double actualDistance = baseDistance * DISTANCE_CALIBRATION_MULTIPLIER;
        
        // Step 2.5: Add 20" offset to correct for consistent measurement error
        actualDistance += 20.0;
        
        // Step 3: Clamp distance to working range
        actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, actualDistance));
        
        // Step 4: Normalize distance to 0.0 (close) to 1.0 (far) for linear interpolation
        double normalizedDistance = (actualDistance - SHORT_DISTANCE_INCHES) / 
            (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
        
        // Step 5: Linear interpolation between short and long distance velocities
        // At 36": normalizedDistance = 0.0 → velocity = 1050 RPM (after reduction: ~950 RPM)
        // At 130": normalizedDistance = 1.0 → velocity = 1450 RPM
        double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
            (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * normalizedDistance;
        
        // Step 6: Apply close-range reduction for better accuracy at short distances
        // Tapered reduction: 100 RPM at 36", 0 RPM at 80"
        if (actualDistance < REDUCTION_TAPER_DISTANCE) {
            double reductionFactor = 1.0 - ((actualDistance - SHORT_DISTANCE_INCHES) / 
                (REDUCTION_TAPER_DISTANCE - SHORT_DISTANCE_INCHES));
            reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor));
            calculatedVelocity -= SHORT_DISTANCE_RPM_REDUCTION * reductionFactor;
        }
        
        // Step 7: Clamp to safe velocity limits
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
    }
    
    /**
     * Stop all motors (cleanup)
     */
    private void stopAllMotors() {
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (feedMotor != null) feedMotor.setPower(0.0);
        stopShooter();
        
        // Stop drivetrain motors directly
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }
    
    // NOTE: manualDriveControl() method removed - now using MecanumDrive subsystem
    // The PedroPathing Follower handles all drive control automatically
    
    /**
     * Removes small joystick movements to prevent drift
     * Scales remaining input to keep full range
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
     * Handle manual drive control with field-oriented control (EXACT MATCH TO BLUE_DEC_TELEOP)
     * Uses PedroPathing for heading only, controls motors directly
     * Field-centric: Forward always moves toward field, regardless of robot orientation
     */
    private void manualDriveControl(double x, double y, double rx) {
        // Store values before rotation for debug
        fcYBefore = y;
        fcXBefore = x;
        
        // Get the robot's current heading from PedroPathing (NOT from IMU directly)
        double botHeading = 0.0;
        boolean headingAvailable = false;
        if (drive != null) {
            try {
                botHeading = drive.getPose().getHeading();
                fcRawHeading = botHeading;
                headingAvailable = true;
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to get heading from PedroPathing: " + e.getMessage());
                botHeading = 0.0;
                headingAvailable = false;
            }
        }

        // Convert joystick input to field coordinates (field-centric control)
        // This makes forward always move toward the field, not the robot's current direction
        if (headingAvailable && drive != null) {
            // Use EXACT formula from BLUE_DEC_TELEOP (line 1148-1150)
            // This is the standard field-centric rotation formula
            // Rotate joystick input based on robot's heading
            double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
            x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
            y = temp;
            
            // Apply strafe speed multiplier (only affects strafe, not forward/backward)
            x *= STRAFE_POWER_MULTIPLIER;
            
            fcHeadingUsed = botHeading;
            fcYAfter = y;
            fcXAfter = x;
            fcStatus = "ACTIVE";
            
            telemetry.addData("Field-Centric", "ACTIVE (Heading: " + String.format("%.1f°", Math.toDegrees(botHeading)) + ")");
        } else {
            // Robot-centric mode (fallback if PedroPathing not working)
            fcStatus = "DISABLED - PedroPathing not available";
            telemetry.addData("Field-Centric", "DISABLED - PedroPathing not available");
        }

        // Mecanum wheel calculations (EXACT match to BLUE_DEC_TELEOP lines 1163-1167)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        // Apply minimum power threshold (EXACT match to BLUE_DEC_TELEOP)
        flPower = applyMinPower(flPower);
        blPower = applyMinPower(blPower);
        frPower = applyMinPower(frPower);
        brPower = applyMinPower(brPower);

        // Apply power multiplier (EXACT match to BLUE_DEC_TELEOP)
        flPower *= DRIVE_POWER_MULTIPLIER;
        blPower *= DRIVE_POWER_MULTIPLIER;
        frPower *= DRIVE_POWER_MULTIPLIER;
        brPower *= DRIVE_POWER_MULTIPLIER;
        
        // Clamp power to maximum of 1.0 (EXACT match to BLUE_DEC_TELEOP)
        flPower = Math.max(-1.0, Math.min(1.0, flPower));
        blPower = Math.max(-1.0, Math.min(1.0, blPower));
        frPower = Math.max(-1.0, Math.min(1.0, frPower));
        brPower = Math.max(-1.0, Math.min(1.0, brPower));

        // Apply motor powers (EXACT match to BLUE_DEC_TELEOP lines 1188-1191)
        if (frontLeft != null) frontLeft.setPower(flPower);
        if (backLeft != null) backLeft.setPower(blPower);
        if (frontRight != null) frontRight.setPower(frPower);
        if (backRight != null) backRight.setPower(brPower);
    }
    
    /**
     * Convert Limelight AprilTag data to PedroPathing coordinates
     * Uses Limelight's botpose to correct PedroPathing's pose estimate
     * 
     * @param result Limelight result containing AprilTag data
     * @return PedroPathing Pose with corrected coordinates, or null if invalid
     */
    private Pose getRobotPoseFromCamera(LLResult result) {
        try {
            // Get robot pose from Limelight (in FTC coordinates)
            Pose3D botpose = result.getBotpose();
            
            // Check if pose data is valid
            if (botpose == null) {
                return null;
            }
            
            // Check if we have valid AprilTag detections
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return null;
            }
            
            // Parse Pose3D string representation to extract coordinates
            String poseString = botpose.toString();
            double x = 0.0, y = 0.0, heading = 0.0;
            
            try {
                // Extract x, y, and yaw (heading) from the pose string
                String[] parts = poseString.replaceAll("[{}]", "").split(",");
                for (String part : parts) {
                    String[] keyValue = part.split("=");
                    if (keyValue.length == 2) {
                        String key = keyValue[0].trim();
                        double value = Double.parseDouble(keyValue[1].trim());
                        
                        switch (key) {
                            case "x":
                                x = value;
                                break;
                            case "y":
                                y = value;
                                break;
                            case "yaw":
                                heading = value;
                                break;
                        }
                    }
                }
            } catch (Exception parseException) {
                telemetry.addData("ERROR", "Failed to parse Pose3D: " + parseException.getMessage());
                return null;
            }
            
            // Validate the coordinates
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading) ||
                Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(heading)) {
                return null;
            }
            
            // Check for reasonable field bounds
            if (Math.abs(x) > 200 || Math.abs(y) > 200) {
                return null;
            }
            
            // Create Pose in FTC coordinate system
            Pose ftcPose = new Pose(
                x,      // X position in inches
                y,      // Y position in inches  
                heading, // Heading in radians
                FTCCoordinates.INSTANCE
            );
            
            // Convert from FTC coordinates to PedroPathing coordinates
            Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            
            // Apply fusion with current PedroPathing pose for smoother corrections
            if (drive == null) {
                return pedroPose;
            }
            
            Pose currentPose = drive.getPose();
            
            // Distance check to prevent large jumps
            double distanceFromCurrent = Math.sqrt(
                Math.pow(pedroPose.getX() - currentPose.getX(), 2) + 
                Math.pow(pedroPose.getY() - currentPose.getY(), 2)
            );
            
            // If the vision correction is too far from current pose, reject it
            if (distanceFromCurrent > 50.0) { // 50 inch threshold
                return null;
            }
            
            // Use weighted average for smoother corrections (70% current, 30% vision)
            double fusionWeight = 0.3; // 30% vision, 70% current
            
            double fusedX = currentPose.getX() * (1 - fusionWeight) + pedroPose.getX() * fusionWeight;
            double fusedY = currentPose.getY() * (1 - fusionWeight) + pedroPose.getY() * fusionWeight;
            double fusedHeading = currentPose.getHeading() * (1 - fusionWeight) + pedroPose.getHeading() * fusionWeight;
            
            return new Pose(fusedX, fusedY, fusedHeading);
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to convert Limelight pose: " + e.getMessage());
            return null;
        }
    }

    // NOTE: applyMinPower() method removed - PedroPathing Follower handles power management automatically
    
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
        
        telemetry.addLine("=== RED JAN TELEOP ===");
        
        // Heading from PedroPathing Follower (manages IMU internally)
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(botHeading)));
        if (drive != null) {
            telemetry.addData("IMU Status", "Managed by PedroPathing");
            telemetry.addData("Field-Centric", "ENABLED (via PedroPathing)");
            Pose currentPose = drive.getPose();
            telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
        } else {
            telemetry.addData("IMU Status", "Drivetrain not initialized");
            telemetry.addData("Field-Centric", "DISABLED");
        }
        
        // Field-Centric Debug Telemetry (always show)
        telemetry.addLine("--- Field-Centric Debug ---");
        telemetry.addData("FC Status", fcStatus);
        telemetry.addData("FC Raw Heading", String.format("%.1f°", Math.toDegrees(fcRawHeading)));
        telemetry.addData("FC Heading Used", String.format("%.1f°", Math.toDegrees(fcHeadingUsed)));
        telemetry.addData("FC Input Before", String.format("y=%.2f, x=%.2f", fcYBefore, fcXBefore));
        telemetry.addData("FC Output After", String.format("y=%.2f, x=%.2f", fcYAfter, fcXAfter));
        
        // Intake status
        if (intakeMotor != null) {
            telemetry.addData("Intake Power", String.format("%.2f", intakeMotor.getPower()));
        }
        
        // Feed motor status
        if (feedMotor != null) {
            telemetry.addData("Feed Power", String.format("%.2f", feedMotor.getPower()));
        }
        
        // Auto-align status
        telemetry.addData("Auto-Align", autoAlignMode ? "ACTIVE" : "INACTIVE");
        
        // Limelight status and velocity calculation telemetry
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double ta = result.getTa();
                double tx = result.getTx();  // Horizontal offset for alignment
                double ty = result.getTy();  // Vertical offset (for reference)
                
                // Update RGB LED based on Limelight status
                // Orange: AprilTag is visible
                // Green: AprilTag is aligned (tx within ±10 degrees)
                // Red: No AprilTag visible (handled in else block)
                if (rgb != null) {
                    try {
                        double desiredColor = RGB_ORANGE;  // Default: Orange when tag is seen
                        
                        // Check if aligned: tx within ±10 degrees
                        if (Math.abs(tx) <= 10.0) {
                            desiredColor = RGB_GREEN;  // Green when aligned
                        }
                        
                        rgb.setPosition(desiredColor);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to set RGB LED: " + e.getMessage());
                    }
                }
                
                telemetry.addData("Limelight TX/TY", String.format("TX: %.2f° | TY: %.2f°", tx, ty) + 
                    (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE ? " [ALIGNED]" : ""));
                
                // Calculate distance from AprilTag to Limelight (matching simplified calculation)
                double distanceCalibrationFactor = 50.0;
                double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
                double actualDistance = baseDistance * DISTANCE_CALIBRATION_MULTIPLIER;
                actualDistance += 20.0;  // Add 20" offset to correct measurement error
                actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, actualDistance));
                
                // Show distance and calculated velocity
                telemetry.addData("Distance to AprilTag", String.format("%.1f\"", actualDistance));
                
                // Calculate velocity using simplified linear interpolation (matching actual calculation)
                double normalizedDistance = (actualDistance - SHORT_DISTANCE_INCHES) / 
                    (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
                double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
                    (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * normalizedDistance;
                
                // Apply close-range reduction for display (if applicable)
                if (actualDistance < REDUCTION_TAPER_DISTANCE) {
                    double reductionFactor = 1.0 - ((actualDistance - SHORT_DISTANCE_INCHES) / 
                        (REDUCTION_TAPER_DISTANCE - SHORT_DISTANCE_INCHES));
                    reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor));
                    double reduction = SHORT_DISTANCE_RPM_REDUCTION * reductionFactor;
                    calculatedVelocity -= reduction;
                }
                calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
                
                telemetry.addData("Calculated Velocity", String.format("%.0f RPM", calculatedVelocity));
            } else {
                // No valid target - set LED to RED
                if (rgb != null) {
                    try {
                        rgb.setPosition(RGB_RED);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to set RGB LED to RED: " + e.getMessage());
                    }
                }
                
                telemetry.addData("Limelight TX/TY", "NO TARGET");
                telemetry.addData("Distance to AprilTag", "N/A");
                telemetry.addData("Calculated Velocity", String.format("%.0f RPM (default)", SHOOTER_TARGET_VELOCITY));
            }
        } else {
            // Limelight not available - set LED to RED
            if (rgb != null) {
                try {
                    rgb.setPosition(RGB_RED);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to set RGB LED to RED: " + e.getMessage());
                }
            }
            
            telemetry.addData("Limelight TX/TY", "DISCONNECTED");
            telemetry.addData("Distance to AprilTag", "N/A");
            telemetry.addData("Calculated Velocity", String.format("%.0f RPM (default)", SHOOTER_TARGET_VELOCITY));
        }
        
        // Shooter status with detailed diagnostics
        telemetry.addData("Shooter State", shooterState.toString());
        if (shooterMotorLeft != null && shooterMotorRight != null) {
            try {
                double leftVel = Math.abs(shooterMotorLeft.getVelocity());
                double rightVel = Math.abs(shooterMotorRight.getVelocity());
                double leftPower = shooterMotorLeft.getPower();
                double rightPower = shooterMotorRight.getPower();
                double velocityPercent = (leftVel / currentCalculatedVelocity) * 100.0;
                
                telemetry.addData("Shooter L Vel", String.format("%.1f RPM", leftVel));
                telemetry.addData("Shooter R Vel", String.format("%.1f RPM", rightVel));
                telemetry.addData("Target Vel", String.format("%.0f RPM", currentCalculatedVelocity));
                telemetry.addData("Velocity %", String.format("%.1f%%", velocityPercent));
                
                // Warning if power is too low and velocity not reached
                if (leftPower < 0.7 && leftVel < currentCalculatedVelocity * 0.9 && shooterState == ShooterState.SPIN_UP) {
                    telemetry.addLine("WARNING: Power stuck at " + String.format("%.2f", leftPower) + " (F=" + String.format("%.0f", SHOOTER_PIDF_F) + ")!");
                    telemetry.addLine("Possible causes:");
                    telemetry.addLine("1. Motor controller power limit");
                    telemetry.addLine("2. Low battery voltage (< 12V)");
                    telemetry.addLine("3. Mechanical load/friction too high");
                    telemetry.addLine("4. Motor max RPM limit reached");
                    telemetry.addLine("Try: Check battery, reduce mechanical load, verify motor specs");
                }
                
                // Warning if velocity is stuck below target
                if (leftVel < currentCalculatedVelocity * 0.6 && shooterState == ShooterState.SPIN_UP) {
                    telemetry.addLine("WARNING: Velocity stuck below 60% of target!");
                    telemetry.addLine("Check: Motor power, PIDF tuning, mechanical load, battery voltage");
                }
            } catch (Exception e) {
                telemetry.addData("Shooter Error", e.getMessage());
            }
        }
        
        // Drivetrain status (from Follower)
        if (drive != null) {
            try {
                Pose currentPose = drive.getPose();
                Vector velocity = drive.getVelocity();
                telemetry.addLine("--- Drivetrain Status ---");
                telemetry.addData("Pose X", String.format("%.2f", currentPose.getX()));
                telemetry.addData("Pose Y", String.format("%.2f", currentPose.getY()));
                telemetry.addData("Pose Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
                telemetry.addData("Velocity", String.format("%.2f in/s", velocity.getMagnitude()));
            } catch (Exception e) {
                telemetry.addData("Drivetrain Error", e.getMessage());
            }
        } else {
        telemetry.addLine("--- Drive Powers ---");
        telemetry.addData("FL", String.format("%.2f", flPower));
        telemetry.addData("FR", String.format("%.2f", frPower));
        telemetry.addData("BL", String.format("%.2f", blPower));
        telemetry.addData("BR", String.format("%.2f", brPower));
        }
        
        telemetry.update();
    }
}
