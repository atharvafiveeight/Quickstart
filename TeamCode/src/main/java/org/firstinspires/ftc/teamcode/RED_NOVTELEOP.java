package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Limelight vision sensor imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// PedroPathing imports
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// REMOVED: PanelsConfigurables imports - no longer needed

import java.util.List;

/**
 * RED_NOVTELEOP - Field-Oriented Control TeleOp Program with PedroPathing Integration
 * 
 * This program provides smooth, field-oriented control using the GoBilda PinPoint IMU.
 * Field-oriented control means the robot moves relative to the field, not relative to the robot's current heading.
 * 
 * Key Features:
 * - Field-Oriented Control (FOC) using GoBilda PinPoint IMU
 * - PedroPathing integration for autonomous navigation to preset locations
 * - Dynamic shooter velocity based on Limelight distance measurement
 * - L1 Scope mode for automatic alignment using Limelight camera
 * - Same motor setup as MainOverdriveTeleOp for consistency
 * - Anti-drift measures to prevent unwanted movement
 * - Bulk reading for faster performance
 * - Easy-to-understand code with lots of comments
 * 
 * Controls:
 * - Left Stick Y: Move forward/backward (field-relative)
 * - Left Stick X: Strafe left/right (field-relative)  
 * - Right Stick X: Rotate left/right
 * - Right Bumper (R1): Distance-based shot (velocity auto-adjusts based on distance)
 * - R2 Trigger: 3-Ball Pulsed Shot (Distance-based velocity)
 * - L1 Trigger: Scope Mode - Auto-align using Limelight camera
 * - X Button: Go to Close Range Scoring Position
 * - Y Button: Go to Long Range Scoring Position
 * - A Button: Go to Home Position
 * 
 * @author Sahas Kumar
 * @version 3.0 - Removed Panels integration, fixed launcher and path following
 */
@TeleOp(name = "RED_NOVTELEOP", group = "TeleOp")
public class RED_NOVTELEOP extends LinearOpMode {

    // Motor declarations
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    // Launcher hardware declarations
    private DcMotorEx shooterMotor;  // Shooter motor with velocity control
    private CRServo leftServo;       // Left feeder servo
    private CRServo rightServo;      // Right feeder servo
    private Servo rgb;               // RGB indicator (configured as servo)

    // RGB indicator positions (servo setPosition values)
    private static final double RGB_RED = 0.2777;
    private static final double RGB_GREEN = 0.5;
    private static final double RGB_ORANGE = 0.333;
    
    // Limelight vision sensor for localization
    private Limelight3A limelight;
    
    // PedroPathing Follower for autonomous path following
    private Follower follower;
    
    // Bulk reading setup for faster performance
    private List<LynxModule> allHubs;
    
    // Anti-drift constants - these help prevent unwanted movement
    private static final double JOYSTICK_DEADZONE = 0.10; // Reduced deadzone for more responsiveness
    private static final double MIN_MOTOR_POWER = 0.12;   // Increased minimum power for more torque
    private static final double DRIVE_POWER_MULTIPLIER = 0.95; // Power multiplier for speed control
    
    // Launcher constants - simplified from StarterBotTeleopMecanums
    private final double FEED_TIME_SECONDS = 0.1;        // How long to feed game pieces
    private final double STOP_SPEED = 0.0;                // Stop speed for shooter
    private final double FULL_SPEED = 1.0;                // Full speed for servos
    
    
    // Launcher velocity control - using setVelocity() method
    private final double LAUNCHER_TARGET_VELOCITY = 1400; // Target velocity for short distance
    private final double LAUNCHER_MIN_VELOCITY = 1350;    // Minimum velocity for short distance
    
    // Long distance velocity constants (used for calibration in distance-based calculation)
    private final double LONG_DISTANCE_TARGET_VELOCITY = 1750; // Target velocity for long distance (calibration point)
    private final double LONG_DISTANCE_MIN_VELOCITY = 1625;    // Minimum velocity for long distance (calibration point)
    
    // Distance-based velocity calculation using actual measured distance
    // Calibration points from actual measurements:
    private final double SHORT_DISTANCE_INCHES = 36.0;  // Actual distance for short shots (inches)
    private final double LONG_DISTANCE_INCHES = 114.0;  // Actual distance for long shots (inches)
    private final double SHORT_DISTANCE_VELOCITY = 1390.0; // Velocity at 36" (RPM) - adjusted from 1400
    private final double LONG_DISTANCE_VELOCITY = 1730.0;  // Velocity at 114" (RPM) - adjusted from 1750
    private final double MIN_VELOCITY = 1200;        // Minimum velocity for very close shots
    private final double MAX_VELOCITY = 1800;       // Maximum velocity for very far shots
    
    // Dynamic velocity calculation
    private double currentCalculatedVelocity = LAUNCHER_TARGET_VELOCITY; // Default to short distance
    private double currentCalculatedMinVelocity = LAUNCHER_MIN_VELOCITY; // Default to short distance
    
    // Field-oriented control is always enabled in this program
    private static final boolean FIELD_CENTRIC = true;

    // Launcher state machine - simplified from StarterBotTeleopMecanums
    private enum LaunchState {
        IDLE,        // Shooter is stopped, waiting for launch command
        SPIN_UP,     // Shooter is spinning up to target velocity
        LAUNCH,      // Ready to launch, start feeding
        LAUNCHING    // Currently feeding game pieces
    }
    
    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();
    
    // R2 pulsed feeding mode tracking
    private boolean r2ThreeSecondMode = false;
    private boolean r2Pressed = false;
    private ElapsedTime r2ServoTimer = new ElapsedTime();
    private boolean r2VelocityReached = false;
    
    // FIXED: Pulsed feeding constants for reliable 3-ball shooting
    private final double R2_BALL_FEED_TIME = 0.1; // Feed each ball for 0.05 seconds
    private final double R2_BALL_PAUSE_TIME = 1; // Pause between balls for 1.0 seconds
    private final double R2_MOTOR_STOP_TIME = 0.8; // Brief motor stop time for recovery (increased)
    private final int R2_BALL_COUNT = 3; // Shoot exactly 3 balls
    private int r2BallsShot = 0; // Track how many balls have been shot
    private boolean r2FeedingBall = false; // Track if currently feeding a ball
    private boolean r2WaitingForVelocity = false; // Track if waiting for velocity recovery
    private boolean r2MotorStopped = false; // Track if motor is briefly stopped for recovery
    
    // FIXED: Add timer to reduce velocity monitoring frequency
    private ElapsedTime r2VelocityCheckTimer = new ElapsedTime();
    
    // FIXED: R2 mode now uses same velocity logic as R1 mode instead of hardcoded offsets
    // R2 mode will trigger servos when velocity reaches the same threshold as R1 mode
    // This ensures consistent behavior between single shots and continuous shooting
    
    // L1 Scope mode - automatic alignment using Limelight (Ty-based)
    private boolean l1ScopeMode = false;
    private boolean l1Pressed = false;
    private ElapsedTime scopeTimer = new ElapsedTime();
    private double scopeRotationCommand = 0.0; // Rotation command from scope mode
    private final double SCOPE_ROTATION_SPEED = 0.35; // Slower auto-rotation for finer control
    private final double SCOPE_TY_TOLERANCE = 0.5; // ty tolerance in degrees (±0.5° = centered heading)
    private final double SCOPE_TIMEOUT = 2.0; // 2 second timeout for scope mode (reduced)
    private final double SCOPE_TY_GAIN = 0.14; // Lower gain to avoid overshoot

    // L1 Scope debug fields (for compact telemetry)
    private boolean scopeDebugActive = false;
    private String scopeDebugStatus = "";
    private double scopeDebugTy = Double.NaN;
    private double scopeDebugRotation = 0.0;
    private double scopeDebugTime = 0.0;
    
    
    // Shooting mode tracking (simplified - all shots use distance-based velocity)
    private enum ShootingMode {
        DISTANCE_BASED  // All shooting uses distance-based velocity
    }
    
    private ShootingMode currentShootingMode;
    
    // Preset locations for panel integration
    private final Pose closeRangePose = new Pose(82.192, 97.534, Math.toRadians(40)); // Close range scoring
    private final Pose longRangePose = new Pose(80.219, 19.288, Math.toRadians(65));  // Long range scoring
    private final Pose homePose = new Pose(18.192, 18.411, Math.toRadians(180));      // Home position
    
    // Path following state management - simplified approach
    private boolean automatedDrive = false;
    private PathChain currentPath;

    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize launcher state machine
        launchState = LaunchState.IDLE;
        currentShootingMode = ShootingMode.DISTANCE_BASED; // All shots use distance-based velocity
        
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
        // FIXED: Map motor names from hardware configuration to code variables with error handling
        boolean driveMotorsInitialized = true;
        
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            if (frontLeft == null) {
                telemetry.addData("ERROR", "Front left motor 'frontLeft' not found in hardware map!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize front left motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            if (frontRight == null) {
                telemetry.addData("ERROR", "Front right motor 'frontRight' not found in hardware map!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize front right motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            if (backLeft == null) {
                telemetry.addData("ERROR", "Back left motor 'backLeft' not found in hardware map!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize back left motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        try {
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");
            if (backRight == null) {
                telemetry.addData("ERROR", "Back right motor 'backRight' not found in hardware map!");
                driveMotorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize back right motor: " + e.getMessage());
            driveMotorsInitialized = false;
        }
        
        if (!driveMotorsInitialized) {
            telemetry.addData("CRITICAL ERROR", "Drive motors failed to initialize - robot cannot move!");
            // Note: We continue initialization but the robot won't be able to drive
        }
        
        // FIXED: Initialize launcher hardware with better error handling
        boolean hardwareInitialized = true;
        
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            if (shooterMotor == null) {
                telemetry.addData("ERROR", "Shooter motor 'shooterMotor' not found in hardware map!");
                hardwareInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor: " + e.getMessage());
            hardwareInitialized = false;
        }
        
        try {
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            if (leftServo == null) {
                telemetry.addData("ERROR", "Left servo 'leftServo' not found in hardware map!");
                hardwareInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize left servo: " + e.getMessage());
            hardwareInitialized = false;
        }
        
        try {
            rightServo = hardwareMap.get(CRServo.class, "rightServo");
            if (rightServo == null) {
                telemetry.addData("ERROR", "Right servo 'rightServo' not found in hardware map!");
                hardwareInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize right servo: " + e.getMessage());
            hardwareInitialized = false;
        }
        
        // Initialize RGB indicator servo (for Limelight state)
        try {
            rgb = hardwareMap.get(Servo.class, "rgb");
            if (rgb == null) {
                telemetry.addData("ERROR", "RGB indicator 'rgb' not found in hardware map!");
            } else {
                // Default to RED on init until we have a reading
                rgb.setPosition(RGB_RED);
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize RGB indicator: " + e.getMessage());
        }
        
        // DEBUG: Verify hardware mapping
        telemetry.addData("DEBUG", "Hardware mapping status:");
        telemetry.addData("DEBUG", "  Shooter Motor: " + (shooterMotor != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  Left Servo: " + (leftServo != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  Right Servo: " + (rightServo != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  RGB Indicator: " + (rgb != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  Overall Status: " + (hardwareInitialized ? "SUCCESS" : "FAILED"));
        
        if (!hardwareInitialized) {
            telemetry.addData("WARNING", "Some hardware failed to initialize - launcher may not work properly");
        }
        
        // Initialize Limelight vision sensor
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.addData("DEBUG", "Limelight: " + (limelight != null ? "FOUND" : "MISSING"));
        } catch (Exception e) {
            telemetry.addData("ERROR", "Limelight initialization failed: " + e.getMessage());
        }

        // FIXED: Set motor directions with null checks - same as MainOverdriveTeleOp
        // This ensures wheels spin in the correct direction
        if (frontRight != null) frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontLeft != null) frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // FIXED: Set launcher motor and servo directions with null checks
        if (shooterMotor != null) shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightServo != null) rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftServo != null) leftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // FIXED: Set motors to brake when no power is applied - prevents drift (with null checks)
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shooterMotor != null) shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // FIXED: Set up shooter motor with encoder-based velocity control (with null check)
        if (shooterMotor != null) {
            try {
                shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // Set custom PIDF coefficients for better launcher control
                // P=1.8 (proportional - stable), I=3 (integral - accurate), D=1.0 (derivative - smooth), F=8 (feedforward - conservative)
                // These values provide very stable velocity control for R2 mode
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(3, 0, 0, 10));
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to configure shooter motor: " + e.getMessage());
            }
        }
        
        // FIXED: Stop servos initially (with null checks)
        if (leftServo != null) leftServo.setPower(0);
        if (rightServo != null) rightServo.setPower(0);

        // ========================================
        // STEP 3: INITIALIZE PEDROPATHING FOLLOWER
        // ========================================
        // FIXED: Initialize PedroPathing Follower for autonomous path following with error handling
        try {
            follower = Constants.createFollower(hardwareMap);
            if (follower == null) {
                telemetry.addData("ERROR", "Failed to create PedroPathing follower!");
            } else {
                telemetry.addData("DEBUG", "PedroPathing follower created successfully");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "PedroPathing initialization failed: " + e.getMessage());
            follower = null;
        }
        
        // REMOVED: PanelsConfigurables initialization - no longer needed
        
        // FIXED: Set starting pose (robot position after autonomous) with null check
        // RED side: Heading 0° = facing away from driver (toward field) in PedroPathing coordinates
        // This aligns with IMU calibration where robot faces away from driver = 0°
        if (follower != null) {
            try {
                follower.setStartingPose(new Pose(81.096, 38.795, Math.toRadians(0)));
                telemetry.addData("DEBUG", "Starting pose set successfully");
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to set starting pose: " + e.getMessage());
            }
        }
        
        // ========================================
        // STEP 4: INITIALIZE LIMELIGHT VISION SENSOR
        // ========================================
        // Configure Limelight for MegaTag2 3D localization accuracy
        if (limelight != null) {
            telemetry.setMsTransmissionInterval(11);
            
            // MegaTag2 typically uses pipeline 0 for AprilTag detection
            // If you have custom pipelines, adjust this number accordingly
            limelight.pipelineSwitch(0); // Use pipeline 0 for MegaTag2
            
            // Start polling for data
            limelight.start();
            
            telemetry.addData("Status", "Limelight MegaTag2 initialized and started");
            telemetry.addData("Limelight Status", "READY");
            telemetry.addData("Pipeline", "0 (MegaTag2)");
            telemetry.addData("Note", "Stream/LED modes controlled by Limelight web interface");
        } else {
            telemetry.addData("ERROR", "Limelight not found - vision localization disabled");
        }

        // ========================================
        // STEP 5: READY TO START
        // ========================================
        telemetry.addData("Status", "RED_NOVTELEOP Ready!");
        telemetry.addData("Localization", "PedroPathing + Limelight Vision");
        telemetry.addData("Control Mode", "Field-Oriented Control");
        telemetry.addData("Instructions", "Use left stick to move, right stick X to rotate");
        telemetry.addData("Launcher", "Right bumper to launch");
        telemetry.update();
        
        waitForStart();

        // ========================================
        // STEP 6: MAIN CONTROL LOOP
        // ========================================
        while (opModeIsActive() && !isStopRequested()) {
            
            // Clear cached data from control hubs for fresh readings
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            
            // Get latest Limelight result for AprilTag-based localization corrections
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    // Check if result is valid and has AprilTag detections
                    boolean hasValidTargets = result.isValid() && 
                        result.getFiducialResults() != null && 
                        !result.getFiducialResults().isEmpty();
                    
                    if (hasValidTargets) {
                        // Use AprilTag data to correct PedroPathing's pose
                        // This maintains field-centric coordinates while improving accuracy
                        Pose correctedPose = getRobotPoseFromCamera(result);
                        if (correctedPose != null) {
                            follower.setPose(correctedPose);
                            telemetry.addData("DEBUG", "AprilTag correction applied: " + 
                                String.format("(%.2f, %.2f, %.1f°)", 
                                    correctedPose.getX(), correctedPose.getY(), 
                                    Math.toDegrees(correctedPose.getHeading())));
                        }
                        
                        // Debug: Log Limelight data
                        telemetry.addData("DEBUG", "Limelight: tx=" + String.format("%.2f", result.getTx()) + 
                            ", ty=" + String.format("%.2f", result.getTy()) + 
                            ", targets detected: YES (" + result.getFiducialResults().size() + " tags)");
                    } else {
                        // More detailed debugging for why no targets are detected
                        telemetry.addData("DEBUG", "Limelight: No valid AprilTags detected");
                        telemetry.addData("DEBUG", "  Result valid: " + (result != null ? result.isValid() : "null result"));
                        telemetry.addData("DEBUG", "  Fiducial results: " + (result != null && result.getFiducialResults() != null ? 
                            result.getFiducialResults().size() + " tags" : "null/empty"));
                        telemetry.addData("DEBUG", "  tx: " + (result != null ? String.format("%.2f", result.getTx()) : "N/A"));
                        telemetry.addData("DEBUG", "  ty: " + (result != null ? String.format("%.2f", result.getTy()) : "N/A"));
                    }
                } else {
                    telemetry.addData("DEBUG", "Limelight: No result received");
                }
            }
            
            // REMOVED: Panels drawing - no longer needed

            // ========================================
            // STEP 7: PANEL INTEGRATION - PRESET LOCATIONS
            // ========================================
            // Handle preset location buttons (X, Y, A)
            handlePresetLocationButtons();

            // ========================================
            // STEP 8: LAUNCHER SYSTEM
            // ========================================
            // Handle R1 button for distance-based shooting
            boolean rightBumperPressed = gamepad1.rightBumperWasPressed();
            
            // Handle R2 button for 3-shot mode
            boolean r2CurrentlyPressed = gamepad1.right_trigger > 0.1;
            boolean r2JustPressed = r2CurrentlyPressed && !r2Pressed;
            
            // Handle L1 button for scope mode
            boolean l1CurrentlyPressed = gamepad1.left_bumper;
            boolean l1JustPressed = l1CurrentlyPressed && !l1Pressed;
            
            // R1 (right bumper): Uses distance-based velocity for all shots
            if (rightBumperPressed) {
                currentShootingMode = ShootingMode.DISTANCE_BASED;
                telemetry.addData("DEBUG", "R1: Distance-based shot (velocity from distance)");
            }
            
            // Handle R2 3-second mode
            if (r2JustPressed) {
                startR2ThreeSecondMode();
            } else if (r2ThreeSecondMode) {
                handleR2ThreeSecondMode();
            }
            
            // Handle L1 scope mode
            if (l1JustPressed) {
                startL1ScopeMode();
            } else if (l1ScopeMode) {
                handleL1ScopeMode();
            }
            
            // Update pressed states
            r2Pressed = r2CurrentlyPressed;
            l1Pressed = l1CurrentlyPressed;
            
            // Launch if R1 is pressed (single shot with distance-based velocity)
            launch(rightBumperPressed);
            
            // ========================================
            // STEP 9: GET JOYSTICK INPUT
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
            // STEP 10: DRIVE CONTROL (SIMPLIFIED PATH FOLLOWING)
            // ========================================
            // FIXED: Update PedroPathing Follower with null check (includes IMU and odometry updates)
            // This is needed for localization even when not following a path
            if (follower != null) {
                try {
                    follower.update();
                    
                    // Debug: Show that PedroPathing is actively tracking position
                    telemetry.addData("DEBUG", "PedroPathing tracking: X=" + String.format("%.2f", follower.getPose().getX()) + 
                        ", Y=" + String.format("%.2f", follower.getPose().getY()) + 
                        ", Heading=" + String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
                    
                    // SIMPLIFIED: Manual teleop drive when not in automated mode
                    if (!automatedDrive) {
                        // Calculate rotation input - use scope mode if active, otherwise manual control
                        double rotationInput = -gamepad1.right_stick_x; // Default manual rotation

                        if (l1ScopeMode) {
                            // Scope mode: rotate in place using PedroPathing drive (zero X/Y, only rotation)
                            if (follower != null) {
                                follower.setTeleOpDrive(0.0, 0.0, scopeRotationCommand, false);
                            }
                        } else {
                            // Normal manual control uses right stick X
                            rotationInput = -gamepad1.right_stick_x;
                        }

                        // Use PedroPathing's setTeleOpDrive for smooth field-centric control
                        // This maintains odometry and Limelight tracking while allowing manual control
                        if (!l1ScopeMode) {
                            follower.setTeleOpDrive(
                                -gamepad1.left_stick_y,  // Forward/backward
                                -gamepad1.left_stick_x,  // Strafe left/right  
                                rotationInput,           // Rotation (manual control)
                                false // Field-centric (false = field-centric, true = robot-centric)
                            );
                        }
                        
                        // Update telemetry with motor powers from PedroPathing
                        double flPower = (frontLeft != null) ? frontLeft.getPower() : 0.0;
                        double frPower = (frontRight != null) ? frontRight.getPower() : 0.0;
                        double blPower = (backLeft != null) ? backLeft.getPower() : 0.0;
                        double brPower = (backRight != null) ? backRight.getPower() : 0.0;
                        updateTelemetry(flPower, frPower, blPower, brPower, follower.getPose().getHeading());
                        
                        telemetry.addData("DEBUG", "Manual teleop drive active - automatedDrive: " + automatedDrive);
                    } else {
                        // Automated path following is active
                        telemetry.addData("DEBUG", "Automated path following active - follower.isBusy: " + follower.isBusy());
                        
                        // Update telemetry with zero motor powers since PedroPathing is controlling the robot
                        updateTelemetry(0, 0, 0, 0, follower.getPose().getHeading());
                    }
                    
                    // Stop automated following if the follower is done or B button is pressed
                    if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                        follower.startTeleopDrive();
                        automatedDrive = false;
                        currentPath = null;
                        telemetry.addData("DEBUG", "Automated drive stopped - returning to manual control");
                    }
                    
                } catch (Exception e) {
                    telemetry.addData("ERROR", "PedroPathing update failed: " + e.getMessage());
                    // Fall back to manual control if PedroPathing fails
                    automatedDrive = false;
                    manualDriveControl(x, y, rx);
                    updateTelemetry(0, 0, 0, 0, 0);
                }
            } else {
                // FIXED: Fallback to manual control if PedroPathing is not available
                telemetry.addData("WARNING", "PedroPathing not available - using manual control only");
                manualDriveControl(x, y, rx);
                updateTelemetry(0, 0, 0, 0, 0);
            }
        }
    }
    
    
    /**
     * Handle manual drive control with field-oriented control
     */
    private void manualDriveControl(double x, double y, double rx) {
        // ========================================
        // FIELD-ORIENTED CONTROL CALCULATION
        // ========================================
        // This is the magic of field-oriented control!
        // We get the robot's current heading from PedroPathing's localizer (includes IMU)
        double botHeading = follower.getPose().getHeading();

        // Transform joystick input to field coordinates
        // This makes the robot move relative to the field, not the robot's current direction
        double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
        x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
        y = temp;

        // ========================================
        // MECANUM WHEEL CALCULATIONS
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

        // Apply power multiplier for more speed
        flPower *= DRIVE_POWER_MULTIPLIER;
        blPower *= DRIVE_POWER_MULTIPLIER;
        frPower *= DRIVE_POWER_MULTIPLIER;
        brPower *= DRIVE_POWER_MULTIPLIER;
        
        // Clamp power to maximum of 1.0 (100%)
        flPower = Math.max(-1.0, Math.min(1.0, flPower));
        blPower = Math.max(-1.0, Math.min(1.0, blPower));
        frPower = Math.max(-1.0, Math.min(1.0, frPower));
        brPower = Math.max(-1.0, Math.min(1.0, brPower));

        // ========================================
        // APPLY MOTOR POWERS
        // ========================================
        // FIXED: Send the calculated powers to each motor (with null checks)
        if (frontLeft != null) frontLeft.setPower(flPower);
        if (backLeft != null) backLeft.setPower(blPower);
        if (frontRight != null) frontRight.setPower(frPower);
        if (backRight != null) backRight.setPower(brPower);

        // ========================================
        // TELEMETRY IS NOW UPDATED IN MAIN LOOP
        // ========================================
        // Telemetry is updated in the main loop to ensure it's always shown
        // regardless of control mode (manual or autonomous)
    }
    
    /**
     * Handle preset location button presses (X, Y, A)
     * SIMPLIFIED: Uses PedroPathing's automated drive system
     */
    private void handlePresetLocationButtons() {
        // X button - Close Range Scoring Position (edge detection)
        if (gamepad1.xWasPressed()) {
            goToPresetLocation(closeRangePose, "Close Range Scoring");
        }
        // Y button - Long Range Scoring Position (edge detection)
        else if (gamepad1.yWasPressed()) {
            goToPresetLocation(longRangePose, "Long Range Scoring");
        }
        // A button - Home Position (edge detection)
        else if (gamepad1.aWasPressed()) {
            goToPresetLocation(homePose, "Home Position");
        }
    }
    
    /**
     * Navigate to a preset location using PedroPathing
     * SIMPLIFIED: Uses PedroPathing's automated drive system
     */
    private void goToPresetLocation(Pose targetPose, String locationName) {
        // Check if follower is available
        if (follower == null) {
            telemetry.addData("ERROR", "Cannot navigate to " + locationName + " - PedroPathing not available");
            return;
        }
        
        try {
            // Cancel any existing automated drive before starting a new one
            if (automatedDrive) {
                follower.startTeleopDrive();
                automatedDrive = false;
                currentPath = null;
                telemetry.addData("DEBUG", "Cancelled previous automated drive before starting new navigation");
            }
            
            // Create path from current position to target
            Pose currentPose = follower.getPose();
            currentPath = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                    .build();
            
            // Start automated path following
            follower.followPath(currentPath);
            automatedDrive = true;
            
            telemetry.addData("DEBUG", "Navigating to " + locationName + " at (" + 
                String.format("%.1f", targetPose.getX()) + ", " + 
                String.format("%.1f", targetPose.getY()) + ")");
            telemetry.addData("DEBUG", "Automated drive started - press B to cancel or wait for completion");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to navigate to " + locationName + ": " + e.getMessage());
            automatedDrive = false;
            currentPath = null;
        }
    }
    
    
    /**
     * Launcher state machine - FIXED: Proper shot completion and motor stopping
     * Uses setVelocity() method for better control
     * Supports both short and long distance shooting
     * 
     * @param shotRequested True when any bumper is pressed (edge detected)
     */
    void launch(boolean shotRequested) {
        // FIXED: Add null check for shooter motor
        if (shooterMotor == null) {
            telemetry.addData("ERROR", "Shooter motor not initialized - cannot launch");
            return;
        }
        
        // Get current target values based on shooting mode
        double currentTargetVelocity = getCurrentTargetVelocity();
        double currentMinVelocity = getCurrentMinVelocity();
        
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    feederTimer.reset(); // FIXED: Reset timer when entering SPIN_UP state
                    telemetry.addData("DEBUG", "Launch requested - entering SPIN_UP (" + currentShootingMode.toString() + ")");
                }
                break;
            case SPIN_UP:
                    // FIXED: Add error handling for setVelocity() call
                try {
                    shooterMotor.setVelocity(currentTargetVelocity);
                    double currentVel = Math.abs(shooterMotor.getVelocity()); // Use absolute value
                    telemetry.addData("DEBUG", "SPIN_UP: setVelocity(" + currentTargetVelocity + ") called (" + currentShootingMode.toString() + ")");
                    telemetry.addData("DEBUG", "SPIN_UP: Current velocity = " + shooterMotor.getVelocity() + " (abs: " + currentVel + ")");
                    telemetry.addData("DEBUG", "SPIN_UP: Need " + currentTargetVelocity + " to launch (min: " + currentMinVelocity + ")");
                    
                    // FIXED: Use a more reasonable target - 95% of target velocity to account for motor variations
                    double effectiveTargetVelocity = currentTargetVelocity * 0.95;
                    
                    // FIXED: Check if motor has reached effective target velocity for launching
                    if (currentVel >= effectiveTargetVelocity) {
                        launchState = LaunchState.LAUNCH;
                        telemetry.addData("DEBUG", "Effective target velocity reached (" + String.format("%.1f", currentVel) + " >= " + String.format("%.1f", effectiveTargetVelocity) + ") - entering LAUNCH");
                    }
                    // FIXED: Add timeout protection to prevent infinite spin-up (increased timeout)
                    else if (feederTimer.seconds() > 5.0) { // 5 second timeout
                        launchState = LaunchState.LAUNCH;
                        telemetry.addData("DEBUG", "Spin-up timeout - forcing launch (current: " + String.format("%.1f", currentVel) + ", target: " + String.format("%.1f", currentTargetVelocity) + ")");
                    } else {
                        telemetry.addData("DEBUG", "Still spinning up... (" + String.format("%.1f", currentVel) + "/" + String.format("%.1f", effectiveTargetVelocity) + " effective target)");
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "setVelocity() failed: " + e.getMessage());
                    launchState = LaunchState.IDLE; // Reset to idle on error
                }
                break;
            case LAUNCH:
                // FIXED: Add null checks for servos
                if (leftServo != null && rightServo != null) {
                    leftServo.setPower(FULL_SPEED);
                    rightServo.setPower(FULL_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                    telemetry.addData("DEBUG", "LAUNCH: Servos activated, entering LAUNCHING");
                } else {
                    telemetry.addData("ERROR", "Servos not initialized - cannot feed");
                    telemetry.addData("ERROR", "Servos not initialized - cannot feed");
                    launchState = LaunchState.IDLE;
                }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    // FIXED: Stop servos first
                    if (leftServo != null) leftServo.setPower(STOP_SPEED);
                    if (rightServo != null) rightServo.setPower(STOP_SPEED);
                    
                    // FIXED: Handle R2 mode motor control
                    if (r2ThreeSecondMode) {
                        // R2 mode - keep motor running during 3-second sequence
                        telemetry.addData("DEBUG", "LAUNCHING: R2 mode - keeping motor running during 3-second sequence");
                    } else {
                        // Normal single shot mode - stop motor
                        try {
                            shooterMotor.setVelocity(0); // Stop the motor
                            telemetry.addData("DEBUG", "LAUNCHING: Motor stopped after shot completion");
                        } catch (Exception e) {
                            telemetry.addData("ERROR", "Failed to stop shooter motor: " + e.getMessage());
                        }
                    }
                    
                    launchState = LaunchState.IDLE;
                    telemetry.addData("DEBUG", "LAUNCHING: Feed complete, returning to IDLE");
                }
                break;
        }
    }
    
    /**
     * Get current target velocity based on shooting mode and Limelight distance
     * @return Target velocity for current shooting mode and distance
     */
    private double getCurrentTargetVelocity() {
        // Always use dynamic velocity calculation based on Limelight distance
        return calculateDynamicVelocity();
    }
    
    /**
     * Get current minimum velocity based on shooting mode and Limelight distance
     * @return Minimum velocity for current shooting mode and distance
     */
    private double getCurrentMinVelocity() {
        // Always use dynamic minimum velocity calculation based on Limelight distance
        return currentCalculatedMinVelocity;
    }
    
    /**
     * Start R2 2-second continuous shot mode
     * Waits for target velocity, then runs servo for 2 seconds
     */
    private void startR2ThreeSecondMode() {
        // Stop any existing launcher activity
        if (launchState != LaunchState.IDLE) {
            launchState = LaunchState.IDLE;
        }
        
        // Initialize pulsed feeding mode
        r2ThreeSecondMode = true;
        r2VelocityReached = false;
        r2BallsShot = 0; // FIXED: Reset ball counter
        r2FeedingBall = false; // FIXED: Reset feeding state
        r2WaitingForVelocity = false; // FIXED: Reset velocity waiting state
        r2MotorStopped = false; // FIXED: Reset motor stopped state
        r2ServoTimer.reset();
        r2VelocityCheckTimer.reset(); // FIXED: Reset velocity check timer
        currentShootingMode = ShootingMode.DISTANCE_BASED;
        
        telemetry.addData("DEBUG", "R2 PULSED FEEDING: Mode started - waiting for trigger velocity (will shoot " + R2_BALL_COUNT + " balls)");
        
        // Start motor spinning up
        if (shooterMotor != null) {
            try {
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(getCurrentTargetVelocity());
                telemetry.addData("DEBUG", "R2 2-SECOND: Motor started at " + getCurrentTargetVelocity() + " velocity");
            } catch (Exception e) {
                telemetry.addData("ERROR", "R2 3-SECOND: Failed to start motor: " + e.getMessage());
                r2ThreeSecondMode = false;
            }
        }
    }
    
    /**
     * Handle R2 pulsed feeding mode
     * Shoots 3 balls with velocity checks and brief motor stop for recovery
     */
    private void handleR2ThreeSecondMode() {
        if (!r2ThreeSecondMode) return;

        // Only check velocity every 1 second to avoid PIDF fighting
        if (r2VelocityCheckTimer.seconds() >= 1.0) {
            r2VelocityCheckTimer.reset();
            if (shooterMotor != null) {
                double currentVel = Math.abs(shooterMotor.getVelocity());
                double targetVel = getCurrentTargetVelocity();
                double velocityTolerance = 100; // allow small droops
                if (currentVel < (targetVel - velocityTolerance)) {
                    try { shooterMotor.setVelocity(targetVel); } catch (Exception ignored) {}
                }
            }
        }

        // Wait for velocity lock before starting the sequence
        if (!r2VelocityReached) {
            double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
            double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
            if (currentVel >= effectiveTargetVelocity) {
                r2VelocityReached = true;
                r2ServoTimer.reset();
            }
            return;
        }

        // Pulsed feeding sequence for 3 balls with motor recovery
        if (r2BallsShot < R2_BALL_COUNT) {
                if (!r2FeedingBall && !r2WaitingForVelocity && !r2MotorStopped) {
                    // Extra wait before the 3rd shot to stabilize velocity
                    if (r2BallsShot == 2 && r2ServoTimer.seconds() < 0.35) {
                        // Wait additional 0.2s before starting final feed
                        return;
                    }
                // Check velocity before feeding next ball
                double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
                double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
                if (currentVel >= effectiveTargetVelocity) {
                    if (leftServo != null && rightServo != null) {
                        leftServo.setPower(FULL_SPEED);
                        rightServo.setPower(FULL_SPEED);
                        r2FeedingBall = true;
                        r2ServoTimer.reset();
                    } else {
                        stopR2ThreeSecondMode("Servo error");
                    }
                } else {
                    r2WaitingForVelocity = true;
                    r2ServoTimer.reset();
                }
            } else if (r2WaitingForVelocity) {
                // Wait until velocity recovers
                double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
                double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
                if (currentVel >= effectiveTargetVelocity) {
                    r2WaitingForVelocity = false;
                }
            } else if (r2FeedingBall) {
                // Feed current ball for fixed time
                if (r2ServoTimer.seconds() >= R2_BALL_FEED_TIME) {
                    if (leftServo != null) leftServo.setPower(STOP_SPEED);
                    if (rightServo != null) rightServo.setPower(STOP_SPEED);
                    r2FeedingBall = false;
                    r2BallsShot++;
                    r2ServoTimer.reset();
                    // Briefly stop motor for recovery if more balls remain
                    if (r2BallsShot < R2_BALL_COUNT) {
                        r2MotorStopped = true;
                        if (shooterMotor != null) {
                            try { shooterMotor.setVelocity(0); } catch (Exception ignored) {}
                        }
                    }
                }
            } else if (r2MotorStopped) {
                // After short stop, restart motor and wait a moment
                if (r2ServoTimer.seconds() >= R2_MOTOR_STOP_TIME) {
                    r2MotorStopped = false;
                    if (shooterMotor != null) {
                        try { shooterMotor.setVelocity(getCurrentTargetVelocity()); } catch (Exception ignored) {}
                    }
                    r2ServoTimer.reset();
                }
            }
        } else {
            // After last ball, pause briefly then stop
            if (r2ServoTimer.seconds() >= R2_BALL_PAUSE_TIME) {
                stopR2ThreeSecondMode("All balls shot");
            }
        }
    }
    
    /**
     * Stop R2 pulsed feeding mode
     */
    private void stopR2ThreeSecondMode(String reason) {
        r2ThreeSecondMode = false;
        r2VelocityReached = false;
        r2BallsShot = 0; // FIXED: Reset ball counter
        r2FeedingBall = false; // FIXED: Reset feeding state
        r2WaitingForVelocity = false; // FIXED: Reset velocity waiting state
        r2MotorStopped = false; // FIXED: Reset motor stopped state
        
        // Stop servos
        if (leftServo != null) leftServo.setPower(STOP_SPEED);
        if (rightServo != null) rightServo.setPower(STOP_SPEED);
        
        // Stop motor
        if (shooterMotor != null) {
            try {
                shooterMotor.setVelocity(0);
                telemetry.addData("DEBUG", "R2 PULSED: Motor stopped (" + reason + ")");
            } catch (Exception e) {
                telemetry.addData("ERROR", "R2 PULSED: Failed to stop motor: " + e.getMessage());
            }
        }
        
        // Reset launch state
        launchState = LaunchState.IDLE;
        
        telemetry.addData("DEBUG", "R2 PULSED: Mode stopped - " + reason);
    }
    
    /**
     * Start L1 scope mode - automatic alignment to center AprilTag in Limelight frame (Ty-based)
     */
    private void startL1ScopeMode() {
        if (limelight == null) {
            telemetry.addData("ERROR", "L1 SCOPE: Limelight not available - cannot use scope mode");
            scopeDebugActive = false;
            scopeDebugStatus = "ERROR: Limelight not available";
            return;
        }
        
        // Ensure manual control so rotation can be applied immediately
        automatedDrive = false;
        currentPath = null;
        telemetry.addData("DEBUG", "L1 SCOPE: Forcing manual control (automatedDrive=false)");
        
        // Get current Limelight result
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("ERROR", "L1 SCOPE: No valid Limelight data - cannot see AprilTag");
            telemetry.addData("ERROR", "L1 SCOPE: Make sure Limelight can see the AprilTag");
            scopeDebugActive = false;
            scopeDebugStatus = "ERROR: No valid AprilTag data";
            return;
        }
        
        // Check if heading is already aligned
        double ty = result.getTy();
        if (Math.abs(ty) <= SCOPE_TY_TOLERANCE) {
            telemetry.addData("DEBUG", "L1 SCOPE: Robot already aligned to AprilTag (ty=" + String.format("%.2f", ty) + "°)");
            l1ScopeMode = true; // Still activate mode to show it's ready
            scopeTimer.reset();
            scopeDebugActive = true;
            scopeDebugTy = ty;
            scopeDebugRotation = 0.0;
            scopeDebugTime = 0.0;
            scopeDebugStatus = "ALIGNED at start";
            return;
        }
        
        // Initialize scope mode
        l1ScopeMode = true;
        scopeTimer.reset();
        
        telemetry.addData("DEBUG", "L1 SCOPE: Mode started - aligning robot to AprilTag");
        telemetry.addData("DEBUG", "L1 SCOPE: Current ty=" + String.format("%.2f", ty) + 
            "° (target: 0.0°, tolerance: ±" + String.format("%.1f", SCOPE_TY_TOLERANCE) + "°)");
        telemetry.addData("DEBUG", "L1 SCOPE: ty<0 means target below center, ty>0 means target above center");

        scopeDebugActive = true;
        scopeDebugTy = ty;
        scopeDebugRotation = 0.0;
        scopeDebugTime = 0.0;
        scopeDebugStatus = "STARTED";
    }
    
    /**
     * Handle L1 scope mode - automatically rotate to center AprilTag in Limelight frame (Ty-based)
     */
    private void handleL1ScopeMode() {
        if (!l1ScopeMode) return;
        
        // Check timeout
        if (scopeTimer.seconds() > SCOPE_TIMEOUT) {
            stopL1ScopeMode("Timeout reached (" + String.format("%.1f", SCOPE_TIMEOUT) + "s)");
            return;
        }
        
        // Get current Limelight data
        if (limelight == null) {
            telemetry.addData("SCOPE", "ERROR: Limelight not available");
            return;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("SCOPE", "WARNING: No valid AprilTag detected - cannot center");
            telemetry.addData("SCOPE", "Make sure AprilTag is in view of Limelight camera");
            return;
        }
        
        // Read Limelight offsets
        // ty: vertical offset (degrees) – per mounting, yaw rotation affects ty directly
        // tx kept for possible future use but not used for rotation here
        double ty = result.getTy();
        
        // Check if heading is aligned (within tolerance)
        if (Math.abs(ty) <= SCOPE_TY_TOLERANCE) {
            stopL1ScopeMode("Robot aligned to AprilTag! (ty=" + String.format("%.2f", ty) + "°)");
            return;
        }
        
        // Calculate rotation speed based on ty value using proportional control
        // ty < 0: Target below → rotate one way
        // ty > 0: Target above → rotate the other way
        // Invert sign so rotation moves toward reducing ty (mount-dependent)
        double rotationCommand = -ty * SCOPE_TY_GAIN; // Proportional control toward center
        
        // Clamp rotation speed to maximum and enforce a small minimum to ensure movement
        rotationCommand = Math.max(-SCOPE_ROTATION_SPEED, Math.min(SCOPE_ROTATION_SPEED, rotationCommand));
        double minRotate = 0.05; // ensure robot actually rotates, but slower
        if (Math.abs(rotationCommand) < minRotate) {
            rotationCommand = Math.copySign(minRotate, rotationCommand == 0.0 ? ty : rotationCommand);
        }
        
        // Store rotation command for use in drive control
        // (This will be used by the drive control section)
        scopeRotationCommand = rotationCommand;
        
        // Update debug fields for compact telemetry
        scopeDebugActive = true;
        scopeDebugTy = ty;
        scopeDebugRotation = rotationCommand;
        scopeDebugTime = scopeTimer.seconds();
        scopeDebugStatus = "ALIGNING (ty)";
    }
    
    /**
     * Stop L1 scope mode
     */
    private void stopL1ScopeMode(String reason) {
        l1ScopeMode = false;
        scopeRotationCommand = 0.0; // Reset rotation command when stopping
        telemetry.addData("DEBUG", "L1 SCOPE: Mode stopped - " + reason);
    }
    
    /**
     * Normalize angle to [-π, π] range
     * @param angle Angle in radians
     * @return Normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
    
    /**
     * Calculate dynamic shooter velocity based on actual distance to AprilTag
     * Uses calculated distance from Limelight ta to interpolate between known velocity points
     * Calibration: 36" = 1400 RPM, 114" = 1750 RPM
     * @return Calculated target velocity
     */
    private double calculateDynamicVelocity() {
        if (limelight == null) {
            // Fallback to default short distance velocity if Limelight not available
            return SHORT_DISTANCE_VELOCITY;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // Fallback to default short distance velocity if no valid Limelight data
            return SHORT_DISTANCE_VELOCITY;
        }
        
        // Get the calculated distance from ta (uses the distance calculation we set up earlier)
        double ta = result.getTa();
        if (ta <= 0.0) {
            return SHORT_DISTANCE_VELOCITY;
        }
        
        // Calculate distance using the same method as the telemetry calibration section
        double distanceCalibrationFactor = 50.0;
        double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
        
        // Apply the same distance-dependent multiplier
        double appliedMultiplier = 1.0;
        if (baseDistance < 40.0) {
            appliedMultiplier = 1.424;
        } else if (baseDistance < 90.0) {
            double slope = -0.00212;
            appliedMultiplier = 1.42 + (baseDistance - 40.0) * slope;
        } else {
            double slope = -0.0030;
            appliedMultiplier = 1.333 + (baseDistance - 81.0) * slope;
            appliedMultiplier = Math.max(0.95, appliedMultiplier);
        }
        
        double actualDistance = baseDistance * appliedMultiplier;
        
        // Clamp distance to our known calibration range
        actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, actualDistance));
        
        // Calculate interpolation factor (0.0 = short distance, 1.0 = long distance)
        double interpolationFactor = (actualDistance - SHORT_DISTANCE_INCHES) / 
            (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
        
        // Interpolate between calibration points: 36" (1400 RPM) and 114" (1750 RPM)
        double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
            (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * interpolationFactor;
        
        // Clamp to our min/max velocity range
        calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
        
        // Calculate corresponding minimum velocity (same interpolation, 95% of target)
        double calculatedMinVelocity = calculatedVelocity * 0.95;
        calculatedMinVelocity = Math.max(MIN_VELOCITY * 0.95, Math.min(MAX_VELOCITY * 0.95, calculatedMinVelocity));
        
        // Update current calculated velocities
        currentCalculatedVelocity = calculatedVelocity;
        currentCalculatedMinVelocity = calculatedMinVelocity;
        
        return calculatedVelocity;
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
        
        // Minimal driving telemetry: Limelight status, distance, and shooter velocity
        telemetry.addLine("=== RED NOVTELEOP ===");
        telemetry.addData("Limelight", limelight != null ? "CONNECTED" : "DISCONNECTED");
        
        // PedroPathing pose (X, Y, Heading)
        if (follower != null) {
            telemetry.addData("X", String.format("%.2f", follower.getPose().getX()));
            telemetry.addData("Y", String.format("%.2f", follower.getPose().getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        }
        
        double distanceInches = 0.0;
        boolean tagVisible = false;
        double tyDisplay = Double.NaN;
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tagVisible = true;
                double ta = result.getTa();
                tyDisplay = result.getTy();
                // RGB indicator logic: default ORANGE when tag is seen
                if (rgb != null) {
                    try {
                        double desired = RGB_ORANGE;
                        if (Math.abs(tyDisplay) <= 1.0) desired = RGB_GREEN; // centered within ±1°
                        rgb.setPosition(desired);
                    } catch (Exception ignored) {}
                }
                if (ta > 0.0) {
                    // Same ta-based distance model with distance-dependent multiplier
                    double distanceCalibrationFactor = 50.0;
                    double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
                    double appliedMultiplier;
                    if (baseDistance < 40.0) {
                        appliedMultiplier = 1.424;
                    } else if (baseDistance < 90.0) {
                        double slope = -0.00212;
                        appliedMultiplier = 1.42 + (baseDistance - 40.0) * slope;
                    } else {
                        double slope = -0.0030;
                        appliedMultiplier = Math.max(0.95, 1.333 + (baseDistance - 81.0) * slope);
                    }
                    distanceInches = baseDistance * appliedMultiplier;
                }
            } else {
                // No valid detection – set RED
                if (rgb != null) {
                    try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {}
                }
            }
        } else {
            // Limelight not available - show RED
            if (rgb != null) {
                try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {}
            }
        }
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        if (!Double.isNaN(tyDisplay)) {
            telemetry.addData("Limelight ty", String.format("%.2f°", tyDisplay));
        } else {
            // No tag - show RED
            if (rgb != null) {
                try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {}
            }
        }
        telemetry.addData("Distance to Tag", String.format("%.1f in", distanceInches));

        // L1 scope debug (compact): show when scope was recently started/aligning/finished
        if (scopeDebugActive || l1ScopeMode) {
            telemetry.addLine("--- SCOPE DEBUG ---");
            telemetry.addData("Active", l1ScopeMode ? "YES" : "NO");
            if (!Double.isNaN(scopeDebugTy)) {
                telemetry.addData("ty", String.format("%.2f°", scopeDebugTy));
            }
            telemetry.addData("rot", String.format("%.2f", scopeDebugRotation));
            telemetry.addData("time", String.format("%.2fs", scopeDebugTime));
            telemetry.addData("status", scopeDebugStatus);
        }
        
        // Velocity tracking
        double targetVelocity = getCurrentTargetVelocity();
        telemetry.addData("Trigger Velocity", String.format("%.0f RPM", targetVelocity));
        if (shooterMotor != null) {
            telemetry.addData("Shooter Velocity", String.format("%.1f RPM", Math.abs(shooterMotor.getVelocity())));
        }
        
        telemetry.update();
    }

    /**
     * Convert Limelight AprilTag data to PedroPathing coordinates
     * FIXED: Uses proper API methods instead of fragile string parsing
     * This method maintains field-centric coordinates while applying vision corrections
     * 
     * Based on FTC best practices:
     * 1. Use Pose3d for initial 3D localization (position and orientation)
     * 2. Convert to Pose2d for 2D path planning
     * 3. The initial measurement must be 3D to be accurate
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
                telemetry.addData("DEBUG", "Limelight pose data is null");
                return null;
            }
            
            // Check if we have valid AprilTag detections
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                telemetry.addData("DEBUG", "No AprilTags detected for pose correction");
                return null;
            }
            
            // FIXED: Parse Pose3D string representation to extract coordinates
            // This is the most reliable way to get coordinates from Pose3D
            String poseString = botpose.toString();
            telemetry.addData("DEBUG", "Raw Limelight 3D pose string: " + poseString);
            
            // Parse the Pose3D string to extract coordinates
            // Format: "Pose3D{x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0}"
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
            
            // Log the parsed coordinates for debugging
            telemetry.addData("DEBUG", "Parsed coordinates: x=" + String.format("%.2f", x) + 
                ", y=" + String.format("%.2f", y) + ", yaw=" + String.format("%.2f", heading));
            
            // Validate the coordinates
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading) ||
                Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(heading)) {
                telemetry.addData("DEBUG", "Invalid 3D pose coordinates from Limelight");
                return null;
            }
            
            // FIXED: Add coordinate validation for reasonable field bounds
            // Typical FTC field is about 144" x 144", so check for reasonable values
            if (Math.abs(x) > 200 || Math.abs(y) > 200) {
                telemetry.addData("DEBUG", "Pose coordinates out of reasonable field bounds: (" + 
                    String.format("%.2f", x) + ", " + String.format("%.2f", y) + ")");
                return null;
            }
            
            // Create Pose in FTC coordinate system
            // Limelight returns pose in inches, which matches PedroPathing
            Pose ftcPose = new Pose(
                x,      // X position in inches
                y,      // Y position in inches  
                heading, // Heading in radians
                FTCCoordinates.INSTANCE
            );
            
            // Convert from FTC coordinates to PedroPathing coordinates
            // This ensures proper field-centric coordinate system
            Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            
            // Apply fusion with current PedroPathing pose for smoother corrections
            // This prevents sudden jumps when AprilTag corrections are applied
            Pose currentPose = follower.getPose();
            
            // FIXED: Add distance check to prevent large jumps
            double distanceFromCurrent = Math.sqrt(
                Math.pow(pedroPose.getX() - currentPose.getX(), 2) + 
                Math.pow(pedroPose.getY() - currentPose.getY(), 2)
            );
            
            // If the vision correction is too far from current pose, reject it
            if (distanceFromCurrent > 50.0) { // 50 inch threshold
                telemetry.addData("DEBUG", "Vision correction too far from current pose (" + 
                    String.format("%.2f", distanceFromCurrent) + " inches) - rejecting");
                return null;
            }
            
            // Use weighted average for smoother corrections (70% current, 30% vision)
            double fusionWeight = 1.0; // Adjust this value (0.0 = no vision, 1.0 = full vision)
            
            double fusedX = currentPose.getX() * (1 - fusionWeight) + pedroPose.getX() * fusionWeight;
            double fusedY = currentPose.getY() * (1 - fusionWeight) + pedroPose.getY() * fusionWeight;
            double fusedHeading = currentPose.getHeading() * (1 - fusionWeight) + pedroPose.getHeading() * fusionWeight;
            
            Pose fusedPose = new Pose(fusedX, fusedY, fusedHeading);
            
            telemetry.addData("DEBUG", "3D->2D conversion: (" + String.format("%.2f", x) + ", " + 
                String.format("%.2f", y) + ", " + String.format("%.1f°", Math.toDegrees(heading)) + ")");
            telemetry.addData("DEBUG", "Vision fusion: " + String.format("%.1f%% vision, %.1f%% current", 
                fusionWeight * 100, (1 - fusionWeight) * 100));
            telemetry.addData("DEBUG", "Distance from current: " + String.format("%.2f", distanceFromCurrent) + " inches");
            
            return fusedPose;
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to convert Limelight pose: " + e.getMessage());
            return null;
        }
    }
}
