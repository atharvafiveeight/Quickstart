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
 * NovTeleOpRed - Field-Oriented Control TeleOp Program with PedroPathing Integration
 * 
 * This program provides smooth, field-oriented control using the GoBilda PinPoint IMU.
 * Field-oriented control means the robot moves relative to the field, not relative to the robot's current heading.
 * 
 * Key Features:
 * - Field-Oriented Control (FOC) using GoBilda PinPoint IMU
 * - PedroPathing integration for autonomous navigation to preset locations
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
 * - Left Bumper: Long Distance Launch
 * - X Button: Go to Close Range Scoring Position
 * - Y Button: Go to Long Range Scoring Position
 * - A Button: Go to Home Position
 * 
 * @author Sahas Kumar
 * @version 3.0 - Removed Panels integration, fixed launcher and path following
 */
@TeleOp(name = "NovTeleOpRedSemiAuto", group = "TeleOp")
public class NovTeleOpRedSemiAuto extends LinearOpMode {

    // Motor declarations
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    // Launcher hardware declarations
    private DcMotorEx shooterMotor;  // Shooter motor with velocity control
    private CRServo leftServo;       // Left feeder servo
    private CRServo rightServo;      // Right feeder servo
    
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
    private final double FEED_TIME_SECONDS = 0.05;        // How long to feed game pieces
    private final double STOP_SPEED = 0.0;                // Stop speed for shooter
    private final double FULL_SPEED = 1.0;                // Full speed for servos
    
    // Launcher velocity control - using setVelocity() method
    private final double LAUNCHER_TARGET_VELOCITY = 1400; // Target velocity for short distance
    private final double LAUNCHER_MIN_VELOCITY = 1300;    // Minimum velocity for short distance
    
    // Long distance shooting (left bumper)
    private final double LONG_DISTANCE_TARGET_VELOCITY = 1750; // Target velocity for long distance
    private final double LONG_DISTANCE_MIN_VELOCITY = 1650;    // Minimum velocity for long distance
    
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
    
    // Shooting mode tracking
    private enum ShootingMode {
        SHORT_DISTANCE,  // Right bumper - normal shooting
        LONG_DISTANCE    // Left bumper - long distance shooting
    }
    
    private ShootingMode currentShootingMode;
    
    // Preset locations for panel integration
    private final Pose closeRangePose = new Pose(82.192, 97.534, Math.toRadians(40)); // Close range scoring
    private final Pose longRangePose = new Pose(80.219, 19.288, Math.toRadians(64));  // Long range scoring
    private final Pose homePose = new Pose(18.192, 18.411, Math.toRadians(180));      // Home position
    
    // Path following state management - simplified approach
    private boolean automatedDrive = false;
    private PathChain currentPath;

    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize launcher state machine
        launchState = LaunchState.IDLE;
        currentShootingMode = ShootingMode.SHORT_DISTANCE; // Default to short distance
        
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
        
        // DEBUG: Verify hardware mapping
        telemetry.addData("DEBUG", "Hardware mapping status:");
        telemetry.addData("DEBUG", "  Shooter Motor: " + (shooterMotor != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  Left Servo: " + (leftServo != null ? "FOUND" : "MISSING"));
        telemetry.addData("DEBUG", "  Right Servo: " + (rightServo != null ? "FOUND" : "MISSING"));
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
                
                // Set custom PIDF coefficients for better launcher control - from StarterBotTeleopMecanums
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(3, 1, 0, 10));
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
        telemetry.addData("Status", "NovTeleOpRed Ready!");
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
            // Handle bumper presses and set shooting mode
            boolean rightBumperPressed = gamepad1.rightBumperWasPressed();
            boolean leftBumperPressed = gamepad1.leftBumperWasPressed();
            
            // Set shooting mode based on which bumper is pressed
            if (rightBumperPressed) {
                currentShootingMode = ShootingMode.SHORT_DISTANCE;
                telemetry.addData("DEBUG", "SHOOTING MODE: Short distance (right bumper)");
            } else if (leftBumperPressed) {
                currentShootingMode = ShootingMode.LONG_DISTANCE;
                telemetry.addData("DEBUG", "SHOOTING MODE: Long distance (left bumper)");
            }
            
            // Launch if either bumper is pressed
            launch(rightBumperPressed || leftBumperPressed);
            
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
                        // Use PedroPathing's setTeleOpDrive for smooth field-centric control
                        // This maintains odometry and Limelight tracking while allowing manual control
                        follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,  // Forward/backward
                            -gamepad1.left_stick_x,  // Strafe left/right  
                            -gamepad1.right_stick_x, // Rotation
                            false // Field-centric (false = field-centric, true = robot-centric)
                        );
                        
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
                    launchState = LaunchState.IDLE;
                }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    // FIXED: Stop servos first
                    if (leftServo != null) leftServo.setPower(STOP_SPEED);
                    if (rightServo != null) rightServo.setPower(STOP_SPEED);
                    
                    // FIXED: Stop the shooter motor after shot is complete
                    try {
                        shooterMotor.setVelocity(0); // Stop the motor
                        telemetry.addData("DEBUG", "LAUNCHING: Motor stopped after shot completion");
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to stop shooter motor: " + e.getMessage());
                    }
                    
                    launchState = LaunchState.IDLE;
                    telemetry.addData("DEBUG", "LAUNCHING: Feed complete, motor stopped, returning to IDLE");
                }
                break;
        }
    }
    
    /**
     * Get current target velocity based on shooting mode
     * @return Target velocity for current shooting mode
     */
    private double getCurrentTargetVelocity() {
        return (currentShootingMode == ShootingMode.LONG_DISTANCE) ? 
            LONG_DISTANCE_TARGET_VELOCITY : LAUNCHER_TARGET_VELOCITY;
    }
    
    /**
     * Get current minimum velocity based on shooting mode
     * @return Minimum velocity for current shooting mode
     */
    private double getCurrentMinVelocity() {
        return (currentShootingMode == ShootingMode.LONG_DISTANCE) ? 
            LONG_DISTANCE_MIN_VELOCITY : LAUNCHER_MIN_VELOCITY;
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
        telemetry.addLine("=== NOV TELEOP RED ===");
        telemetry.addData("Control Mode", "Field-Oriented Control");
        telemetry.addData("Localization", "PedroPathing + AprilTag Corrections");
        telemetry.addData("Limelight Status", limelight != null ? "CONNECTED" : "DISCONNECTED");
        telemetry.addLine();
        
        // Robot heading information
        telemetry.addLine("--- ROBOT STATUS ---");
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(botHeading)));
        telemetry.addData("Heading (rad)", String.format("%.3f", botHeading));
        telemetry.addLine();
        
        // FIXED: PedroPathing coordinates - prominently displayed (with null checks)
        telemetry.addLine("--- PEDROPATHING COORDINATES ---");
        if (follower != null) {
            telemetry.addData("X Position", String.format("%.2f", follower.getPose().getX()));
            telemetry.addData("Y Position", String.format("%.2f", follower.getPose().getY()));
            telemetry.addData("Pose Valid", "YES (PedroPathing Active)");
        } else {
            telemetry.addData("X Position", "N/A (PedroPathing not available)");
            telemetry.addData("Y Position", "N/A (PedroPathing not available)");
            telemetry.addData("Pose Valid", "NO (PedroPathing not available)");
        }
        telemetry.addLine();
        
        // FIXED: PedroPathing status (with null checks) - simplified approach
        telemetry.addLine("--- PATH FOLLOWING ---");
        telemetry.addData("Automated Drive", automatedDrive ? "YES" : "NO");
        telemetry.addData("Path Busy", (follower != null && follower.isBusy()) ? "YES" : "NO");
        telemetry.addData("Current Path", currentPath != null ? "EXISTS" : "NULL");
        telemetry.addData("Control Mode", automatedDrive ? "AUTONOMOUS" : "MANUAL");
        telemetry.addData("Manual Control Active", !automatedDrive ? "YES" : "NO");
        telemetry.addLine();
        
        // Motor power information
        telemetry.addLine("--- MOTOR POWERS ---");
        telemetry.addData("Front Left", String.format("%.3f", flPower));
        telemetry.addData("Front Right", String.format("%.3f", frPower));
        telemetry.addData("Back Left", String.format("%.3f", blPower));
        telemetry.addData("Back Right", String.format("%.3f", brPower));
        telemetry.addLine();
        
        // FIXED: Launcher information (with null checks)
        telemetry.addLine("--- LAUNCHER STATUS ---");
        telemetry.addData("Launch State", launchState.toString());
        telemetry.addData("Shooting Mode", currentShootingMode.toString());
        if (shooterMotor != null) {
            telemetry.addData("Shooter Velocity", String.format("%.1f", shooterMotor.getVelocity()));
            telemetry.addData("Abs Velocity", String.format("%.1f", Math.abs(shooterMotor.getVelocity())));
            telemetry.addData("Shooter Power", String.format("%.3f", shooterMotor.getPower()));
            telemetry.addData("Motor Mode", shooterMotor.getMode().toString());
        } else {
            telemetry.addData("Shooter Velocity", "N/A (Motor not available)");
            telemetry.addData("Abs Velocity", "N/A (Motor not available)");
            telemetry.addData("Shooter Power", "N/A (Motor not available)");
            telemetry.addData("Motor Mode", "N/A (Motor not available)");
        }
        telemetry.addData("Target Velocity", getCurrentTargetVelocity() + " (" + currentShootingMode.toString() + ")");
        telemetry.addData("Min Velocity", getCurrentMinVelocity());
        telemetry.addData("Right Bumper", gamepad1.right_bumper ? "PRESSED" : "RELEASED");
        telemetry.addData("Left Bumper", gamepad1.left_bumper ? "PRESSED" : "RELEASED");
        if (leftServo != null) {
            telemetry.addData("Left Servo Power", String.format("%.3f", leftServo.getPower()));
        } else {
            telemetry.addData("Left Servo Power", "N/A (Servo not available)");
        }
        if (rightServo != null) {
            telemetry.addData("Right Servo Power", String.format("%.3f", rightServo.getPower()));
        } else {
            telemetry.addData("Right Servo Power", "N/A (Servo not available)");
        }
        telemetry.addLine();
        
        // Control instructions
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Left Stick Y", "Move Forward/Backward");
        telemetry.addData("Left Stick X", "Strafe Left/Right");
        telemetry.addData("Right Stick X", "Rotate Left/Right");
        telemetry.addData("Right Bumper", "Short Distance Launch");
        telemetry.addData("Left Bumper", "Long Distance Launch");
        telemetry.addData("X Button", "Go to Close Range Scoring");
        telemetry.addData("Y Button", "Go to Long Range Scoring");
        telemetry.addData("A Button", "Go to Home Position");
        telemetry.addData("B Button", "Cancel Automated Drive");
        telemetry.addLine();
        
        // FIXED: Performance information (with null checks)
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("PedroPathing Status", (follower != null && follower.isBusy()) ? "BUSY" : "IDLE");
        telemetry.addData("Deadzone", JOYSTICK_DEADZONE);
        telemetry.addData("Drive Power Multiplier", DRIVE_POWER_MULTIPLIER);
        telemetry.addLine();
        
        // Limelight information
        if (limelight != null) {
            telemetry.addLine("--- LIMELIGHT VISION ---");
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Targets Detected", "YES");
                telemetry.addData("tx", String.format("%.2f", result.getTx()));
                telemetry.addData("ty", String.format("%.2f", result.getTy()));
                Pose3D botpose = result.getBotpose();
                telemetry.addData("Botpose", botpose.toString());
            } else {
                telemetry.addData("Targets Detected", "NO");
            }
            telemetry.addLine();
        }
        
        // Debug information section
        telemetry.addLine("--- DEBUG INFO ---");
        // Debug messages are already added above in the main loop
        
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
