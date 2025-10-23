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

// PanelsConfigurables imports for PedroPathing visualization and configuration
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.List;

/**
 * NovTeleOpRed - Field-Oriented Control TeleOp Program with PedroPathing Panel Integration
 * 
 * This program provides smooth, field-oriented control using the GoBilda PinPoint IMU.
 * Field-oriented control means the robot moves relative to the field, not relative to the robot's current heading.
 * 
 * Key Features:
 * - Field-Oriented Control (FOC) using GoBilda PinPoint IMU
 * - PedroPathing integration for autonomous navigation to preset locations
 * - Panel integration with PanelsConfigurables for visualization
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
 * @version 2.0 - Added PedroPathing Panel Integration
 */
@Configurable
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
    private static final double DRIVE_POWER_MULTIPLIER = 0.8; // Power multiplier for speed control
    
    // Launcher constants - adjust these values for your robot
    private final double FEED_TIME_SECONDS = 0.05;        // How long to feed game pieces (reduced for single ball)
    private final double FEED_DELAY_SECONDS = 1.25;       // Delay between feeds for continuous shooting (1.25 second - synced with NovAutoRed)
    private final double FINAL_LAUNCH_DELAY = 2.00;       // Extra delay after last ball to ensure it launches
    private final double STOP_SPEED = 0.0;                // Stop speed for shooter
    private final double FULL_SPEED = 1.0;                // Full speed for servos
    
    // Short distance shooting (right bumper) - synced with NovAutoRed
    private final double LAUNCHER_TARGET_VELOCITY = 1225; // Target velocity for shooter (synced with NovAutoRed)
    private final double LAUNCHER_MIN_VELOCITY = 1225;     // Minimum velocity before launching (synced with NovAutoRed)
    private final double LAUNCHER_POWER = 0.8;            // Motor power for short distance
    
    // Long distance shooting (left bumper) - higher values
    private final double LONG_DISTANCE_TARGET_VELOCITY = 1600; // Higher velocity for long distance
    private final double LONG_DISTANCE_MIN_VELOCITY = 1550;    // Higher minimum velocity
    private final double LONG_DISTANCE_POWER = 0.95;           // Higher motor power for long distance
    
    // Configurable power variables for state machine (can be adjusted during initialization)
    private double spinUpPower = 0.8;                     // Power during spin-up phase
    private double launchingPower = 0.8;                  // Power during launching phase
    private double launchingPowerReduced = 0.7;           // Reduced power when at target velocity
    private double waitingPower = 0.8;                    // Power during waiting phase
    private double waitingPowerReduced = 0.7;             // Reduced power when at target velocity
    private double finalWaitPower = 0.8;                  // Power during final wait phase
    private double finalWaitPowerReduced = 0.7;           // Reduced power when at target velocity
    
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
    
    // Shooting mode tracking
    private enum ShootingMode {
        SHORT_DISTANCE,  // Right bumper - normal shooting
        LONG_DISTANCE    // Left bumper - long distance shooting
    }
    
    private LaunchState launchState;
    private ShootingMode currentShootingMode;
    private ElapsedTime feederTimer = new ElapsedTime();
    
    // Control variables for edge detection
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    
    // Preset locations for panel integration
    private final Pose closeRangePose = new Pose(82.192, 97.534, Math.toRadians(40)); // Close range scoring
    private final Pose longRangePose = new Pose(80.219, 19.288, Math.toRadians(64));  // Long range scoring
    private final Pose homePose = new Pose(18.192, 18.411, Math.toRadians(180));      // Home position
    
    // Path following state management
    private boolean isFollowingPath = false;
    private boolean joystickOverride = false;
    private PathChain currentPath;
    
    // Panels telemetry manager for PedroPathing visualization
    private TelemetryManager telemetryM;
    
    // Panels field for drawing
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style("", "#3F51B5", 0.0);
    private static final Style historyLook = new Style("", "#4CAF50", 0.0);
    private static final double ROBOT_RADIUS = 9; // Robot radius for drawing

    @Override
    public void runOpMode() throws InterruptedException {
        
        // Initialize launcher state machine
        launchState = LaunchState.IDLE;
        currentShootingMode = ShootingMode.SHORT_DISTANCE; // Default to short distance
        
        // Configure launcher power variables (can be adjusted here)
        configureLauncherPowers();
        
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
        
        // Initialize Limelight vision sensor
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.addData("DEBUG", "Limelight: " + (limelight != null ? "FOUND" : "MISSING"));
        } catch (Exception e) {
            telemetry.addData("ERROR", "Limelight initialization failed: " + e.getMessage());
        }

        // Set motor directions - same as MainOverdriveTeleOp
        // This ensures wheels spin in the correct direction
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set launcher motor and servo directions
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to brake when no power is applied - prevents drift
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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
        // STEP 3: INITIALIZE PEDROPATHING FOLLOWER
        // ========================================
        // Initialize PedroPathing Follower for autonomous path following
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize PanelsConfigurables for PedroPathing visualization
        PanelsConfigurables.INSTANCE.refreshClass(this);
        
        // Initialize Panels telemetry manager
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Set starting pose (robot position after autonomous)
        follower.setStartingPose(new Pose(81.096, 38.795, Math.toRadians(0)));
        
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
            
            // Update PedroPathing Follower (includes IMU and odometry updates)
            follower.update();
            
            // Draw robot position and path on Panels dashboard
            drawCurrentAndHistory();

            // ========================================
            // STEP 7: PANEL INTEGRATION - PRESET LOCATIONS
            // ========================================
            // Handle preset location buttons (X, Y, A)
            handlePresetLocationButtons();
            
            // Check for joystick override during path following
            checkJoystickOverride();

            // ========================================
            // STEP 8: PRE-WARM LAUNCHER SYSTEM
            // ========================================
            // Detect bumper presses and holds
            boolean rightBumperPressed = gamepad1.right_bumper && !prevRightBumper;
            boolean rightBumperHeld = gamepad1.right_bumper;
            boolean rightBumperReleased = !gamepad1.right_bumper && prevRightBumper;
            prevRightBumper = gamepad1.right_bumper;
            
            boolean leftBumperPressed = gamepad1.left_bumper && !prevLeftBumper;
            boolean leftBumperHeld = gamepad1.left_bumper;
            boolean leftBumperReleased = !gamepad1.left_bumper && prevLeftBumper;
            prevLeftBumper = gamepad1.left_bumper;
            
            // Determine which bumper is being used and set shooting mode
            boolean anyBumperPressed = rightBumperPressed || leftBumperPressed;
            boolean anyBumperHeld = rightBumperHeld || leftBumperHeld;
            boolean anyBumperReleased = rightBumperReleased || leftBumperReleased;
            
            // Set shooting mode based on which bumper is pressed
            if (rightBumperPressed) {
                currentShootingMode = ShootingMode.SHORT_DISTANCE;
                telemetry.addData("DEBUG", "SHOOTING MODE: Short distance (right bumper)");
            } else if (leftBumperPressed) {
                currentShootingMode = ShootingMode.LONG_DISTANCE;
                telemetry.addData("DEBUG", "SHOOTING MODE: Long distance (left bumper)");
            }
            
            // Stop shooter when not in use and in idle state
            if (!anyBumperHeld && launchState == LaunchState.IDLE) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(0);
                telemetry.addData("DEBUG", "STOPPING: Setting velocity to 0");
            }
            
            // Run the launch state machine
            launch(anyBumperPressed, anyBumperReleased, anyBumperHeld);
            
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
            // STEP 10: DRIVE CONTROL (PATH FOLLOWING OR MANUAL)
            // ========================================
            // Choose between path following or manual control
            if (isFollowingPath && !joystickOverride) {
                // Use PedroPathing for autonomous path following
                // The follower.update() call above handles the path following
                telemetry.addData("DEBUG", "Following path to preset location");
                
                // IMPORTANT: Don't call manualDriveControl when following path
                // This prevents joystick input from interfering with path following
            } else {
                // Use manual field-oriented control
                manualDriveControl(x, y, rx);
            }
        }
    }
    
    /**
     * Configure launcher power variables - adjust these values as needed
     */
    private void configureLauncherPowers() {
        // Default power settings (can be adjusted here)
        spinUpPower = 0.8;                     // Power during spin-up phase
        launchingPower = 0.8;                  // Power during launching phase
        launchingPowerReduced = 0.7;           // Reduced power when at target velocity
        waitingPower = 0.8;                    // Power during waiting phase
        waitingPowerReduced = 0.7;             // Reduced power when at target velocity
        finalWaitPower = 0.8;                  // Power during final wait phase
        finalWaitPowerReduced = 0.7;           // Reduced power when at target velocity
        
        telemetry.addData("DEBUG", "Launcher power configuration:");
        telemetry.addData("DEBUG", "  Spin-up: " + spinUpPower);
        telemetry.addData("DEBUG", "  Launching: " + launchingPower + " / " + launchingPowerReduced);
        telemetry.addData("DEBUG", "  Waiting: " + waitingPower + " / " + waitingPowerReduced);
        telemetry.addData("DEBUG", "  Final wait: " + finalWaitPower + " / " + finalWaitPowerReduced);
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
        // Send the calculated powers to each motor
        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);

        // ========================================
        // UPDATE TELEMETRY
        // ========================================
        // Show important information on the driver station screen
        updateTelemetry(flPower, frPower, blPower, brPower, botHeading);
    }
    
    /**
     * Handle preset location button presses (X, Y, A)
     */
    private void handlePresetLocationButtons() {
        // X button - Close Range Scoring Position
        if (gamepad1.x) {
            goToPresetLocation(closeRangePose, "Close Range Scoring");
        }
        // Y button - Long Range Scoring Position  
        else if (gamepad1.y) {
            goToPresetLocation(longRangePose, "Long Range Scoring");
        }
        // A button - Home Position
        else if (gamepad1.a) {
            goToPresetLocation(homePose, "Home Position");
        }
    }
    
    /**
     * Navigate to a preset location using PedroPathing
     */
    private void goToPresetLocation(Pose targetPose, String locationName) {
        // Always allow new destination selection, even if currently following a path
        // This resets the override and starts a new path
        
        // Create path from current position to target
        Pose currentPose = follower.getPose();
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();
        
        // Start following the path and reset override
        follower.followPath(currentPath);
        isFollowingPath = true;
        joystickOverride = false; // Reset override when new destination is selected
        
        telemetry.addData("DEBUG", "Navigating to " + locationName + " at (" + 
            String.format("%.1f", targetPose.getX()) + ", " + 
            String.format("%.1f", targetPose.getY()) + ") - Override reset");
    }
    
    /**
     * Check for joystick override during path following
     */
    private void checkJoystickOverride() {
        // Check if joysticks are being used (use raw values, not processed ones)
        double leftStickMagnitude = Math.sqrt(
            gamepad1.left_stick_x * gamepad1.left_stick_x + 
            gamepad1.left_stick_y * gamepad1.left_stick_y
        );
        double rightStickMagnitude = Math.abs(gamepad1.right_stick_x);
        
        // If joysticks are moved beyond deadzone, override path following
        if (leftStickMagnitude > JOYSTICK_DEADZONE || rightStickMagnitude > JOYSTICK_DEADZONE) {
            if (isFollowingPath && !joystickOverride) {
                joystickOverride = true;
                isFollowingPath = false; // Stop path following immediately
                telemetry.addData("DEBUG", "Joystick override activated - path stopped, manual control active");
            }
        }
        
        // Check if path is complete naturally (without joystick override)
        if (isFollowingPath && !follower.isBusy()) {
            isFollowingPath = false;
            // Don't set joystickOverride = true here - let it remain false for natural completion
            telemetry.addData("DEBUG", "Path following complete naturally - returning to manual control");
        }
    }
    
    /**
     * Launcher state machine - controls the shooting sequence
     * This method handles the complete launch process from spin-up to feeding
     * Now supports both short distance (right bumper) and long distance (left bumper) shooting
     * 
     * @param shotRequested True when any bumper is pressed (edge detected)
     * @param shotReleased True when any bumper is released (edge detected)
     * @param bumperHeld True when any bumper is currently held down
     */
    void launch(boolean shotRequested, boolean shotReleased, boolean bumperHeld) {
        // DEBUG: Log state machine transitions
        LaunchState previousState = launchState;
        
        // Get current target values based on shooting mode
        double currentTargetVelocity = getCurrentTargetVelocity();
        double currentMinVelocity = getCurrentMinVelocity();
        double currentPower = getCurrentPower();
        
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
                // Use power control based on shooting mode
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(currentPower);
                
                // DEBUG: Log detailed hardware status
                double currentVelocity = Math.abs(shooterMotor.getVelocity()); // Use absolute value
                double shooterPower = shooterMotor.getPower();
                telemetry.addData("DEBUG", "SPIN_UP: Motor mode: " + shooterMotor.getMode().toString());
                telemetry.addData("DEBUG", "SPIN_UP: Setting power to " + String.format("%.2f", currentPower) + 
                    " (" + currentShootingMode.toString() + ")");
                telemetry.addData("DEBUG", "SPIN_UP: Current velocity (abs): " + String.format("%.1f", currentVelocity));
                telemetry.addData("DEBUG", "SPIN_UP: Shooter power: " + String.format("%.3f", shooterPower));
                telemetry.addData("DEBUG", "SPIN_UP: Target velocity: " + currentTargetVelocity);
                telemetry.addData("DEBUG", "SPIN_UP: Min velocity: " + currentMinVelocity);
                
                // Check if velocity is close to target
                double maxAllowedVelocity = currentTargetVelocity + 50; // 50 more than target
                if (currentVelocity > maxAllowedVelocity) {
                    telemetry.addData("DEBUG", "WARNING: Velocity too high (" + String.format("%.1f", currentVelocity) + "), max allowed: " + maxAllowedVelocity);
                }
                
                // Trigger when velocity reaches target
                if (currentVelocity >= currentTargetVelocity) {
                    telemetry.addData("DEBUG", "SPIN_UP -> LAUNCH: Target velocity reached! (" + String.format("%.1f", currentVelocity) + " >= " + currentTargetVelocity + ")");
                    launchState = LaunchState.LAUNCH;
                } else {
                    telemetry.addData("DEBUG", "SPIN_UP: Waiting for target velocity... " + String.format("%.1f", currentVelocity) + " / " + currentTargetVelocity);
                }
                break;

            case LAUNCH:
                // Maintain power control while feeding
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(currentPower);
                
                // Start feeding the game pieces
                telemetry.addData("DEBUG", "LAUNCH -> LAUNCHING: Starting feeder servos");
                telemetry.addData("DEBUG", "LAUNCH: Setting left servo to " + FULL_SPEED);
                telemetry.addData("DEBUG", "LAUNCH: Setting right servo to " + FULL_SPEED);
                telemetry.addData("DEBUG", "LAUNCH: Maintaining power: " + String.format("%.2f", currentPower) + 
                    " (" + currentShootingMode.toString() + "), velocity: " + String.format("%.1f", Math.abs(shooterMotor.getVelocity())));
                
                leftServo.setPower(FULL_SPEED);
                rightServo.setPower(FULL_SPEED);
                
                // DEBUG: Check if servos are actually responding
                telemetry.addData("DEBUG", "LAUNCH: Left servo power reading: " + String.format("%.3f", leftServo.getPower()));
                telemetry.addData("DEBUG", "LAUNCH: Right servo power reading: " + String.format("%.3f", rightServo.getPower()));
                
                feederTimer.reset(); // Start timing the feed
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // Maintain power control while feeding with velocity limiting
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double launchingVelocity = Math.abs(shooterMotor.getVelocity());
                
                // Limit motor power based on velocity to prevent going beyond target
                if (launchingVelocity >= currentTargetVelocity) {
                    // Reduce power if velocity is at or above target
                    shooterMotor.setPower(launchingPowerReduced);
                } else {
                    // Use normal power if below target
                    shooterMotor.setPower(launchingPower);
                }
                
                // Continue feeding for the specified time
                double feedTime = feederTimer.seconds();
                telemetry.addData("DEBUG", "LAUNCHING: Feed time: " + String.format("%.2f", feedTime) + 
                                " / Target: " + FEED_TIME_SECONDS);
                telemetry.addData("DEBUG", "LAUNCHING: Velocity: " + String.format("%.1f", launchingVelocity) + 
                                " / Target: " + currentTargetVelocity + " (" + currentShootingMode.toString() + ")");
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
                // Maintain power control during continuous shooting with velocity limiting
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double continuousVelocity = Math.abs(shooterMotor.getVelocity());
                
                // Limit motor power based on velocity to prevent going beyond target
                if (continuousVelocity >= currentTargetVelocity) {
                    // Reduce power if velocity is at or above target
                    shooterMotor.setPower(waitingPowerReduced);
                    telemetry.addData("DEBUG", "CONTINUOUS: Velocity at target, reducing power to " + String.format("%.2f", waitingPowerReduced));
                } else {
                    // Use normal power if below target
                    shooterMotor.setPower(waitingPower);
                    telemetry.addData("DEBUG", "CONTINUOUS: Below target velocity, using " + String.format("%.2f", waitingPower) + " power");
                }
                
                // Continuous shooting mode - wait for velocity before each shot
                double continuousTime = feederTimer.seconds();
                telemetry.addData("DEBUG", "CONTINUOUS: Time since last feed: " + String.format("%.2f", continuousTime));
                telemetry.addData("DEBUG", "CONTINUOUS: Current velocity: " + String.format("%.1f", continuousVelocity));
                telemetry.addData("DEBUG", "CONTINUOUS: Target velocity: " + currentTargetVelocity + " (" + currentShootingMode.toString() + ")");
                telemetry.addData("DEBUG", "CONTINUOUS: Bumper held: " + bumperHeld);
                
                if (bumperHeld) {
                    // Check if it's time for next feed AND velocity is at target
                    if (continuousTime > FEED_DELAY_SECONDS && continuousVelocity >= currentTargetVelocity) {
                        // Time to feed another ball and velocity is ready
                        telemetry.addData("DEBUG", "CONTINUOUS -> LAUNCHING: Feeding next ball (velocity: " + String.format("%.1f", continuousVelocity) + ")");
                        leftServo.setPower(FULL_SPEED);
                        rightServo.setPower(FULL_SPEED);
                        feederTimer.reset(); // Start timing the feed
                        launchState = LaunchState.LAUNCHING;
                    } else if (continuousTime > FEED_DELAY_SECONDS) {
                        telemetry.addData("DEBUG", "CONTINUOUS: Waiting for target velocity... " + String.format("%.1f", continuousVelocity) + " / " + currentTargetVelocity);
                    } else {
                        telemetry.addData("DEBUG", "CONTINUOUS: Waiting for delay... " + String.format("%.2f", continuousTime) + " / " + FEED_DELAY_SECONDS);
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
     * Get current motor power based on shooting mode
     * @return Motor power for current shooting mode
     */
    private double getCurrentPower() {
        return (currentShootingMode == ShootingMode.LONG_DISTANCE) ? 
            LONG_DISTANCE_POWER : LAUNCHER_POWER;
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
        telemetry.addData("X Position", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Y Position", String.format("%.2f", follower.getPose().getY()));
        telemetry.addLine();
        
        // PedroPathing status
        telemetry.addLine("--- PATH FOLLOWING ---");
        telemetry.addData("Following Path", isFollowingPath ? "YES" : "NO");
        telemetry.addData("Joystick Override", joystickOverride ? "ACTIVE (Manual Mode)" : "INACTIVE");
        telemetry.addData("Path Busy", follower.isBusy() ? "YES" : "NO");
        telemetry.addData("Control Mode", (isFollowingPath && !joystickOverride) ? "AUTONOMOUS" : "MANUAL");
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
        telemetry.addData("Shooting Mode", currentShootingMode.toString());
        telemetry.addData("Shooter Velocity", String.format("%.1f", shooterMotor.getVelocity()));
        telemetry.addData("Target Velocity", getCurrentTargetVelocity() + " (" + currentShootingMode.toString() + ")");
        telemetry.addData("Min Velocity", getCurrentMinVelocity());
        telemetry.addData("Right Bumper", gamepad1.right_bumper ? "PRESSED" : "RELEASED");
        telemetry.addData("Left Bumper", gamepad1.left_bumper ? "PRESSED" : "RELEASED");
        telemetry.addData("Shooter Power", String.format("%.3f", shooterMotor.getPower()));
        telemetry.addData("Target Power", String.format("%.2f", getCurrentPower()) + " (" + currentShootingMode.toString() + ")");
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
        telemetry.addData("Right Bumper", "Short Distance Launch");
        telemetry.addData("Left Bumper", "Long Distance Launch");
        telemetry.addData("X Button", "Go to Close Range Scoring");
        telemetry.addData("Y Button", "Go to Long Range Scoring");
        telemetry.addData("A Button", "Go to Home Position");
        telemetry.addLine();
        
        // Performance information
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("PedroPathing Status", follower.isBusy() ? "BUSY" : "IDLE");
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
     * Draw current robot position and pose history on Panels dashboard
     */
    private void drawCurrentAndHistory() {
        try {
            // Set Panels field offsets for PedroPathing
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            
            // Draw pose history (robot's path)
            drawPoseHistory(follower.getPoseHistory());
            
            // Draw current robot position
            Pose currentPose = follower.getPose();
            drawRobot(currentPose);
            
            // Also draw a test robot at the starting position to verify drawing works
            Pose startPose = new Pose(81.096, 38.795, Math.toRadians(0));
            drawRobotAtPosition(startPose, "#FF0000"); // Red color for test robot
            
            // Debug: Show robot position in telemetry
            telemetry.addData("DEBUG", "Drawing robot at: (" + 
                String.format("%.1f", currentPose.getX()) + ", " + 
                String.format("%.1f", currentPose.getY()) + ") heading: " + 
                String.format("%.1f", Math.toDegrees(currentPose.getHeading())));
            
            // Send the drawing packet to Panels
            panelsField.update();
        } catch (Exception e) {
            // Handle drawing errors and show in telemetry
            telemetry.addData("DEBUG", "Drawing error: " + e.getMessage());
        }
    }
    
    /**
     * Draw robot at specified pose
     */
    private void drawRobot(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            telemetry.addData("DEBUG", "Invalid pose for drawing robot");
            return;
        }
        
        // Set style and draw robot circle
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);
        
        // Draw heading line using the same method as the original Drawing class
        com.pedropathing.math.Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();
        
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        
        telemetry.addData("DEBUG", "Robot drawn: circle at (" + 
            String.format("%.1f", pose.getX()) + ", " + 
            String.format("%.1f", pose.getY()) + ") radius: " + ROBOT_RADIUS);
    }
    
    /**
     * Draw pose history
     */
    private void drawPoseHistory(com.pedropathing.util.PoseHistory poseHistory) {
        panelsField.setStyle(historyLook);
        
        double[] xPositions = poseHistory.getXPositionsArray();
        double[] yPositions = poseHistory.getYPositionsArray();
        
        int pointsDrawn = 0;
        for (int i = 0; i < xPositions.length - 1; i++) {
            panelsField.moveCursor(xPositions[i], yPositions[i]);
            panelsField.line(xPositions[i + 1], yPositions[i + 1]);
            pointsDrawn++;
        }
        
        telemetry.addData("DEBUG", "Pose history drawn: " + pointsDrawn + " line segments");
    }
    
    /**
     * Draw robot at specified position with custom color
     */
    private void drawRobotAtPosition(Pose pose, String color) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }
        
        // Create custom style for this robot
        Style customStyle = new Style("", color, 0.0);
        
        // Set style and draw robot circle
        panelsField.setStyle(customStyle);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);
        
        // Draw heading line
        com.pedropathing.math.Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();
        
        panelsField.setStyle(customStyle);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        
        telemetry.addData("DEBUG", "Test robot drawn at: (" + 
            String.format("%.1f", pose.getX()) + ", " + 
            String.format("%.1f", pose.getY()) + ") color: " + color);
    }
    
    /**
     * Convert Limelight AprilTag data to PedroPathing coordinates
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
            
            // Log the raw 3D pose for debugging
            String poseString = botpose.toString();
            telemetry.addData("DEBUG", "Raw Limelight 3D pose: " + poseString);
            
            // Convert Pose3D to 2D coordinates for path planning
            // This follows FTC best practices: 3D measurement -> 2D path planning
            // Since toPose2d() method is not available, we'll parse the string representation
            
            // Parse the Pose3D string to extract coordinates
            // Format: "Pose3D{x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0}"
            double x = 0.0, y = 0.0, heading = 0.0;
            
            try {
                // Extract x, y, and yaw (heading) from the pose string
                // This is a simplified parser - you may need to adjust based on actual format
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
            
            // Validate the converted coordinates
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading) ||
                Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(heading)) {
                telemetry.addData("DEBUG", "Invalid converted 2D pose coordinates");
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
            
            // Use weighted average for smoother corrections (70% current, 30% vision)
            double fusionWeight = 0.3; // Adjust this value (0.0 = no vision, 1.0 = full vision)
            
            double fusedX = currentPose.getX() * (1 - fusionWeight) + pedroPose.getX() * fusionWeight;
            double fusedY = currentPose.getY() * (1 - fusionWeight) + pedroPose.getY() * fusionWeight;
            double fusedHeading = currentPose.getHeading() * (1 - fusionWeight) + pedroPose.getHeading() * fusionWeight;
            
            Pose fusedPose = new Pose(fusedX, fusedY, fusedHeading);
            
            telemetry.addData("DEBUG", "3D->2D conversion: (" + String.format("%.2f", x) + ", " + 
                String.format("%.2f", y) + ", " + String.format("%.1f°", Math.toDegrees(heading)) + ")");
            telemetry.addData("DEBUG", "Vision fusion: " + String.format("%.1f%% vision, %.1f%% current", 
                fusionWeight * 100, (1 - fusionWeight) * 100));
            
            return fusedPose;
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to convert Limelight pose: " + e.getMessage());
            return null;
        }
    }
}
