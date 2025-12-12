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

import java.util.List;

/**
 * BLUE_DEC_TELEOP - Field-Oriented Control TeleOp Program for December Robot
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
 * @version 1.0 - Initial December robot implementation
 */
@TeleOp(name = "BLUE_DEC_TELEOP", group = "TeleOp")
public class BLUE_DEC_TELEOP extends LinearOpMode {

    // ========================================
    // DRIVETRAIN MOTORS
    // ========================================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
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
    private GoBildaPinpointDriver odo;   // GoBilda PinPoint IMU ("odo") for heading and odometry
    
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
    private static final double DRIVE_POWER_MULTIPLIER = 0.95;  // Overall speed control (0.95 = 95% speed)
    
    // Intake Motor Constants
    private static final double INTAKE_FORWARD_POWER = 0.95;  // Power when L1 button pressed (forward)
    private static final double INTAKE_REVERSE_POWER = 0.85;  // Power when L2 trigger pressed (reverse)
    private static final double INTAKE_SHOOTER_POWER = 0.85;   // Power when R1 pressed (helps feed ball)
    
    // Feed Motor Constants - now dynamic based on R1 (0.8) or R2 (0.5)
    // Feed motor power is set dynamically: R1 (right bumper) = 0.8, R2 (right trigger) = 0.5
    
    // Shooter Distance Constants
    private static final double SHORT_DISTANCE_INCHES = 36.0;   // Closest shooting distance (inches)
    private static final double LONG_DISTANCE_INCHES = 130.0;   // Farthest shooting distance (inches)
    private static final double SHORT_DISTANCE_VELOCITY = 1000.0;  // Shooter speed at close distance (RPM)
    private static final double LONG_DISTANCE_VELOCITY = 1500.0;   // Shooter speed at far distance (RPM) - Increased by 50 RPM
    private static final double MIN_VELOCITY = 800.0;   // Slowest allowed shooter speed (RPM)
    private static final double MAX_VELOCITY = 1700.0;  // Fastest allowed shooter speed (RPM)
    
    // Shooter Velocity Adjustments
    private static final double SHORT_DISTANCE_RPM_REDUCTION = 100.0;  // Reduce speed by this much for close shots (RPM)
    private static final double REDUCTION_TAPER_DISTANCE = 80.0;       // Distance where reduction stops (inches)
    private static final double LONG_DISTANCE_RPM_INCREASE = 100.0;    // Add this much speed for far shots (RPM)
    private static final double INCREASE_START_DISTANCE = 114.0;       // Distance where increase starts (inches)
    
    // Distance-specific corrections (based on testing)
    private static final double CORRECTION_DISTANCE_CENTER = 117.0;     // Distance where correction is applied (inches)
    private static final double CORRECTION_DISTANCE_RANGE = 2.0;        // Range around center distance (±inches)
    private static final double CORRECTION_RPM_REDUCTION = 25.0;       // RPM to reduce at correction distance
    
    // Shooter Velocity Curve (Polynomial)
    // These numbers control how shooter speed changes between close and far distances
    // Formula: speed = SHORT_VELOCITY + (LONG_VELOCITY - SHORT_VELOCITY) * curve
    // Default values make a straight line (linear)
    private static final double POLY_COEFF_A = 0.0;  // Cubic term (x^3)
    private static final double POLY_COEFF_B = 0.0;  // Quadratic term (x^2)
    private static final double POLY_COEFF_C = 1.0;  // Linear term (x)
    private static final double POLY_COEFF_D = 0.0;  // Constant term
    
    // Shooter Control Constants
    private static final double SHOOTER_TARGET_VELOCITY = 1475.0;  // Default speed when Limelight not working (RPM)
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
    private double currentFeedMotorPower = 0.5;  // Current feed motor power (set by R1=0.8 or R2=0.5)
    
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
        // STEP 2: INITIALIZE DRIVETRAIN MOTORS
        // ========================================
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
        }
        
        // Set motor directions - same as RED_DEC_TELEOP
        if (frontRight != null) frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontLeft != null) frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motors to FLOAT (coast) when no power is applied - allows robot to coast with inertia
        // Changed from BRAKE to prevent robot from lifting when stopping
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
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
                telemetry.addData("DEBUG", "Intake motor initialized successfully");
                telemetry.addData("DEBUG", "Intake motor name: intakeMotor");
                telemetry.addData("DEBUG", "Intake motor direction: REVERSE");
                telemetry.addData("DEBUG", "Intake motor mode: RUN_WITHOUT_ENCODER");
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
                telemetry.addData("DEBUG", "Feed motor initialized successfully");
                telemetry.addData("DEBUG", "Feed motor direction: REVERSE");
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
                
                telemetry.addData("DEBUG", "Shooter motors initialized with velocity control");
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to configure shooter motors: " + e.getMessage());
                shooterInitialized = false;
            }
        }
        
        // ========================================
        // STEP 6: INITIALIZE IMU FOR FIELD-CENTRIC CONTROL
        // ========================================
        // Initialize IMU to know which direction robot is facing
        // Configuration matches Constants.java for consistency
        boolean imuInitialized = true;
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            if (odo == null) {
                telemetry.addData("ERROR", "PinPoint IMU 'odo' not found in hardware map!");
                imuInitialized = false;
            } else {
                // Configure odometry pods similar to Constants.java
                // Note: These values match Constants.localizerConstants configuration
                try {
                    // Set odometry pod offsets (matching Constants.java values)
                    // forwardPodY = -3.622 inches, strafePodX = -3.976 inches
                    odo.setOffsets(-3.976, -3.622, DistanceUnit.INCH);
                    
                    // Set encoder resolution for 4-bar pod
                    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                    
                    // Set encoder directions (matching Constants.java)
                    // Forward encoder: REVERSED, Strafe encoder: FORWARD
                    odo.setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED,  // Forward encoder
                        GoBildaPinpointDriver.EncoderDirection.FORWARD      // Strafe encoder
                    );
                    
                    telemetry.addData("DEBUG", "PinPoint odometry pods configured");
                } catch (Exception e) {
                    telemetry.addData("WARNING", "Odometry pod configuration failed (may not have pods): " + e.getMessage());
                    telemetry.addData("WARNING", "Continuing with IMU-only mode (heading only)");
                }
                
                // Reset IMU to 0 degrees (robot must be stationary)
                odo.resetPosAndIMU();
                
                // Wait for IMU to be ready
                telemetry.addData("Status", "Initializing PinPoint IMU (BLUE side - resetting to 0°)...");
                telemetry.addData("IMU Reset", "Robot should be facing away from driver (toward field)");
                telemetry.update();
                
                // Wait for IMU to be ready (check device status)
                while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY && opModeIsActive()) {
                    telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
                    telemetry.addData("Waiting", "Please wait for IMU to be ready...");
                    telemetry.update();
                    sleep(50);
                }
                
                // Verify heading is reset to 0° (or close to it)
                try {
                    double initialHeading = odo.getHeading(AngleUnit.DEGREES);
                    telemetry.addData("DEBUG", "PinPoint IMU initialized successfully");
                    telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
                    telemetry.addData("Initial Heading", String.format("%.1f°", initialHeading) + " (should be ~0° for BLUE side)");
                } catch (Exception e) {
                    telemetry.addData("DEBUG", "PinPoint IMU initialized (heading read failed: " + e.getMessage() + ")");
                    telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
                }
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "PinPoint IMU initialization failed: " + e.getMessage());
            imuInitialized = false;
            odo = null;
        }
        
        if (!imuInitialized) {
            telemetry.addData("WARNING", "PinPoint IMU not available - field-centric control disabled");
        }
        
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
                
                telemetry.addData("DEBUG", "Limelight initialized successfully");
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
                telemetry.addData("DEBUG", "RGB LED initialized successfully");
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
        telemetry.addData("Status", "BLUE_DEC_TELEOP Ready!");
        telemetry.addData("Localization", "IMU Field-Centric (no odometry)");
        telemetry.addData("Control Mode", "Field-Oriented Control");
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
            
            // Update IMU to get current robot heading
            // Using full update() to get both heading and odometry position (if pods are configured)
            if (odo != null) {
                try {
                    // Update full odometry (heading + position if pods available)
                    // If pods not available, this still updates heading
                    odo.update();
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to update PinPoint IMU: " + e.getMessage());
                }
            }
            
            // Update Limelight data for distance-based velocity calculation
            // Limelight updates automatically, but we need to read results in the shooter control
            
            // ========================================
            // STEP 9: HANDLE IMU RESET (D-PAD DOWN)
            // ========================================
            // Reset IMU to 0 degrees when D-pad down is pressed
            // Use this if field-centric drive stops working correctly
            boolean dpadDownCurrentlyPressed = gamepad1.dpad_down;
            boolean dpadDownJustPressed = dpadDownCurrentlyPressed && !prevDpadDown;
            
            if (dpadDownJustPressed && odo != null) {
                try {
                    odo.resetPosAndIMU(); // Reset IMU to 0 degrees
                    telemetry.addData("IMU RESET", "IMU reset to 0° (D-pad down pressed)");
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to reset IMU: " + e.getMessage());
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
            // Read joystick values and apply deadzone to prevent drift
            double y = applyAdvancedDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x = applyAdvancedDeadzone(gamepad1.left_stick_x);  // Strafing
            double rx = applyAdvancedDeadzone(gamepad1.right_stick_x); // Rotation
            
            // Override rotation with auto-align command if active
            if (autoAlignMode) {
                rx = autoAlignRotationCommand;
            }

            // Square inputs to make small movements smoother
            y = Math.copySign(y * y, y);
            x = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);

            // ========================================
            // STEP 14: DRIVE CONTROL
            // ========================================
            // Drive robot using field-centric control (IMU-based)
            manualDriveControl(x, y, rx);
            
            // Get current heading for telemetry
            double botHeading = 0.0;
            if (odo != null) {
                try {
                    botHeading = odo.getHeading(AngleUnit.RADIANS);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to read PinPoint IMU heading: " + e.getMessage());
                }
            }
            
            // Get motor powers for telemetry
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
            telemetry.addData("DEBUG", "Intake motor is NULL - cannot control");
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
                telemetry.addData("AUTO-ALIGN", "ERROR: Limelight not available");
                stopAutoAlign("Limelight not available");
                return;
            }
            
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addData("AUTO-ALIGN", "WARNING: No valid AprilTag detected - cannot align");
                telemetry.addData("AUTO-ALIGN", "Make sure AprilTag is in view of Limelight camera");
                // Don't stop auto-align, just wait for tag to appear
                autoAlignRotationCommand = 0.0;
                return;
            }
            
            // Read Limelight tx (horizontal offset in degrees)
            // tx < 0: Target to the left → rotate left (negative rotation)
            // tx > 0: Target to the right → rotate right (positive rotation)
            double tx = result.getTx();
            
            // Auto-correct adjustment: Subtract 3 degrees to compensate for shooting right
            // This shifts the alignment target 3 degrees to the left
            tx = tx - 3.0;
            
            // Check if heading is aligned (within tolerance)
            if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
                // Aligned! Stop rotation but keep mode active
                autoAlignRotationCommand = 0.0;
                telemetry.addData("AUTO-ALIGN", "ALIGNED! (tx=" + String.format("%.2f", tx) + "°)");
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
            
            telemetry.addData("AUTO-ALIGN", "Aligning... (tx=" + String.format("%.2f", tx) + 
                "°, rot=" + String.format("%.2f", rotationCommand) + ")");
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
            telemetry.addData("AUTO-ALIGN", "ERROR: No valid Limelight data - cannot see AprilTag");
            telemetry.addData("AUTO-ALIGN", "Make sure Limelight can see the AprilTag");
            return;
        }
        
        // Check if already aligned
        double tx = result.getTx();
        
        // Auto-correct adjustment: Subtract 3 degrees to compensate for shooting right
        // This shifts the alignment target 3 degrees to the left
        tx = tx - 3.0;
        
        if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
            telemetry.addData("AUTO-ALIGN", "Already aligned! (tx=" + String.format("%.2f", tx) + "°)");
            autoAlignMode = true;
            autoAlignRotationCommand = 0.0;
            autoAlignTimer.reset();
            return;
        }
        
        // Initialize auto-align mode
        autoAlignMode = true;
        autoAlignTimer.reset();
        
        telemetry.addData("AUTO-ALIGN", "Mode started - aligning robot to center AprilTag");
        telemetry.addData("AUTO-ALIGN", "Current tx=" + String.format("%.2f", tx) + 
            "° (target: 0.0°, tolerance: ±" + String.format("%.1f", AUTO_ALIGN_TX_TOLERANCE) + "°)");
        telemetry.addData("AUTO-ALIGN", "tx<0 means target left, tx>0 means target right");
    }
    
    /**
     * Stop auto-align mode
     */
    private void stopAutoAlign(String reason) {
        autoAlignMode = false;
        autoAlignRotationCommand = 0.0;
        telemetry.addData("AUTO-ALIGN", "Mode stopped - " + reason);
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
            currentFeedMotorPower = 0.5;  // R2 uses 0.5 power
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
                        telemetry.addData("DEBUG", "Shooter: Starting spin-up to " + String.format("%.0f", targetVelocity) + " RPM (distance-based)");
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
                    
                    // Debug: Show target vs actual
                    telemetry.addData("DEBUG", "Shooter: Target=" + String.format("%.0f", currentTargetVelocity) + 
                        " RPM, L=" + String.format("%.1f", leftVel) + " R=" + String.format("%.1f", rightVel) + 
                        " RPM (need " + String.format("%.1f", effectiveTarget) + ")");
                    
                    if (leftVel >= effectiveTarget && rightVel >= effectiveTarget) {
                        shooterState = ShooterState.READY;
                        telemetry.addData("DEBUG", "Shooter: Velocity reached (L:" + String.format("%.1f", leftVel) + 
                            " R:" + String.format("%.1f", rightVel) + " RPM)");
                    } else {
                        telemetry.addData("DEBUG", "Shooter: Spinning up... (L:" + String.format("%.1f", leftVel) + 
                            " R:" + String.format("%.1f", rightVel) + " / " + String.format("%.1f", effectiveTarget) + " RPM)");
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
                    telemetry.addData("DEBUG", "Shooter: Feed motor started at " + String.format("%.1f", currentFeedMotorPower) + " power");
                }
                
                // Start intake at reduced power (0.5) to feed ball to feeder
                // This pushes the ball into the chamber when first feed is triggered
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_SHOOTER_POWER);
                    telemetry.addData("DEBUG", "Shooter: Intake started at " + INTAKE_SHOOTER_POWER + " power (pushing ball into chamber)");
                }
                
                telemetry.addData("DEBUG", "Shooter: Feeding ball to shooter");
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
                    currentFeedMotorPower = 0.5;
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
                    telemetry.addData("DEBUG", "Shooter: Button released - stopped all systems");
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
        telemetry.addData("DEBUG", "Shooter: Stopped");
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
        telemetry.addData("DEBUG", "Shooter: Feeding stopped");
    }
    
    /**
     * Calculate dynamic shooter velocity based on actual distance to AprilTag
     * Uses calculated distance from Limelight ta with polynomial aggression curve
     * Calibration: Short distance (36") = 1000 RPM, Long distance (114") = 1400 RPM
     * 
     * Polynomial Formula:
     * - Normalizes distance to 0.0 (short) to 1.0 (long)
     * - Applies polynomial curve: curve = a*x^3 + b*x^2 + c*x + d
     * - Maps curve result to velocity range
     * 
     * Tuning Guide:
     * - POLY_COEFF_A (cubic): Controls high-distance aggression
     *   * Positive: Aggressive velocity increase at long distances
     *   * Negative: Gentler velocity increase at long distances
     * - POLY_COEFF_B (quadratic): Controls mid-distance curve shape
     *   * Positive: Faster mid-range acceleration
     *   * Negative: Slower mid-range acceleration
     * - POLY_COEFF_C (linear): Controls overall slope (1.0 = linear)
     * - POLY_COEFF_D (constant): Baseline offset (usually 0.0)
     * 
     * Example aggressive curve: A=0.5, B=-0.3, C=0.8, D=0.0
     * Example gentle curve: A=-0.2, B=0.1, C=1.1, D=0.0
     * 
     * @return Calculated target velocity in RPM
     */
    private double calculateDynamicVelocity() {
        if (limelight == null) {
            // Fallback to default velocity if Limelight not available
            return SHOOTER_TARGET_VELOCITY;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // Fallback to default velocity if no valid Limelight data
            return SHOOTER_TARGET_VELOCITY;
        }
        
        // Get target area (ta) from Limelight to calculate distance
        double ta = result.getTa();
        if (ta <= 0.0) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
        // Calculate base distance from target area
        double distanceCalibrationFactor = 50.0;
        double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
        
        // Adjust distance based on how far away the target is
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
        
        // Keep distance within our working range
        actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, actualDistance));
        
        // Convert distance to a number between 0.0 (close) and 1.0 (far)
        double normalizedDistance = (actualDistance - SHORT_DISTANCE_INCHES) / 
            (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
        
        // Calculate curve value using polynomial formula
        double polynomialCurve = POLY_COEFF_A * normalizedDistance * normalizedDistance * normalizedDistance +
                                 POLY_COEFF_B * normalizedDistance * normalizedDistance +
                                 POLY_COEFF_C * normalizedDistance +
                                 POLY_COEFF_D;
        
        // Keep curve between 0.0 and 1.0
        polynomialCurve = Math.max(0.0, Math.min(1.0, polynomialCurve));
        
        // Calculate base velocity: 0.0 = slow speed, 1.0 = fast speed
        double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
            (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * polynomialCurve;
        
        // Reduce speed for close shots (helps accuracy)
        if (actualDistance < REDUCTION_TAPER_DISTANCE) {
            // Calculate how much to reduce (more reduction when closer)
            double reductionFactor = 1.0 - ((actualDistance - SHORT_DISTANCE_INCHES) / 
                (REDUCTION_TAPER_DISTANCE - SHORT_DISTANCE_INCHES));
            reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor));
            
            // Apply the reduction
            double velocityReduction = SHORT_DISTANCE_RPM_REDUCTION * reductionFactor;
            calculatedVelocity -= velocityReduction;
        }
        
        // Increase speed for far shots (helps reach target)
        if (actualDistance >= INCREASE_START_DISTANCE) {
            calculatedVelocity += LONG_DISTANCE_RPM_INCREASE;
        }
        
        // Apply distance-specific correction for far shots (reduce RPM at 117 inches by 25 RPM)
        // Only applies when in the far shot range (above INCREASE_START_DISTANCE)
        if (actualDistance >= INCREASE_START_DISTANCE) {
            double distanceFromCorrection = Math.abs(actualDistance - CORRECTION_DISTANCE_CENTER);
            if (distanceFromCorrection <= CORRECTION_DISTANCE_RANGE) {
                // Apply correction with smooth taper (full correction at center, tapering to edges)
                double correctionFactor = 1.0 - (distanceFromCorrection / CORRECTION_DISTANCE_RANGE);
                double correctionAmount = CORRECTION_RPM_REDUCTION * correctionFactor;
                calculatedVelocity -= correctionAmount;
            }
        }
        
        // Keep velocity within safe limits
        calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
        
        return calculatedVelocity;
    }
    
    /**
     * Stop all motors (cleanup)
     */
    private void stopAllMotors() {
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (feedMotor != null) feedMotor.setPower(0.0);
        stopShooter();
        
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }
    
    /**
     * Handle manual drive control with field-oriented control
     * Uses PinPoint IMU directly for field-centric heading
     * Field-centric: Forward always moves toward field, regardless of robot orientation
     * For BLUE side: 0° = facing away from driver (toward field) - same as RED side
     */
    private void manualDriveControl(double x, double y, double rx) {
        // Get the robot's current heading from PinPoint IMU
        // For BLUE side: 0° = facing away from driver (toward field) - same as RED side
        double botHeading = 0.0;
        boolean imuAvailable = false;
        if (odo != null) {
            try {
                botHeading = odo.getHeading(AngleUnit.RADIANS);
                imuAvailable = true;
            } catch (Exception e) {
                // If IMU read fails, use 0 heading (robot-centric mode)
                telemetry.addData("ERROR", "PinPoint IMU read failed, using robot-centric: " + e.getMessage());
                botHeading = 0.0;
                imuAvailable = false;
            }
        } else {
            telemetry.addData("ERROR", "PinPoint IMU is NULL - using robot-centric mode");
            botHeading = 0.0;
            imuAvailable = false;
        }

        // Convert joystick input to field coordinates (field-centric control)
        // This makes forward always move toward the field, not the robot's current direction
        if (imuAvailable && odo != null) {
            // Rotate joystick input based on robot's heading
            double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
            x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
            y = temp;
            
            telemetry.addData("Field-Centric", "ACTIVE (Heading: " + String.format("%.1f°", Math.toDegrees(botHeading)) + ")");
        } else {
            // Robot-centric mode (fallback if IMU not working)
            if (odo == null) {
                telemetry.addData("Field-Centric", "DISABLED - IMU is NULL");
            } else if (!imuAvailable) {
                telemetry.addData("Field-Centric", "DISABLED - IMU read failed");
            }
        }

        // Mecanum wheel calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        // Apply minimum power threshold
        flPower = applyMinPower(flPower);
        blPower = applyMinPower(blPower);
        frPower = applyMinPower(frPower);
        brPower = applyMinPower(brPower);

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
     * Adds minimum power to prevent motors from stalling
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
        
        telemetry.addLine("=== BLUE DEC TELEOP ===");
        
        // PinPoint IMU heading (no position tracking without odometry pods)
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(botHeading)));
        telemetry.addData("Heading (Rad)", String.format("%.3f", botHeading));
        if (odo != null) {
            telemetry.addData("IMU Status", odo.getDeviceStatus().toString());
            // Debug: Check if IMU is actually updating
            try {
                double currentHeading = odo.getHeading(AngleUnit.RADIANS);
                telemetry.addData("IMU Heading", String.format("%.3f rad (%.1f°)", currentHeading, Math.toDegrees(currentHeading)));
            } catch (Exception e) {
                telemetry.addData("IMU Read Error", e.getMessage());
            }
        } else {
            telemetry.addData("IMU Status", "DISCONNECTED");
            telemetry.addData("WARNING", "Field-centric disabled - IMU is NULL!");
        }
        
        // Intake status with gamepad state for debugging
        if (intakeMotor != null) {
            telemetry.addData("Intake Power", String.format("%.2f", intakeMotor.getPower()));
            telemetry.addData("L1 Button", gamepad1.left_bumper ? "PRESSED" : "RELEASED");
            telemetry.addData("L2 Trigger", String.format("%.2f", gamepad1.left_trigger));
        } else {
            telemetry.addData("Intake Motor", "NULL - NOT INITIALIZED");
        }
        
        // Feed motor status
        if (feedMotor != null) {
            telemetry.addData("Feed Power", String.format("%.2f", feedMotor.getPower()));
        }
        
        // Auto-align status
        telemetry.addData("Auto-Align", autoAlignMode ? "ACTIVE" : "INACTIVE");
        if (autoAlignMode) {
            telemetry.addData("Align Time", String.format("%.1fs", autoAlignTimer.seconds()));
            telemetry.addData("Align Rot Cmd", String.format("%.2f", autoAlignRotationCommand));
        }
        
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
                
                // Line 1: Show tx and ty from Limelight
                telemetry.addData("Limelight TX/TY", String.format("TX: %.2f° | TY: %.2f°", tx, ty) + 
                    (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE ? " [ALIGNED]" : ""));
                
                // Calculate distance from AprilTag to Limelight
                double distanceCalibrationFactor = 50.0;
                double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
                
                // Apply distance multiplier
                double appliedMultiplier = 1.0;
                if (baseDistance < 40.0) {
                    appliedMultiplier = 1.424;
                } else if (baseDistance < 90.0) {
                    appliedMultiplier = 1.42 + (baseDistance - 40.0) * (-0.00212);
                } else {
                    appliedMultiplier = Math.max(0.95, 1.333 + (baseDistance - 81.0) * (-0.0030));
                }
                
                double actualDistance = baseDistance * appliedMultiplier;
                actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, actualDistance));
                
                // Line 2: Show distance calculation
                telemetry.addData("Distance to AprilTag", String.format("%.1f\" (TA: %.3f, Base: %.1f\", Mult: %.3f)", 
                    actualDistance, ta, baseDistance, appliedMultiplier));
                
                // Calculate velocity using polynomial curve
                double normalizedDistance = (actualDistance - SHORT_DISTANCE_INCHES) / 
                    (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
                double polynomialCurve = POLY_COEFF_A * normalizedDistance * normalizedDistance * normalizedDistance +
                                        POLY_COEFF_B * normalizedDistance * normalizedDistance +
                                        POLY_COEFF_C * normalizedDistance +
                                        POLY_COEFF_D;
                polynomialCurve = Math.max(0.0, Math.min(1.0, polynomialCurve));
                
                double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
                    (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * polynomialCurve;
                calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
                
                // Line 3: Show calculated velocity
                telemetry.addData("Calculated Velocity", String.format("%.0f RPM (NormDist: %.3f, Curve: %.3f)", 
                    calculatedVelocity, normalizedDistance, polynomialCurve));
                
                // Additional debug info
                telemetry.addData("Limelight Status", "CONNECTED - Target Detected");
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
                telemetry.addData("Distance to AprilTag", "N/A - No target");
                telemetry.addData("Calculated Velocity", "N/A - Using default: " + String.format("%.0f", SHOOTER_TARGET_VELOCITY) + " RPM");
                telemetry.addData("Limelight Status", "NO TARGET");
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
            telemetry.addData("Distance to AprilTag", "N/A - Limelight disconnected");
            telemetry.addData("Calculated Velocity", "N/A - Using default: " + String.format("%.0f", SHOOTER_TARGET_VELOCITY) + " RPM");
            telemetry.addData("Limelight Status", "DISCONNECTED");
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
                telemetry.addData("L Power", String.format("%.2f", leftPower));
                telemetry.addData("R Power", String.format("%.2f", rightPower));
                telemetry.addData("PIDF F", String.format("%.1f", SHOOTER_PIDF_F));
                
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
        
        // Drive motor powers
        telemetry.addLine("--- Drive Powers ---");
        telemetry.addData("FL", String.format("%.2f", flPower));
        telemetry.addData("FR", String.format("%.2f", frPower));
        telemetry.addData("BL", String.format("%.2f", blPower));
        telemetry.addData("BR", String.format("%.2f", brPower));
        
        telemetry.update();
    }
}

