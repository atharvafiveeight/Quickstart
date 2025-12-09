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
 * NovTeleOpRed - Field-Oriented Control TeleOp Program
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
 * @author Sahas Kumar
 * @version 1.0
 */
@TeleOp(name = "NovTeleOpRed", group = "TeleOp")
public class NovTeleOpRed extends LinearOpMode {

    // Motor declarations
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
    private static final double JOYSTICK_DEADZONE = 0.10; // Reduced deadzone for more responsiveness
    private static final double MIN_MOTOR_POWER = 0.12;   // Increased minimum power for more torque
    private static final double DRIVE_POWER_MULTIPLIER = 0.8; // Power multiplier for speed control
    
    // Launcher constants - adjust these values for your robot
    private final double FEED_TIME_SECONDS = 0.05;        // How long to feed game pieces (reduced for single ball)
    private final double FEED_DELAY_SECONDS = 1.5;        // Delay between feeds for continuous shooting (1 second)
    private final double STOP_SPEED = 0.0;                // Stop speed for shooter
    private final double FULL_SPEED = 1.0;                // Full speed for servos
    
    // Short distance shooting (right bumper) - original values
    private final double LAUNCHER_TARGET_VELOCITY = 1225; // Target velocity for shooter
    private final double LAUNCHER_MIN_VELOCITY = 1225;    // Minimum velocity before launching
    private final double LAUNCHER_POWER = 0.8;            // Motor power for short distance
    
    // Long distance shooting (left bumper) - higher values
    private final double LONG_DISTANCE_TARGET_VELOCITY = 1600; // Higher velocity for long distance
    private final double LONG_DISTANCE_MIN_VELOCITY = 1600;    // Higher minimum velocity
    private final double LONG_DISTANCE_POWER = 0.95;           // Higher motor power for long distance
    
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
        telemetry.addData("Status", "NovTeleOpRed Ready!");
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
            
            // Manual pre-warm with X button (uses current shooting mode)
            if (gamepad1.x) {
                double targetVelocity = (currentShootingMode == ShootingMode.LONG_DISTANCE) ? 
                    LONG_DISTANCE_TARGET_VELOCITY : LAUNCHER_TARGET_VELOCITY;
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(targetVelocity);
                telemetry.addData("DEBUG", "MANUAL PRE-WARM: Setting velocity to " + targetVelocity + 
                    " (" + currentShootingMode.toString() + ")");
                telemetry.addData("DEBUG", "MANUAL PRE-WARM: Current velocity: " + String.format("%.1f", shooterMotor.getVelocity()));
            } else if (!anyBumperHeld && launchState == LaunchState.IDLE) {
                // Only stop if not using any bumper and in idle state
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setVelocity(0);
                telemetry.addData("DEBUG", "STOPPING: Setting velocity to 0");
            }
            
            // Run the launch state machine
            launch(anyBumperPressed, anyBumperReleased, anyBumperHeld);
            
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
            // STEP 8: FIELD-ORIENTED CONTROL CALCULATION
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
            // STEP 9: MECANUM WHEEL CALCULATIONS
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
            // STEP 10: APPLY MOTOR POWERS
            // ========================================
            // Send the calculated powers to each motor
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // ========================================
            // STEP 11: UPDATE TELEMETRY
            // ========================================
            // Show important information on the driver station screen
            updateTelemetry(flPower, frPower, blPower, brPower, botHeading);
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
                    shooterMotor.setPower(currentPower * 0.6); // Reduce to 60% of current power
                } else {
                    // Use normal power if below target
                    shooterMotor.setPower(currentPower);
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
                    shooterMotor.setPower(currentPower * 0.6); // Reduce to 60% of current power
                    telemetry.addData("DEBUG", "CONTINUOUS: Velocity at target, reducing power to " + String.format("%.2f", currentPower * 0.6));
                } else {
                    // Use normal power if below target
                    shooterMotor.setPower(currentPower);
                    telemetry.addData("DEBUG", "CONTINUOUS: Below target velocity, using " + String.format("%.2f", currentPower) + " power");
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
        telemetry.addData("X Button", "Manual Pre-warm (current mode)");
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
