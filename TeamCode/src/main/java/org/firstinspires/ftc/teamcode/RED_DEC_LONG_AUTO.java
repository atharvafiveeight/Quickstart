package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Limelight vision sensor imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * RED_DEC_LONG_AUTO - Long Autonomous Program for December Robot (Red Side)
 * 
 * This autonomous program:
 * - Shoots 3 preloaded balls at shotPose
 * - Picks up 3 balls from intakeR3 location
 * - Shoots those 3 balls
 * - Picks up 3 balls from intakeR2 location
 * - Shoots those 3 balls
 * - Parks at endPose
 * 
 * Features:
 * - Uses PedroPathing for smooth movement
 * - Uses Limelight for distance-based velocity calculation
 * - Pre-warms shooter motors to 75% target velocity while moving to shotPose
 * - Runs intake motors during pickup and while moving to shotPose
 * 
 * @author Team
 * @version 1.0
 */
@Autonomous(name = "RED_DEC_LONG_AUTO", group = "Autonomous")
@Configurable
public class RED_DEC_LONG_AUTO extends OpMode {

    // ========================================
    // PEDRO PATHING
    // ========================================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    
    // ========================================
    // HARDWARE DECLARATIONS
    // ========================================
    private DcMotor intakeMotor;           // Intake motor (no encoder)
    private DcMotor feedMotor;            // Feed motor (no encoder)
    private DcMotorEx shooterMotorLeft;    // Left shooter motor (velocity controlled)
    private DcMotorEx shooterMotorRight;   // Right shooter motor (velocity controlled)
    private Limelight3A limelight;         // Limelight for distance-based velocity calculation
    private Servo rgb;                     // RGB LED indicator
    
    // ========================================
    // PANELS TELEMETRY
    // ========================================
    private TelemetryManager panelsTelemetry;
    
    // ========================================
    // PATH DEFINITIONS
    // ========================================
    private PathChain shootPose1;          // First shot (preloaded balls)
    private PathChain intakeR3Ready;       // Ready position for R3 intake
    private PathChain intakeR3;            // R3 intake position
    private PathChain shootPose2;          // Second shot (after R3 intake)
    private PathChain intakeR2Ready;       // Ready position for R2 intake
    private PathChain intakeR2;            // R2 intake position
    private PathChain shootPose3;           // Third shot (after R2 intake)
    private PathChain endPose;             // Final park position
    
    // ========================================
    // SHOOTING CONSTANTS (from RED_DEC_TELEOP)
    // ========================================
    private static final double INTAKE_FORWARD_POWER = 0.95;      // Power for intake when picking up
    private static final double INTAKE_MOVING_POWER = 0.7;        // Power for intake while moving to shotPose
    private static final double FEED_MOTOR_POWER = 0.5;           // Power for feed motor when shooting
    private static final double SHOOTER_PREWARM_PERCENT = 0.75;   // Pre-warm to 75% of target velocity
    
    // Shooter Distance Constants
    private static final double SHORT_DISTANCE_INCHES = 36.0;
    private static final double LONG_DISTANCE_INCHES = 130.0;
    private static final double SHORT_DISTANCE_VELOCITY = 1000.0;
    private static final double LONG_DISTANCE_VELOCITY = 1400.0;  // Reduced by 25 RPM from 1400.0
    private static final double MIN_VELOCITY = 800.0;
    private static final double MAX_VELOCITY = 1700.0;
    
    // Shooter Velocity Adjustments
    private static final double SHORT_DISTANCE_RPM_REDUCTION = 100.0;
    private static final double REDUCTION_TAPER_DISTANCE = 80.0;
    private static final double LONG_DISTANCE_RPM_INCREASE = 100.0;
    private static final double INCREASE_START_DISTANCE = 114.0;
    
    // Shooter Velocity Curve (Polynomial)
    private static final double POLY_COEFF_A = 0.0;
    private static final double POLY_COEFF_B = 0.0;
    private static final double POLY_COEFF_C = 1.0;
    private static final double POLY_COEFF_D = 0.0;
    
    // Shooter Control Constants
    private static final double SHOOTER_TARGET_VELOCITY = 1475.0;
    private static final double SHOOTER_VELOCITY_THRESHOLD = 0.95;
    private static final double SHOOTER_PIDF_P = 100.0;
    private static final double SHOOTER_PIDF_I = 0.0;
    private static final double SHOOTER_PIDF_D = 0.0;
    private static final double SHOOTER_PIDF_F = 30.0;
    
    // RGB LED Constants
    private static final double RGB_RED = 0.2777;
    private static final double RGB_GREEN = 0.5;
    private static final double RGB_ORANGE = 0.333;
    
    // Shooting Constants
    private static final int TOTAL_BALLS_PER_ROUND = 3;           // Number of balls to shoot per round
    private static final double FEED_TIME_SECONDS = 0.5;         // How long to feed each ball
    private static final double FEED_DELAY_SECONDS = 0.3;        // Delay between feeds
    private static final double FINAL_LAUNCH_DELAY = 1.0;        // Extra delay after last ball
    
    // Autonomous Speed Constants (Conservative values for accuracy)
    private static final double AUTONOMOUS_SPEED_MULTIPLIER = 0.7;  // 10% of max speed (very conservative for accuracy)
    
    // Auto-Align Constants (Optimized for smooth, non-wobbly movement)
    private static final double AUTO_ALIGN_TX_TOLERANCE = 0.5;   // How close to center is "aligned" (degrees)
    private static final double AUTO_ALIGN_ROTATION_SPEED = 0.25; // Maximum rotation speed for alignment (reduced from 0.35 for smoother movement)
    private static final double AUTO_ALIGN_TX_GAIN = 0.10;       // How fast to rotate based on target position (reduced from 0.14 to reduce overshoot)
    private static final double AUTO_ALIGN_TIMEOUT = 3.0;        // Stop trying after 3 seconds
    private static final double AUTO_ALIGN_MIN_ROTATE = 0.03;    // Minimum rotation to make robot move (reduced from 0.05 for finer control)
    private static final double AUTO_ALIGN_SMOOTHING_FACTOR = 0.7; // Exponential smoothing factor (0.0-1.0, higher = more smoothing, less responsive)
    private static final double AUTO_ALIGN_DEADBAND = 0.3;        // Deadband zone - don't rotate if within this range (degrees)
    
    // Intake during shooting
    private static final double INTAKE_SHOOTING_POWER = 0.85;     // Intake power while shooting (helps feed balls)
    private static final double INTAKE_FULL_POWER = 1.0;          // Full power intake when moving to intake spots
    
    // ========================================
    // SHOOTING STATE MACHINE
    // ========================================
    private enum ShooterState {
        IDLE,           // Shooter is stopped
        PREWARM,        // Pre-warming to 75% of target velocity (while moving)
        SPIN_UP,        // Spinning up to full target velocity
        READY,          // At target velocity, ready to feed
        FEEDING         // Currently feeding balls
    }
    
    private ShooterState shooterState = ShooterState.IDLE;
    private int ballsScored = 0;
    private boolean shootingComplete = false;
    private double currentTargetVelocity = SHOOTER_TARGET_VELOCITY;
    private ElapsedTime feedTimer = new ElapsedTime();
    private boolean isPrewarming = false;  // Track if we're in pre-warm mode
    
    // Auto-Align Variables
    private boolean autoAlignMode = false;  // Is auto-align currently active?
    private double autoAlignRotationCommand = 0.0;  // How much to rotate (from auto-align)
    private ElapsedTime autoAlignTimer = new ElapsedTime();  // Timer to stop auto-align after timeout
    private double smoothedTx = 0.0;  // Smoothed tx value (exponential moving average) to reduce noise and wobbling
    
    // ========================================
    // INITIALIZATION
    // ========================================
    @Override
    public void init() {
        // Initialize PedroPathing using standard Constants (simple approach)
        follower = Constants.createFollower(hardwareMap);
        
        // Set maxPower to 10% for slow, accurate movement
        // Using setMaxPower method on follower (simpler than custom MecanumConstants)
        follower.setMaxPower(AUTONOMOUS_SPEED_MULTIPLIER);
        telemetry.addData("DEBUG", "MaxPower set to " + String.format("%.0f%%", AUTONOMOUS_SPEED_MULTIPLIER * 100));
        
        follower.setStartingPose(new Pose(88.012, 8.251, Math.toRadians(90)));
        
        pathTimer = new Timer();
        pathState = 0;
        
        // Initialize Panels telemetry
        PanelsConfigurables.INSTANCE.refreshClass(this);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Initialize hardware
        initializeHardware();
        
        // Build paths
        buildPaths();
        
        // Initialize state variables
        shooterState = ShooterState.IDLE;
        ballsScored = 0;
        shootingComplete = false;
        
        telemetry.addData("Status", "RED_DEC_LONG_AUTO Initialized");
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        // L1 button (left bumper) control for pre-feeding balls during init
        // Hold L1 to run intake motor for pre-feeding balls into the robot
        if (gamepad1.left_bumper) {
            // L1 pressed - run intake motor at full power
            if (intakeMotor != null) {
                intakeMotor.setPower(INTAKE_FULL_POWER);
                telemetry.addData("Status", "Waiting for start... (L1: INTAKE RUNNING)");
            } else {
                telemetry.addData("Status", "Waiting for start... (L1: Intake motor not available)");
            }
        } else {
            // L1 released - stop intake motor
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
            telemetry.addData("Status", "Waiting for start... (Hold L1 to run intake)");
        }
        
        // Show intake status
        if (intakeMotor != null) {
            telemetry.addData("Intake Power", String.format("%.2f", intakeMotor.getPower()));
        }
        
        telemetry.update();
    }
    
    @Override
    public void start() {
        // Stop intake motor when autonomous starts (in case it was running during init)
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        
        pathTimer.resetTimer();
        pathState = 0;
        shootingComplete = false;
        ballsScored = 0;
        shooterState = ShooterState.IDLE;
    }
    
    @Override
    public void loop() {
        // Update PedroPathing
        follower.update();
        
        // Update autonomous state machine
        autonomousPathUpdate();
        
        // Update shooting state machine (runs continuously)
        autonomousShootingUpdate();
        
        // Handle auto-align during shooting (runs continuously when shooting)
        handleAutoAlign();
        
        // Update telemetry
        updateTelemetry();
    }
    
    @Override
    public void stop() {
        // Stop all motors
        stopAllMotors();
    }
    
    // ========================================
    // HARDWARE INITIALIZATION
    // ========================================
    private void initializeHardware() {
        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            if (intakeMotor != null) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intakeMotor.setPower(0.0);
                telemetry.addData("DEBUG", "Intake motor initialized");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize intake motor: " + e.getMessage());
        }
        
        // Initialize feed motor
        try {
            feedMotor = hardwareMap.get(DcMotor.class, "feedMotor");
            if (feedMotor != null) {
                feedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                feedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                feedMotor.setPower(0.0);
                telemetry.addData("DEBUG", "Feed motor initialized");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize feed motor: " + e.getMessage());
        }
        
        // Initialize shooter motors
        try {
            shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
            shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
            
            if (shooterMotorLeft != null && shooterMotorRight != null) {
                // Set opposite directions for flywheel
                shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
                
                // Set to FLOAT mode
                shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                
                // Configure for velocity control
                shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // Set PIDF coefficients
                PIDFCoefficients pidfCoeffs = new PIDFCoefficients(
                    SHOOTER_PIDF_P, SHOOTER_PIDF_I, SHOOTER_PIDF_D, SHOOTER_PIDF_F
                );
                shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
                shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
                
                telemetry.addData("DEBUG", "Shooter motors initialized");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motors: " + e.getMessage());
        }
        
        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(0);
                limelight.start();
                telemetry.addData("DEBUG", "Limelight initialized");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize Limelight: " + e.getMessage());
        }
        
        // Initialize RGB LED
        try {
            rgb = hardwareMap.get(Servo.class, "rgb");
            if (rgb != null) {
                rgb.setPosition(RGB_RED);
                telemetry.addData("DEBUG", "RGB LED initialized");
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize RGB LED: " + e.getMessage());
        }
    }
    
    // ========================================
    // PATH BUILDING
    // ========================================
    private void buildPaths() {
        try {
            // First shot pose (preloaded balls)
            shootPose1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.012, 8.251), new Pose(89.780, 13.752)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))
                .build();
            
            // Intake R3 ready position
            intakeR3Ready = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.780, 13.752), new Pose(96.952, 35.255)))
                .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                .build();
            
            // Intake R3 position
            intakeR3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96.952, 35.255), new Pose(132.33, 35.255)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            
            // Second shot pose (after R3 intake)
            shootPose2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.33, 35.255), new Pose(89.780, 13.752)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();
            
            // Intake R2 ready position
            intakeR2Ready = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.780, 13.752), new Pose(98.939, 59.462)))
                .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                .build();
            
            // Intake R2 position
            intakeR2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(98.939, 59.462), new Pose(131.589, 58.345)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            
            // Third shot pose (after R2 intake)
            shootPose3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(131.589, 58.345), new Pose(89.780, 13.752)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();
            
            // End pose (park)
            endPose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.780, 13.752), new Pose(109.229, 8.644)))
                .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                .build();
            
            telemetry.addData("DEBUG", "All paths built successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to build paths: " + e.getMessage());
        }
    }
    
    // ========================================
    // AUTONOMOUS STATE MACHINE
    // ========================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to first shot pose (preloaded balls)
                if (shootPose1 != null) {
                    follower.followPath(shootPose1);
                    setPathState(1);
                    telemetry.addData("DEBUG", "Moving to first shot pose");
                }
                break;
                
            case 1:
                // Wait for arrival at first shot pose, then start shooting
                if (!follower.isBusy()) {
                    setPathState(2);
                    setShootingState(1); // Start shooting preloaded balls
                    telemetry.addData("DEBUG", "Arrived at first shot pose, starting to shoot 3 preloaded balls");
                }
                break;
                
            case 2:
                // Wait for first shooting round to complete
                if (shootingComplete) {
                    setPathState(3);
                    shootingComplete = false; // Reset for next round
                    telemetry.addData("DEBUG", "First shooting round complete, moving to intakeR3Ready");
                }
                break;
                
            case 3:
                // Move to intakeR3Ready position - start intake at full power
                if (intakeR3Ready != null) {
                    // Start intake at full power (1.0) when moving to intake spot
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    follower.followPath(intakeR3Ready);
                    setPathState(4);
                    telemetry.addData("DEBUG", "Moving to intakeR3Ready with intake at full power");
                }
                break;
                
            case 4:
                // Wait for arrival, continue intake at full power
                if (!follower.isBusy()) {
                    // Keep intake at full power
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    setPathState(5);
                    telemetry.addData("DEBUG", "Arrived at intakeR3Ready, continuing intake at full power");
                }
                break;
                
            case 5:
                // Move to intakeR3 while running intake at full power
                if (intakeR3 != null) {
                    // Keep intake at full power while moving to intake spot
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    follower.followPath(intakeR3);
                    setPathState(6);
                    telemetry.addData("DEBUG", "Moving to intakeR3 while running intake at full power");
                }
                break;
                
            case 6:
                // Wait for arrival at intakeR3, then stop intake
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(7);
                    telemetry.addData("DEBUG", "Arrived at intakeR3, stopping intake, moving to shotPose with pre-warm");
                }
                break;
                
            case 7:
                // Move to shotPose while pre-warming shooters (75% velocity) and running intake at 0.7
                if (shootPose2 != null) {
                    // Start pre-warming shooters
                    startShooterPreWarm();
                    // Start intake at 0.7 power
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_MOVING_POWER);
                    }
                    follower.followPath(shootPose2);
                    setPathState(8);
                    telemetry.addData("DEBUG", "Moving to shotPose with pre-warm and intake at 0.7");
                }
                break;
                
            case 8:
                // Wait for arrival, then finish spin-up and shoot
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(9);
                    setShootingState(1); // Start shooting
                    telemetry.addData("DEBUG", "Arrived at shotPose, finishing spin-up and shooting");
                }
                break;
                
            case 9:
                // Wait for second shooting round to complete
                if (shootingComplete) {
                    setPathState(10);
                    shootingComplete = false; // Reset for next round
                    telemetry.addData("DEBUG", "Second shooting round complete, moving to intakeR2Ready");
                }
                break;
                
            case 10:
                // Move to intakeR2Ready position - start intake at full power
                if (intakeR2Ready != null) {
                    // Start intake at full power (1.0) when moving to intake spot
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    follower.followPath(intakeR2Ready);
                    setPathState(11);
                    telemetry.addData("DEBUG", "Moving to intakeR2Ready with intake at full power");
                }
                break;
                
            case 11:
                // Wait for arrival, continue intake at full power
                if (!follower.isBusy()) {
                    // Keep intake at full power
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    setPathState(12);
                    telemetry.addData("DEBUG", "Arrived at intakeR2Ready, continuing intake at full power");
                }
                break;
                
            case 12:
                // Move to intakeR2 while running intake at full power
                if (intakeR2 != null) {
                    // Keep intake at full power while moving to intake spot
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_FULL_POWER);
                    }
                    follower.followPath(intakeR2);
                    setPathState(13);
                    telemetry.addData("DEBUG", "Moving to intakeR2 while running intake at full power");
                }
                break;
                
            case 13:
                // Wait for arrival at intakeR2, then stop intake
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(14);
                    telemetry.addData("DEBUG", "Arrived at intakeR2, stopping intake, moving to shotPose with pre-warm");
                }
                break;
                
            case 14:
                // Move to shotPose while pre-warming shooters (75% velocity) and running intake at 0.7
                if (shootPose3 != null) {
                    // Start pre-warming shooters
                    startShooterPreWarm();
                    // Start intake at 0.7 power
                    if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_MOVING_POWER);
                    }
                    follower.followPath(shootPose3);
                    setPathState(15);
                    telemetry.addData("DEBUG", "Moving to shotPose with pre-warm and intake at 0.7");
                }
                break;
                
            case 15:
                // Wait for arrival, then finish spin-up and shoot
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(16);
                    setShootingState(1); // Start shooting
                    telemetry.addData("DEBUG", "Arrived at shotPose, finishing spin-up and shooting");
                }
                break;
                
            case 16:
                // Wait for third shooting round to complete
                if (shootingComplete) {
                    setPathState(17);
                    shootingComplete = false;
                    telemetry.addData("DEBUG", "Third shooting round complete, moving to endPose");
                }
                break;
                
            case 17:
                // Move to end pose
                if (endPose != null) {
                    follower.followPath(endPose);
                    setPathState(18);
                    telemetry.addData("DEBUG", "Moving to endPose");
                }
                break;
                
            case 18:
                // Wait for arrival at end pose, then finish
                if (!follower.isBusy()) {
                    setPathState(-1);
                    telemetry.addData("DEBUG", "Autonomous complete!");
                }
                break;
        }
    }
    
    // ========================================
    // SHOOTING STATE MACHINE
    // ========================================
    public void autonomousShootingUpdate() {
        switch (shooterState) {
            case IDLE:
                // Shooter is stopped
                if (shooterMotorLeft != null && shooterMotorRight != null) {
                    try {
                        shooterMotorLeft.setVelocity(0);
                        shooterMotorRight.setVelocity(0);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to stop shooters: " + e.getMessage());
                    }
                }
                break;
                
            case PREWARM:
                // Pre-warming to 75% of target velocity (while moving to shotPose)
                if (shooterMotorLeft != null && shooterMotorRight != null) {
                    // Calculate target velocity based on Limelight distance
                    double targetVel = calculateDynamicVelocity();
                    currentTargetVelocity = targetVel;
                    double prewarmVel = targetVel * SHOOTER_PREWARM_PERCENT;
                    
                    try {
                        shooterMotorLeft.setVelocity(prewarmVel);
                        shooterMotorRight.setVelocity(prewarmVel);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to pre-warm shooters: " + e.getMessage());
                    }
                }
                break;
                
            case SPIN_UP:
                // Spinning up to full target velocity
                // Start auto-align when spinning up
                if (!autoAlignMode) {
                    startAutoAlign();
                }
                
                // Run intake at 0.75 power while spinning up (helps prepare balls)
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_SHOOTING_POWER);
                }
                
                if (shooterMotorLeft != null && shooterMotorRight != null) {
                    // Update velocity based on current distance
                    double targetVel = calculateDynamicVelocity();
                    currentTargetVelocity = targetVel;
                    
                    try {
                        shooterMotorLeft.setVelocity(targetVel);
                        shooterMotorRight.setVelocity(targetVel);
                        
                        // Check if both motors have reached threshold velocity
                        double leftVel = Math.abs(shooterMotorLeft.getVelocity());
                        double rightVel = Math.abs(shooterMotorRight.getVelocity());
                        double effectiveTarget = targetVel * SHOOTER_VELOCITY_THRESHOLD;
                        
                        if (leftVel >= effectiveTarget && rightVel >= effectiveTarget) {
                            shooterState = ShooterState.READY;
                            telemetry.addData("DEBUG", "Shooter ready at velocity: " + String.format("%.1f", leftVel) + " RPM");
                        }
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to spin up shooters: " + e.getMessage());
                    }
                }
                break;
                
            case READY:
                // At target velocity, start feeding
                // Continue auto-align and intake
                shooterState = ShooterState.FEEDING;
                ballsScored = 0;
                feedTimer.reset();
                
                // Start feed motor
                if (feedMotor != null) {
                    feedMotor.setPower(FEED_MOTOR_POWER);
                }
                
                // Continue intake at 0.75 power while feeding
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_SHOOTING_POWER);
                }
                
                telemetry.addData("DEBUG", "Starting to feed balls");
                break;
                
            case FEEDING:
                // Feed balls one at a time with timing
                // Continue auto-align and intake during feeding
                if (shooterMotorLeft != null && shooterMotorRight != null) {
                    // Maintain velocity
                    double targetVel = calculateDynamicVelocity();
                    currentTargetVelocity = targetVel;
                    
                    try {
                        shooterMotorLeft.setVelocity(targetVel);
                        shooterMotorRight.setVelocity(targetVel);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to maintain shooter velocity: " + e.getMessage());
                    }
                }
                
                // Continue intake at 0.75 power while feeding
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_SHOOTING_POWER);
                }
                
                // Feed logic: Run feed motor continuously, count balls based on timing
                // Each ball takes FEED_TIME_SECONDS to feed, with FEED_DELAY_SECONDS between balls
                double elapsedTime = feedTimer.seconds();
                double feedCycleTime = FEED_TIME_SECONDS + FEED_DELAY_SECONDS;
                
                // Calculate how many balls should have been fed by now
                int expectedBalls = (int) ((elapsedTime + FEED_DELAY_SECONDS) / feedCycleTime);
                expectedBalls = Math.min(expectedBalls, TOTAL_BALLS_PER_ROUND);
                
                // Update balls scored if we've fed more
                if (expectedBalls > ballsScored) {
                    ballsScored = expectedBalls;
                    telemetry.addData("DEBUG", "Ball " + ballsScored + " fed (time: " + String.format("%.2f", elapsedTime) + "s)");
                }
                
                // Check if all balls have been fed and final delay has passed
                if (ballsScored >= TOTAL_BALLS_PER_ROUND) {
                    double totalFeedTime = TOTAL_BALLS_PER_ROUND * feedCycleTime - FEED_DELAY_SECONDS + FINAL_LAUNCH_DELAY;
                    if (elapsedTime >= totalFeedTime) {
                        // Stop feed motor
                        if (feedMotor != null) {
                            feedMotor.setPower(0.0);
                        }
                        
                        // Stop intake
                        if (intakeMotor != null) {
                            intakeMotor.setPower(0.0);
                        }
                        
                        // Stop auto-align
                        stopAutoAlign("Shooting complete");
                        
                        // Stop shooters
                        stopShooter();
                        shootingComplete = true;
                        telemetry.addData("DEBUG", "All " + TOTAL_BALLS_PER_ROUND + " balls shot");
                    }
                }
                break;
        }
    }
    
    // ========================================
    // SHOOTER CONTROL METHODS
    // ========================================
    private void startShooterPreWarm() {
        shooterState = ShooterState.PREWARM;
        isPrewarming = true;
        telemetry.addData("DEBUG", "Starting shooter pre-warm to 75%");
    }
    
    private void setShootingState(int state) {
        if (state == 1) {
            // If we were pre-warming, transition to spin-up
            if (isPrewarming) {
                shooterState = ShooterState.SPIN_UP;
                isPrewarming = false;
            } else {
                // Start from idle
                shooterState = ShooterState.SPIN_UP;
            }
            ballsScored = 0;
            shootingComplete = false;
            feedTimer.reset();
        }
    }
    
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
        isPrewarming = false;
        
        // Stop intake when shooter stops
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
    }
    
    // ========================================
    // INTAKE CONTROL METHODS
    // ========================================
    private void startIntake() {
        if (intakeMotor != null) {
            intakeMotor.setPower(INTAKE_FORWARD_POWER);
            telemetry.addData("DEBUG", "Intake started at " + INTAKE_FORWARD_POWER + " power");
        }
    }
    
    private void stopIntake() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
            telemetry.addData("DEBUG", "Intake stopped");
        }
    }
    
    // ========================================
    // VELOCITY CALCULATION (from RED_DEC_TELEOP)
    // ========================================
    /**
     * Calculate dynamic shooter velocity based on Limelight distance
     * Uses the same logic as RED_DEC_TELEOP
     */
    private double calculateDynamicVelocity() {
        if (limelight == null) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return SHOOTER_TARGET_VELOCITY;
        }
        
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
        
        // Calculate base velocity
        double calculatedVelocity = SHORT_DISTANCE_VELOCITY + 
            (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * polynomialCurve;
        
        // Reduce speed for close shots
        if (actualDistance < REDUCTION_TAPER_DISTANCE) {
            double reductionFactor = 1.0 - ((actualDistance - SHORT_DISTANCE_INCHES) / 
                (REDUCTION_TAPER_DISTANCE - SHORT_DISTANCE_INCHES));
            reductionFactor = Math.max(0.0, Math.min(1.0, reductionFactor));
            double velocityReduction = SHORT_DISTANCE_RPM_REDUCTION * reductionFactor;
            calculatedVelocity -= velocityReduction;
        }
        
        // Increase speed for far shots
        if (actualDistance >= INCREASE_START_DISTANCE) {
            calculatedVelocity += LONG_DISTANCE_RPM_INCREASE;
        }
        
        // Keep velocity within safe limits
        calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
        
        // Add 50 RPM boost for better accuracy at this distance
        calculatedVelocity += 50.0;
        
        // Re-clamp after adding boost
        calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
        
        return calculatedVelocity;
    }
    
    // ========================================
    // AUTO-ALIGN METHODS (from RED_DEC_TELEOP)
    // ========================================
    /**
     * Handle auto-align: Rotates robot to center AprilTag in Limelight view
     * Runs during shooting states (SPIN_UP, READY, FEEDING)
     */
    private void handleAutoAlign() {
        // Only run auto-align when shooting (SPIN_UP, READY, or FEEDING)
        if (shooterState == ShooterState.IDLE || shooterState == ShooterState.PREWARM) {
            if (autoAlignMode) {
                stopAutoAlign("Shooter not active");
            }
            return;
        }
        
        // Start auto-align when shooting starts
        if (!autoAlignMode && (shooterState == ShooterState.SPIN_UP || 
                               shooterState == ShooterState.READY || 
                               shooterState == ShooterState.FEEDING)) {
            startAutoAlign();
            // ALWAYS switch to teleop drive mode when shooting starts
            // This is needed because setTeleOpDrive only works in teleop mode, not path following mode
            // Even if path is complete, we need to explicitly switch to teleop mode
            follower.startTeleopDrive();
            telemetry.addData("AUTO-ALIGN", "Switched to teleop drive mode for auto-align");
        }
        
        // Handle auto-align if active
        if (autoAlignMode) {
            // ALWAYS ensure we're in teleop drive mode (not path following mode)
            // This allows setTeleOpDrive to work for rotation
            // Call startTeleopDrive every loop to ensure we stay in teleop mode
            follower.startTeleopDrive();
            // Check timeout
            if (autoAlignTimer.seconds() > AUTO_ALIGN_TIMEOUT) {
                stopAutoAlign("Timeout reached (" + String.format("%.1f", AUTO_ALIGN_TIMEOUT) + "s)");
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
                // Set RGB LED to RED when no tag visible
                if (rgb != null) {
                    try {
                        rgb.setPosition(RGB_RED);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to set RGB LED to RED: " + e.getMessage());
                    }
                }
                // Don't stop auto-align, just wait for tag to appear
                autoAlignRotationCommand = 0.0;
                // Apply zero rotation (no movement) - always apply
                follower.setTeleOpDrive(0, 0, 0, false);
                return;
            }
            
            // Read Limelight tx (horizontal offset in degrees)
            // tx < 0: Target to the left → rotate left (negative rotation)
            // tx > 0: Target to the right → rotate right (positive rotation)
            double rawTx = result.getTx();
            
            // Apply exponential smoothing to reduce noise and wobbling
            // Formula: smoothed = smoothing_factor * previous_smoothed + (1 - smoothing_factor) * new_value
            // Higher smoothing_factor = more smoothing (less responsive, less noisy)
            smoothedTx = AUTO_ALIGN_SMOOTHING_FACTOR * smoothedTx + (1.0 - AUTO_ALIGN_SMOOTHING_FACTOR) * rawTx;
            
            // Use smoothed tx for all calculations
            double tx = smoothedTx;
            
            // Update RGB LED based on alignment status (similar to RED_DEC_TELEOP)
            if (rgb != null) {
                try {
                    if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
                        // Green: Aligned within ±0.5 degrees
                        rgb.setPosition(RGB_GREEN);
                    } else {
                        // Orange: Tag visible but not aligned
                        rgb.setPosition(RGB_ORANGE);
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to set RGB LED: " + e.getMessage());
                }
            }
            
            // Check if heading is aligned (within tolerance of ±0.5 degrees)
            if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
                // Aligned! Stop rotation but keep mode active
                autoAlignRotationCommand = 0.0;
                // Apply zero rotation (no movement)
                follower.setTeleOpDrive(0, 0, 0, false);
                telemetry.addData("AUTO-ALIGN", "ALIGNED! (tx=" + String.format("%.2f", tx) + 
                    "°, raw=" + String.format("%.2f", rawTx) + "°)");
                return;
            }
            
            // Apply deadband: Don't rotate if within deadband range (prevents oscillation)
            // This creates a "settle zone" larger than tolerance to prevent wobbling
            if (Math.abs(tx) <= AUTO_ALIGN_DEADBAND) {
                // Within deadband - stop rotation to prevent oscillation
                autoAlignRotationCommand = 0.0;
                follower.setTeleOpDrive(0, 0, 0, false);
                telemetry.addData("AUTO-ALIGN", "Within deadband (tx=" + String.format("%.2f", tx) + 
                    "°), holding position");
                return;
            }
            
            // Calculate rotation speed based on smoothed tx value using proportional control
            // REVERSED: tx < 0 (target left) → positive rotation (rotate right)
            // REVERSED: tx > 0 (target right) → negative rotation (rotate left)
            // Negate tx to reverse rotation direction
            double rotationCommand = -tx * AUTO_ALIGN_TX_GAIN;
            
            // Clamp rotation speed to maximum
            rotationCommand = Math.max(-AUTO_ALIGN_ROTATION_SPEED, Math.min(AUTO_ALIGN_ROTATION_SPEED, rotationCommand));
            
            // Enforce minimum rotation to ensure movement (only if outside deadband)
            if (Math.abs(rotationCommand) < AUTO_ALIGN_MIN_ROTATE) {
                // Use negated tx sign to determine rotation direction if command is too small
                rotationCommand = Math.copySign(AUTO_ALIGN_MIN_ROTATE, -tx);
            }
            
            // Store rotation command for use in drive control
            autoAlignRotationCommand = rotationCommand;
            
            // Apply rotation using setTeleOpDrive
            // We've already ensured we're in teleop drive mode above
            follower.setTeleOpDrive(0, 0, rotationCommand, false);
            
            telemetry.addData("AUTO-ALIGN", "Aligning... (tx=" + String.format("%.2f", tx) + 
                "°, raw=" + String.format("%.2f", rawTx) + "°, rot=" + String.format("%.2f", rotationCommand) + 
                ", target: ±" + String.format("%.1f", AUTO_ALIGN_TX_TOLERANCE) + "°)");
        }
    }
    
    /**
     * Start auto-align mode
     * Called when shooting starts
     */
    private void startAutoAlign() {
        if (limelight == null) {
            telemetry.addData("AUTO-ALIGN", "ERROR: Limelight not available - cannot use auto-align");
            return;
        }
        
        // Reset smoothed tx value when starting auto-align
        smoothedTx = 0.0;
        
        // Get current Limelight result
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("AUTO-ALIGN", "WARNING: No valid Limelight data - cannot see AprilTag");
            telemetry.addData("AUTO-ALIGN", "Will start auto-align when AprilTag is detected");
            autoAlignMode = true;
            autoAlignTimer.reset();
            autoAlignRotationCommand = 0.0;
            return;
        }
        
        // Initialize smoothed tx with first reading
        double tx = result.getTx();
        smoothedTx = tx;
        
        // Check if already aligned
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
    }
    
    /**
     * Stop auto-align mode
     */
    private void stopAutoAlign(String reason) {
        autoAlignMode = false;
        autoAlignRotationCommand = 0.0;
        
        // Stop rotation - always apply
        follower.setTeleOpDrive(0, 0, 0, false);
        
        // Set RGB LED to RED when auto-align stops
        if (rgb != null) {
            try {
                rgb.setPosition(RGB_RED);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to set RGB LED to RED: " + e.getMessage());
            }
        }
        
        telemetry.addData("AUTO-ALIGN", "Mode stopped - " + reason);
    }
    
    // ========================================
    // UTILITY METHODS
    // ========================================
    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
    
    private void stopAllMotors() {
        stopIntake();
        if (feedMotor != null) feedMotor.setPower(0.0);
        stopShooter();
        stopAutoAlign("Autonomous stopped");
    }
    
    /**
     * Get current RGB LED status as string
     */
    private String getRGBStatus() {
        if (rgb == null) return "Not Available";
        if (limelight == null) return "RED (no Limelight)";
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return "RED (no target)";
        }
        
        double tx = result.getTx();
        if (Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE) {
            return "GREEN (aligned ±0.5°)";
        } else {
            return "ORANGE (tag visible, aligning)";
        }
    }
    
    // ========================================
    // TELEMETRY
    // ========================================
    private void updateTelemetry() {
        telemetry.clear();
        
        telemetry.addLine("=== RED DEC LONG AUTO ===");
        telemetry.addData("Drive Speed", String.format("%.0f%%", AUTONOMOUS_SPEED_MULTIPLIER * 100) + " (very conservative for accuracy)");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("X", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Y", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        
        telemetry.addLine("--- Auto-Align Status ---");
        telemetry.addData("Auto-Align", autoAlignMode ? "ACTIVE" : "INACTIVE");
        if (autoAlignMode) {
            telemetry.addData("Align Time", String.format("%.1fs", autoAlignTimer.seconds()));
            telemetry.addData("Align Rot Cmd", String.format("%.2f", autoAlignRotationCommand));
        }
        
        telemetry.addLine("--- Shooting Status ---");
        telemetry.addData("Shooter State", shooterState.toString());
        telemetry.addData("Balls Scored", ballsScored + "/" + TOTAL_BALLS_PER_ROUND);
        telemetry.addData("Shooting Complete", shootingComplete);
        telemetry.addData("Target Velocity", String.format("%.0f RPM", currentTargetVelocity));
        
        if (shooterMotorLeft != null && shooterMotorRight != null) {
            try {
                double leftVel = Math.abs(shooterMotorLeft.getVelocity());
                double rightVel = Math.abs(shooterMotorRight.getVelocity());
                telemetry.addData("Shooter L Vel", String.format("%.1f RPM", leftVel));
                telemetry.addData("Shooter R Vel", String.format("%.1f RPM", rightVel));
            } catch (Exception e) {
                telemetry.addData("Shooter Error", e.getMessage());
            }
        }
        
        telemetry.addLine("--- Intake Status ---");
        if (intakeMotor != null) {
            telemetry.addData("Intake Power", String.format("%.2f", intakeMotor.getPower()));
        }
        if (feedMotor != null) {
            telemetry.addData("Feed Power", String.format("%.2f", feedMotor.getPower()));
        }
        
        telemetry.addLine("--- Limelight Status ---");
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("Limelight", "Target Detected");
                telemetry.addData("TX (degrees)", String.format("%.2f", tx));
                telemetry.addData("Aligned", Math.abs(tx) <= AUTO_ALIGN_TX_TOLERANCE ? "YES (±0.5°)" : "NO");
                telemetry.addData("RGB LED", getRGBStatus());
            } else {
                telemetry.addData("Limelight", "No Target");
                telemetry.addData("RGB LED", "RED (no target)");
            }
        } else {
            telemetry.addData("Limelight", "Not Available");
            telemetry.addData("RGB LED", "RED (not available)");
        }
        
        telemetry.update();
    }
}

