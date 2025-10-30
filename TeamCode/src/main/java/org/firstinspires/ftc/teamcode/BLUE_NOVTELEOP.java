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

import java.util.List;

@TeleOp(name = "BLUE_NOVTELEOP", group = "TeleOp")
public class BLUE_NOVTELEOP extends LinearOpMode {

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

    // Launcher state machine - simplified
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();
    
    // R2 pulsed feeding mode tracking
    private boolean r2ThreeSecondMode = false;
    private boolean r2Pressed = false;
    private ElapsedTime r2ServoTimer = new ElapsedTime();
    private boolean r2VelocityReached = false;
    
    // Pulsed feeding constants
    private final double R2_BALL_FEED_TIME = 0.1; // Feed each ball for 0.05 seconds
    private final double R2_BALL_PAUSE_TIME = 1; // Pause between balls for 1.0 seconds
    private final double R2_MOTOR_STOP_TIME = 0.8; // Brief motor stop time for recovery (increased)
    private final int R2_BALL_COUNT = 3; // Shoot exactly 3 balls
    private int r2BallsShot = 0; // Track how many balls have been shot
    private boolean r2FeedingBall = false; // Track if currently feeding a ball
    private boolean r2WaitingForVelocity = false; // Track if waiting for velocity recovery
    private boolean r2MotorStopped = false; // Track if motor is briefly stopped for recovery
    
    private ElapsedTime r2VelocityCheckTimer = new ElapsedTime();
    
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
    
    // Shooting mode tracking
    private enum ShootingMode { DISTANCE_BASED }
    private ShootingMode currentShootingMode;
    
    // Preset locations (Blue side from NovTeleOpBlueSemiAuto)
    private final Pose closeRangePose = new Pose(61.808, 97.534, Math.toRadians(140));
    private final Pose longRangePose = new Pose(63.781, 19.288, Math.toRadians(115));
    private final Pose homePose = new Pose(125.808, 18.411, Math.toRadians(0));
    
    // Path following state
    private boolean automatedDrive = false;
    private PathChain currentPath;

    @Override
    public void runOpMode() throws InterruptedException {
        launchState = LaunchState.IDLE;
        currentShootingMode = ShootingMode.DISTANCE_BASED; 
        
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        
        boolean driveMotorsInitialized = true;
        try { frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); if (frontLeft == null) { telemetry.addData("ERROR", "Front left motor 'frontLeft' not found in hardware map!"); driveMotorsInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize front left motor: "+e.getMessage()); driveMotorsInitialized=false; }
        try { frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); if (frontRight == null) { telemetry.addData("ERROR", "Front right motor 'frontRight' not found in hardware map!"); driveMotorsInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize front right motor: "+e.getMessage()); driveMotorsInitialized=false; }
        try { backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); if (backLeft == null) { telemetry.addData("ERROR", "Back left motor 'backLeft' not found in hardware map!"); driveMotorsInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize back left motor: "+e.getMessage()); driveMotorsInitialized=false; }
        try { backRight = hardwareMap.get(DcMotorEx.class, "backRight"); if (backRight == null) { telemetry.addData("ERROR", "Back right motor 'backRight' not found in hardware map!"); driveMotorsInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize back right motor: "+e.getMessage()); driveMotorsInitialized=false; }
        if (!driveMotorsInitialized) telemetry.addData("CRITICAL ERROR", "Drive motors failed to initialize - robot cannot move!");
        
        boolean hardwareInitialized = true;
        try { shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor"); if (shooterMotor == null) { telemetry.addData("ERROR", "Shooter motor 'shooterMotor' not found in hardware map!"); hardwareInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize shooter motor: "+e.getMessage()); hardwareInitialized=false; }
        try { leftServo = hardwareMap.get(CRServo.class, "leftServo"); if (leftServo == null) { telemetry.addData("ERROR", "Left servo 'leftServo' not found in hardware map!"); hardwareInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize left servo: "+e.getMessage()); hardwareInitialized=false; }
        try { rightServo = hardwareMap.get(CRServo.class, "rightServo"); if (rightServo == null) { telemetry.addData("ERROR", "Right servo 'rightServo' not found in hardware map!"); hardwareInitialized=false; } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize right servo: "+e.getMessage()); hardwareInitialized=false; }
        
        try { rgb = hardwareMap.get(Servo.class, "rgb"); if (rgb == null) { telemetry.addData("ERROR", "RGB indicator 'rgb' not found in hardware map!"); } else { rgb.setPosition(RGB_RED); } } catch (Exception e) { telemetry.addData("ERROR", "Failed to initialize RGB indicator: "+e.getMessage()); }
        
        telemetry.addData("DEBUG", "Hardware mapping status:");
        telemetry.addData("DEBUG", "  Shooter Motor: "+(shooterMotor!=null?"FOUND":"MISSING"));
        telemetry.addData("DEBUG", "  Left Servo: "+(leftServo!=null?"FOUND":"MISSING"));
        telemetry.addData("DEBUG", "  Right Servo: "+(rightServo!=null?"FOUND":"MISSING"));
        telemetry.addData("DEBUG", "  RGB Indicator: "+(rgb!=null?"FOUND":"MISSING"));
        telemetry.addData("DEBUG", "  Overall Status: "+(hardwareInitialized?"SUCCESS":"FAILED"));
        
        if (!hardwareInitialized) telemetry.addData("WARNING", "Some hardware failed to initialize - launcher may not work properly");
        
        try { limelight = hardwareMap.get(Limelight3A.class, "limelight"); telemetry.addData("DEBUG", "Limelight: "+(limelight!=null?"FOUND":"MISSING")); } catch (Exception e) { telemetry.addData("ERROR", "Limelight initialization failed: "+e.getMessage()); }
        
        if (frontRight != null) frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontLeft != null) frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (shooterMotor != null) shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightServo != null) rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftServo != null) leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shooterMotor != null) shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shooterMotor != null) { try { shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(3,0,0,10)); } catch (Exception ignored) {} }
        if (leftServo != null) leftServo.setPower(0);
        if (rightServo != null) rightServo.setPower(0);
        
        try { follower = Constants.createFollower(hardwareMap); if (follower == null) { telemetry.addData("ERROR", "Failed to create PedroPathing follower!"); } else { telemetry.addData("DEBUG", "PedroPathing follower created successfully"); } } catch (Exception e) { telemetry.addData("ERROR", "PedroPathing initialization failed: "+e.getMessage()); follower = null; }
        if (follower != null) { try { follower.setStartingPose(new Pose(62.904, 38.795, Math.toRadians(180))); telemetry.addData("DEBUG", "Starting pose set successfully"); } catch (Exception e) { telemetry.addData("ERROR", "Failed to set starting pose: "+e.getMessage()); } }
        
        if (limelight != null) { telemetry.setMsTransmissionInterval(11); limelight.pipelineSwitch(0); limelight.start(); telemetry.addData("Status", "Limelight MegaTag2 initialized and started"); telemetry.addData("Limelight Status", "READY"); telemetry.addData("Pipeline", "0 (MegaTag2)"); }
        else telemetry.addData("ERROR", "Limelight not found - vision localization disabled");
        
        telemetry.addData("Status", "BLUE_NOVTELEOP Ready!");
        telemetry.addData("Localization", "PedroPathing + Limelight Vision");
        telemetry.addData("Control Mode", "Field-Oriented Control");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            
            // Preset locations
            handlePresetLocationButtons();
            
            boolean rightBumperPressed = gamepad1.rightBumperWasPressed();
            boolean r2CurrentlyPressed = gamepad1.right_trigger > 0.1;
            boolean r2JustPressed = r2CurrentlyPressed && !r2Pressed;
            boolean l1CurrentlyPressed = gamepad1.left_bumper;
            boolean l1JustPressed = l1CurrentlyPressed && !l1Pressed;
            
            if (rightBumperPressed) { currentShootingMode = ShootingMode.DISTANCE_BASED; }
            if (r2JustPressed) { startR2ThreeSecondMode(); } else if (r2ThreeSecondMode) { handleR2ThreeSecondMode(); }
            if (l1JustPressed) { startL1ScopeMode(); } else if (l1ScopeMode) { handleL1ScopeMode(); }
            r2Pressed = r2CurrentlyPressed; l1Pressed = l1CurrentlyPressed;
            
            launch(rightBumperPressed);
            
            // Blue-side field-centric control reversal
            // For PedroPathing: pass RAW inputs (PedroPathing handles field-centric internally)
            // For manual fallback: process inputs with Blue-side inversions
            double y = applyAdvancedDeadzone(gamepad1.left_stick_y);
            double x = applyAdvancedDeadzone(-gamepad1.left_stick_x);
            double rx = applyAdvancedDeadzone(gamepad1.right_stick_x); // Fixed: removed negation for correct rotation direction
            y = Math.copySign(y * y, y);
            x = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);
            
            if (follower != null) {
                try {
                    follower.update();
                    if (!automatedDrive) {
                        if (l1ScopeMode) {
                            follower.setTeleOpDrive(0.0, 0.0, scopeRotationCommand, false);
                        } else {
                            // Fixed: Pass RAW gamepad inputs to PedroPathing (matches NovTeleOpBlueSemiAuto exactly)
                            // PedroPathing handles field-centric transformation internally
                            follower.setTeleOpDrive(
                                gamepad1.left_stick_y,   // Forward/backward (raw input)
                                gamepad1.left_stick_x,   // Strafe left/right (raw input - matches NovTeleOpBlueSemiAuto)
                                -gamepad1.right_stick_x, // Rotation (negated - matches NovTeleOpBlueSemiAuto)
                                false // Field-centric mode
                            );
                        }
                        double flPower = (frontLeft != null) ? frontLeft.getPower() : 0.0;
                        double frPower = (frontRight != null) ? frontRight.getPower() : 0.0;
                        double blPower = (backLeft != null) ? backLeft.getPower() : 0.0;
                        double brPower = (backRight != null) ? backRight.getPower() : 0.0;
                        updateTelemetry(flPower, frPower, blPower, brPower, follower.getPose().getHeading());
                    } else {
                        updateTelemetry(0, 0, 0, 0, follower.getPose().getHeading());
                    }
                    if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                        follower.startTeleopDrive();
                        automatedDrive = false;
                        currentPath = null;
                    }
                } catch (Exception e) {
                    automatedDrive = false;
                    manualDriveControl(x, y, rx);
                    updateTelemetry(0, 0, 0, 0, 0);
                }
            } else {
                manualDriveControl(x, y, rx);
                updateTelemetry(0, 0, 0, 0, 0);
            }
        }
    }

    // The following methods are copied 1:1 from RED_NOVTELEOP to maintain identical behavior.
    // START copy

    void launch(boolean shotRequested) {
        if (shooterMotor == null) { telemetry.addData("ERROR", "Shooter motor not initialized - cannot launch"); return; }
        double currentTargetVelocity = getCurrentTargetVelocity();
        double currentMinVelocity = getCurrentMinVelocity();
        switch (launchState) {
            case IDLE:
                if (shotRequested) { launchState = LaunchState.SPIN_UP; feederTimer.reset(); }
                break;
            case SPIN_UP:
                try {
                    shooterMotor.setVelocity(currentTargetVelocity);
                    double currentVel = Math.abs(shooterMotor.getVelocity());
                    double effectiveTargetVelocity = currentTargetVelocity * 0.95;
                    if (currentVel >= effectiveTargetVelocity) { launchState = LaunchState.LAUNCH; }
                    else if (feederTimer.seconds() > 5.0) { launchState = LaunchState.LAUNCH; }
                } catch (Exception e) { telemetry.addData("ERROR", "setVelocity() failed: "+e.getMessage()); launchState = LaunchState.IDLE; }
                break;
            case LAUNCH:
                if (leftServo != null && rightServo != null) {
                    leftServo.setPower(FULL_SPEED); rightServo.setPower(FULL_SPEED); feederTimer.reset(); launchState = LaunchState.LAUNCHING;
                } else { launchState = LaunchState.IDLE; }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    if (leftServo != null) leftServo.setPower(STOP_SPEED);
                    if (rightServo != null) rightServo.setPower(STOP_SPEED);
                    if (!r2ThreeSecondMode) { try { shooterMotor.setVelocity(0); } catch (Exception ignored) {} }
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    private double getCurrentTargetVelocity() { return calculateDynamicVelocity(); }
    private double getCurrentMinVelocity() { return currentCalculatedMinVelocity; }

    private void startR2ThreeSecondMode() {
        if (launchState != LaunchState.IDLE) launchState = LaunchState.IDLE;
        r2ThreeSecondMode = true; r2VelocityReached = false; r2BallsShot = 0; r2FeedingBall=false; r2WaitingForVelocity=false; r2MotorStopped=false; r2ServoTimer.reset(); r2VelocityCheckTimer.reset(); currentShootingMode = ShootingMode.DISTANCE_BASED;
        if (shooterMotor != null) { try { shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); shooterMotor.setVelocity(getCurrentTargetVelocity()); } catch (Exception ignored) {} }
    }

    private void handleR2ThreeSecondMode() {
        if (!r2ThreeSecondMode) return;
        if (r2VelocityCheckTimer.seconds() >= 1.0) {
            r2VelocityCheckTimer.reset();
            if (shooterMotor != null) {
                double currentVel = Math.abs(shooterMotor.getVelocity());
                double targetVel = getCurrentTargetVelocity();
                double velocityTolerance = 100;
                if (currentVel < (targetVel - velocityTolerance)) { try { shooterMotor.setVelocity(targetVel); } catch (Exception ignored) {} }
            }
        }
        if (!r2VelocityReached) {
            double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
            double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
            if (currentVel >= effectiveTargetVelocity) { r2VelocityReached = true; r2ServoTimer.reset(); }
            return;
        }
        if (r2BallsShot < R2_BALL_COUNT) {
            if (!r2FeedingBall && !r2WaitingForVelocity && !r2MotorStopped) {
                if (r2BallsShot == 2 && r2ServoTimer.seconds() < 0.35) { return; }
                double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
                double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
                if (currentVel >= effectiveTargetVelocity) {
                    if (leftServo != null && rightServo != null) { leftServo.setPower(FULL_SPEED); rightServo.setPower(FULL_SPEED); r2FeedingBall=true; r2ServoTimer.reset(); }
                    else { stopR2ThreeSecondMode("Servo error"); }
                } else { r2WaitingForVelocity = true; r2ServoTimer.reset(); }
            } else if (r2WaitingForVelocity) {
                double currentVel = shooterMotor != null ? Math.abs(shooterMotor.getVelocity()) : 0;
                double effectiveTargetVelocity = getCurrentTargetVelocity() * 0.95;
                if (currentVel >= effectiveTargetVelocity) { r2WaitingForVelocity=false; }
            } else if (r2FeedingBall) {
                if (r2ServoTimer.seconds() >= R2_BALL_FEED_TIME) {
                    if (leftServo != null) leftServo.setPower(STOP_SPEED);
                    if (rightServo != null) rightServo.setPower(STOP_SPEED);
                    r2FeedingBall=false; r2BallsShot++; r2ServoTimer.reset();
                    if (r2BallsShot < R2_BALL_COUNT) { r2MotorStopped=true; if (shooterMotor != null) { try { shooterMotor.setVelocity(0); } catch (Exception ignored) {} } }
                }
            } else if (r2MotorStopped) {
                if (r2ServoTimer.seconds() >= R2_MOTOR_STOP_TIME) {
                    r2MotorStopped=false; if (shooterMotor != null) { try { shooterMotor.setVelocity(getCurrentTargetVelocity()); } catch (Exception ignored) {} } r2ServoTimer.reset();
                }
            }
        } else { if (r2ServoTimer.seconds() >= R2_BALL_PAUSE_TIME) { stopR2ThreeSecondMode("All balls shot"); } }
    }

    private void stopR2ThreeSecondMode(String reason) {
        r2ThreeSecondMode=false; r2VelocityReached=false; r2BallsShot=0; r2FeedingBall=false; r2WaitingForVelocity=false; r2MotorStopped=false;
        if (leftServo != null) leftServo.setPower(STOP_SPEED);
        if (rightServo != null) rightServo.setPower(STOP_SPEED);
        if (shooterMotor != null) { try { shooterMotor.setVelocity(0); } catch (Exception ignored) {} }
        launchState = LaunchState.IDLE;
    }

    private void startL1ScopeMode() {
        if (limelight == null) { scopeDebugActive=false; scopeDebugStatus="ERROR: Limelight not available"; return; }
        automatedDrive=false; currentPath=null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { scopeDebugActive=false; scopeDebugStatus="ERROR: No valid AprilTag data"; return; }
        double ty = result.getTy();
        if (Math.abs(ty) <= SCOPE_TY_TOLERANCE) { l1ScopeMode=true; scopeTimer.reset(); scopeDebugActive=true; scopeDebugTy=ty; scopeDebugRotation=0.0; scopeDebugTime=0.0; scopeDebugStatus="ALIGNED at start"; return; }
        l1ScopeMode=true; scopeTimer.reset(); scopeDebugActive=true; scopeDebugTy=ty; scopeDebugRotation=0.0; scopeDebugTime=0.0; scopeDebugStatus="STARTED";
    }

    private void handleL1ScopeMode() {
        if (!l1ScopeMode) return;
        if (scopeTimer.seconds() > SCOPE_TIMEOUT) { stopL1ScopeMode("Timeout"); return; }
        if (limelight == null) return;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        double ty = result.getTy();
        if (Math.abs(ty) <= SCOPE_TY_TOLERANCE) { stopL1ScopeMode("Aligned"); return; }
        double rotationCommand = -ty * SCOPE_TY_GAIN;
        rotationCommand = Math.max(-SCOPE_ROTATION_SPEED, Math.min(SCOPE_ROTATION_SPEED, rotationCommand));
        double minRotate = 0.05; if (Math.abs(rotationCommand) < minRotate) rotationCommand = Math.copySign(minRotate, rotationCommand == 0.0 ? ty : rotationCommand);
        scopeRotationCommand = rotationCommand; scopeDebugActive=true; scopeDebugTy=ty; scopeDebugRotation=rotationCommand; scopeDebugTime=scopeTimer.seconds(); scopeDebugStatus="ALIGNING (ty)";
    }

    private void stopL1ScopeMode(String reason) { l1ScopeMode=false; scopeRotationCommand=0.0; }

    private double calculateDynamicVelocity() {
        if (limelight == null) return SHORT_DISTANCE_VELOCITY;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return SHORT_DISTANCE_VELOCITY;
        double ta = result.getTa(); if (ta <= 0.0) return SHORT_DISTANCE_VELOCITY;
        double distanceCalibrationFactor = 50.0;
        double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1));
        double appliedMultiplier;
        if (baseDistance < 40.0) appliedMultiplier = 1.424;
        else if (baseDistance < 90.0) appliedMultiplier = 1.42 + (baseDistance - 40.0) * (-0.00212);
        else appliedMultiplier = Math.max(0.95, 1.333 + (baseDistance - 81.0) * (-0.0030));
        double actualDistance = Math.max(SHORT_DISTANCE_INCHES, Math.min(LONG_DISTANCE_INCHES, baseDistance * appliedMultiplier));
        double t = (actualDistance - SHORT_DISTANCE_INCHES) / (LONG_DISTANCE_INCHES - SHORT_DISTANCE_INCHES);
        double calculatedVelocity = SHORT_DISTANCE_VELOCITY + (LONG_DISTANCE_VELOCITY - SHORT_DISTANCE_VELOCITY) * t;
        calculatedVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, calculatedVelocity));
        double calculatedMinVelocity = Math.max(MIN_VELOCITY * 0.95, Math.min(MAX_VELOCITY * 0.95, calculatedVelocity * 0.95));
        currentCalculatedVelocity = calculatedVelocity; currentCalculatedMinVelocity = calculatedMinVelocity; return calculatedVelocity;
    }

    private double applyAdvancedDeadzone(double value) { if (Math.abs(value) < JOYSTICK_DEADZONE) return 0.0; return Math.signum(value) * ((Math.abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE)); }
    private double applyMinPower(double power) { if (Math.abs(power) > 0 && Math.abs(power) < MIN_MOTOR_POWER) return Math.signum(power) * MIN_MOTOR_POWER; return power; }

    private void updateTelemetry(double flPower, double frPower, double blPower, double brPower, double botHeading) {
        telemetry.clear();
        telemetry.addLine("=== BLUE NOVTELEOP ===");
        telemetry.addData("Limelight", limelight != null ? "CONNECTED" : "DISCONNECTED");
        if (follower != null) { telemetry.addData("X", String.format("%.2f", follower.getPose().getX())); telemetry.addData("Y", String.format("%.2f", follower.getPose().getY())); telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading()))); }
        double distanceInches = 0.0; boolean tagVisible = false; double tyDisplay = Double.NaN;
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tagVisible = true; double ta = result.getTa(); tyDisplay = result.getTy();
                if (rgb != null) { try { double desired = RGB_ORANGE; if (Math.abs(tyDisplay) <= 1.0) desired = RGB_GREEN; rgb.setPosition(desired); } catch (Exception ignored) {} }
                if (ta > 0.0) { double distanceCalibrationFactor = 50.0; double baseDistance = distanceCalibrationFactor / Math.sqrt(Math.max(ta, 0.1)); double appliedMultiplier; if (baseDistance < 40.0) appliedMultiplier = 1.424; else if (baseDistance < 90.0) { double slope = -0.00212; appliedMultiplier = 1.42 + (baseDistance - 40.0) * slope; } else { double slope = -0.0030; appliedMultiplier = Math.max(0.95, 1.333 + (baseDistance - 81.0) * slope); } distanceInches = baseDistance * appliedMultiplier; }
            } else { if (rgb != null) { try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {} } }
        } else { if (rgb != null) { try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {} } }
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        if (!Double.isNaN(tyDisplay)) telemetry.addData("Limelight ty", String.format("%.2f°", tyDisplay)); else { if (rgb != null) { try { rgb.setPosition(RGB_RED); } catch (Exception ignored) {} } }
        telemetry.addData("Distance to Tag", String.format("%.1f in", distanceInches));
        if (scopeDebugActive || l1ScopeMode) { telemetry.addLine("--- SCOPE DEBUG ---"); telemetry.addData("Active", l1ScopeMode ? "YES" : "NO"); if (!Double.isNaN(scopeDebugTy)) telemetry.addData("ty", String.format("%.2f°", scopeDebugTy)); telemetry.addData("rot", String.format("%.2f", scopeDebugRotation)); telemetry.addData("time", String.format("%.2fs", scopeDebugTime)); telemetry.addData("status", scopeDebugStatus); }
        double targetVelocity = getCurrentTargetVelocity(); telemetry.addData("Trigger Velocity", String.format("%.0f RPM", targetVelocity)); if (shooterMotor != null) telemetry.addData("Shooter Velocity", String.format("%.1f RPM", Math.abs(shooterMotor.getVelocity())));
        telemetry.update();
    }

    private void manualDriveControl(double x, double y, double rx) {
        // Fixed: Blue-side field-centric transformation
        // Note: x input is already inverted for Blue side when passed to this method
        // This method does robot-to-field coordinate transformation
        double botHeading = follower != null ? follower.getPose().getHeading() : 0.0;
        // Standard field-centric rotation (Blue side inversions are in the input, not here)
        double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
        x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
        y = temp;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        flPower = applyMinPower(flPower); blPower = applyMinPower(blPower); frPower = applyMinPower(frPower); brPower = applyMinPower(brPower);
        flPower *= DRIVE_POWER_MULTIPLIER; blPower *= DRIVE_POWER_MULTIPLIER; frPower *= DRIVE_POWER_MULTIPLIER; brPower *= DRIVE_POWER_MULTIPLIER;
        flPower = Math.max(-1.0, Math.min(1.0, flPower)); blPower = Math.max(-1.0, Math.min(1.0, blPower)); frPower = Math.max(-1.0, Math.min(1.0, frPower)); brPower = Math.max(-1.0, Math.min(1.0, brPower));
        if (frontLeft != null) frontLeft.setPower(flPower);
        if (backLeft != null) backLeft.setPower(blPower);
        if (frontRight != null) frontRight.setPower(frPower);
        if (backRight != null) backRight.setPower(brPower);
    }

    private void handlePresetLocationButtons() {
        if (gamepad1.xWasPressed()) { goToPresetLocation(closeRangePose, "Close Range Scoring"); }
        else if (gamepad1.yWasPressed()) { goToPresetLocation(longRangePose, "Long Range Scoring"); }
        else if (gamepad1.aWasPressed()) { goToPresetLocation(homePose, "Home Position"); }
    }

    private void goToPresetLocation(Pose targetPose, String locationName) {
        if (follower == null) { telemetry.addData("ERROR", "Cannot navigate to "+locationName+" - PedroPathing not available"); return; }
        try {
            if (automatedDrive) { follower.startTeleopDrive(); automatedDrive=false; currentPath=null; }
            Pose currentPose = follower.getPose();
            currentPath = follower.pathBuilder().addPath(new BezierLine(currentPose, targetPose)).setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading()).build();
            follower.followPath(currentPath); automatedDrive=true;
        } catch (Exception e) { telemetry.addData("ERROR", "Failed to navigate: "+e.getMessage()); automatedDrive=false; currentPath=null; }
    }

    private Pose getRobotPoseFromCamera(LLResult result) {
        try {
            Pose3D botpose = result.getBotpose(); if (botpose == null) return null;
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return null;
            String poseString = botpose.toString();
            double x = 0.0, y = 0.0, heading = 0.0;
            try {
                String[] parts = poseString.replaceAll("[{}]", "").split(",");
                for (String part : parts) {
                    String[] keyValue = part.split("=");
                    if (keyValue.length == 2) {
                        String key = keyValue[0].trim();
                        double value = Double.parseDouble(keyValue[1].trim());
                        switch (key) { case "x": x = value; break; case "y": y = value; break; case "yaw": heading = value; break; }
                    }
                }
            } catch (Exception parseException) { return null; }
            Pose ftcPose = new Pose(x, y, heading, FTCCoordinates.INSTANCE);
            Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            Pose currentPose = follower.getPose();
            double distanceFromCurrent = Math.sqrt(Math.pow(pedroPose.getX() - currentPose.getX(), 2) + Math.pow(pedroPose.getY() - currentPose.getY(), 2));
            if (distanceFromCurrent > 50.0) return null;
            double fusionWeight = 1.0;
            double fusedX = currentPose.getX() * (1 - fusionWeight) + pedroPose.getX() * fusionWeight;
            double fusedY = currentPose.getY() * (1 - fusionWeight) + pedroPose.getY() * fusionWeight;
            double fusedHeading = currentPose.getHeading() * (1 - fusionWeight) + pedroPose.getHeading() * fusionWeight;
            return new Pose(fusedX, fusedY, fusedHeading);
        } catch (Exception e) { return null; }
    }
}
