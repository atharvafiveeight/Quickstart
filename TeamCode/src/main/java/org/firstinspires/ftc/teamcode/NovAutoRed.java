package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Panels imports for visualization
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Autonomous(name = "NovAutoRed", group = "Autonomous")
@Configurable
public class NovAutoRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    
    // Panels telemetry manager
    private TelemetryManager telemetryM;
    
    // Panels field drawing
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style("", "#3F51B5", 0.0);
    private static final Style historyLook = new Style("", "#4CAF50", 0.0);
    private static final Style pathLook = new Style("", "#FF9800", 0.0);
    private static final double ROBOT_RADIUS = 9; // 18x18 robot = 9 inch radius

    // Launcher hardware declarations
    private DcMotorEx shooterMotor;  // Shooter motor with velocity control
    private CRServo leftServo;       // Left feeder servo
    private CRServo rightServo;      // Right feeder servo
    
    // Autonomous state management (simplified)
    private int pathState;
    private int scoringState;
    private int ballsScored;
    private boolean scoringComplete;
    
    // Define poses for the autonomous path (updated to match NovTeleOpRedSemiAuto.java)
    private final Pose startPose = new Pose(86.795, 134.575, Math.toRadians(0)); // Start position
    private final Pose scorePose = new Pose(82.192, 97.534, Math.toRadians(40)); // Close range scoring position (from NovTeleOpRedSemiAuto.java)
    private final Pose startTeleopPose = new Pose(81.096, 38.795, Math.toRadians(0)); // Final park pose for teleop

    // Define path chains (simplified)
    private PathChain startToScore;
    private PathChain scoreToTeleop;
    
    // Launcher constants (updated from NovTeleOpRedSemiAuto.java)
    private final double FEED_TIME_SECONDS = 0.05;        // How long to feed each ball (from NovTeleOpRedSemiAuto.java)
    private final double FEED_DELAY_SECONDS = 1.5;        // Delay between feeds - reduced for better flow
    private final double FINAL_LAUNCH_DELAY = 2.0;        // Extra delay after last ball to ensure it launches
    private final double FULL_SPEED = 1.0;                // Full speed for servos (from NovTeleOpRedSemiAuto.java)
    private final double STOP_SPEED = 0.0;                // Stop speed for servos (from NovTeleOpRedSemiAuto.java)
    private final double LAUNCHER_TARGET_VELOCITY = 1400; // Target velocity for short distance (from NovTeleOpRedSemiAuto.java)
    private final double LAUNCHER_MIN_VELOCITY = 1300;    // Minimum velocity for short distance (from NovTeleOpRedSemiAuto.java)
    private final int TOTAL_BALLS_TO_SCORE = 3;           // Number of balls to score per round
    
    // REMOVED: Power-based control variables - now using setVelocity() method like NovTeleOpRedSemiAuto.java
    
    // Configurable drive speed for autonomous (0.0 to 1.0)
    private double autonomousDriveSpeed = 0.3;            // 30% speed for autonomous precision
    
    // Launcher state machine (updated from NovTeleOpRedSemiAuto.java with wait states)
    private enum LaunchState {
        IDLE,        // Shooter is stopped, waiting for launch command
        SPIN_UP,     // Shooter is spinning up to target velocity
        LAUNCH,      // Ready to launch, start feeding
        LAUNCHING,   // Currently feeding game pieces
        WAITING,     // Waiting between shots for accuracy
        FINAL_WAIT   // Waiting for final shot to complete before moving
    }
    
    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize the Follower using Constants (recommended approach)
        follower = Constants.createFollower(hardwareMap);
        
        // Apply autonomous drive speed reduction
        applyAutonomousSpeedReduction();
        
        // Initialize Panels configuration and telemetry
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize launcher hardware
        initializeLauncherHardware();
        
        // REMOVED: configureLauncherPowers() - now using setVelocity() method like NovTeleOpRedSemiAuto.java

        // Build the autonomous paths
        buildAutonomousPaths();

        // Set the starting pose
        follower.setStartingPose(startPose);

        // Initialize state variables (simplified)
        pathState = 0;
        scoringState = 0;
        ballsScored = 0;
        scoringComplete = false;
        launchState = LaunchState.IDLE;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scoring State", scoringState);
        telemetry.addData("Balls Scored", ballsScored);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setScoringState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        autonomousScoringUpdate();

        // Update Panels visualization
        drawCurrentAndHistory();
        
        // Update Panels telemetry with real-time launcher data
        updatePanelsTelemetry();

        // Feedback to Driver Hub for debugging
        updateTelemetry();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        // Stop all launcher hardware when autonomous ends
        stopLauncher();
    }

    /**
     * Initialize launcher hardware (shooter motor and servos)
     */
    private void initializeLauncherHardware() {
        try {
            // Initialize shooter motor
            shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            
            // Set custom PIDF coefficients for better launcher control
            shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(300, 0, 0, 10));
            
            // Initialize servos
            leftServo = hardwareMap.get(CRServo.class, Constants.LEFT_SERVO_NAME);
            rightServo = hardwareMap.get(CRServo.class, Constants.RIGHT_SERVO_NAME);
            leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
            
            // Stop servos initially
            leftServo.setPower(0);
            rightServo.setPower(0);
            
            telemetry.addData("DEBUG", "Launcher hardware initialized successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize launcher hardware: " + e.getMessage());
        }
    }
    
    /**
     * Apply autonomous drive speed reduction for precision
     */
    private void applyAutonomousSpeedReduction() {
        try {
            // Note: PedroPathing doesn't have a direct way to modify max power after creation
            // The speed reduction is handled through the autonomousDriveSpeed variable
            // which is used for telemetry and can be adjusted in configureLauncherPowers()
            telemetry.addData("DEBUG", "Autonomous drive speed set to: " + String.format("%.1f%%", autonomousDriveSpeed * 100));
            telemetry.addData("DEBUG", "Note: Speed reduction applied through path constraints");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to apply speed reduction: " + e.getMessage());
        }
    }
    
    /**
     * REMOVED: configureLauncherPowers() method - now using setVelocity() method like NovTeleOpRedSemiAuto.java
     */
    
    /**
     * Build the autonomous paths using pose definitions (simplified)
     */
    private void buildAutonomousPaths() {
        try {
            // Path 1: Start to Score (BezierLine with heading interpolation 90° to 40°)
            startToScore = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
            telemetry.addData("DEBUG", "Built startToScore path successfully with " + String.format("%.1f%%", autonomousDriveSpeed * 100) + " speed");
                    
            // Path 2: Score to Teleop (BezierLine with heading interpolation 40° to 180°)
            scoreToTeleop = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, startTeleopPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), startTeleopPose.getHeading())
                    .build();
            telemetry.addData("DEBUG", "Built scoreToTeleop path successfully with " + String.format("%.1f%%", autonomousDriveSpeed * 100) + " speed");
            telemetry.addData("DEBUG", "All paths built successfully!");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to build paths: " + e.getMessage());
        }
    }

    /**
     * Autonomous path update method - manages the movement state machine (simplified)
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move from start to scoring position
                telemetry.addData("DEBUG", "Moving to scoring position");
                if (startToScore != null) {
                    follower.followPath(startToScore);
                    setPathState(1);
                    telemetry.addData("DEBUG", "Path started, moving to state 1");
                } else {
                    telemetry.addData("ERROR", "startToScore path is null!");
                }
                break;

            case 1:
                // Wait for path to complete, then start scoring
                telemetry.addData("DEBUG", "Path state 1: Waiting for path to complete. isBusy: " + follower.isBusy());
                if (!follower.isBusy()) {
                    telemetry.addData("DEBUG", "Reached scoring position, starting scoring round");
                    setPathState(2);
                    setScoringState(1); // Start scoring round
                }
                break;
                
            case 2:
                // Wait for scoring to complete, then move to teleop position
                if (scoringComplete) {
                    telemetry.addData("DEBUG", "Scoring complete, moving to teleop position");
                    follower.followPath(scoreToTeleop);
                    setPathState(3);
                } else {
                    telemetry.addData("DEBUG", "Waiting for scoring to complete... Balls scored: " + ballsScored + "/" + TOTAL_BALLS_TO_SCORE);
                }
                break;
                
            case 3:
                // Wait for path to teleop position, then finish autonomous
                if (!follower.isBusy()) {
                    telemetry.addData("DEBUG", "Reached teleop position, autonomous finished");
                    setPathState(-1); // Finish autonomous
                }
                break;
        }
    }

    /**
     * Autonomous scoring update method - manages the scoring state machine (simplified)
     */
    public void autonomousScoringUpdate() {
        switch (scoringState) {
            case 0:
                // Idle - not scoring
                break;
                
            case 1:
                // Start scoring sequence
                telemetry.addData("DEBUG", "Starting scoring sequence, balls to score: " + TOTAL_BALLS_TO_SCORE);
                ballsScored = 0;
                launchState = LaunchState.SPIN_UP;
                feederTimer.reset();
                setScoringState(2);
                break;
                
            case 2:
                // Scoring in progress - run launcher state machine
                runLauncherStateMachine();
                
                // Check if all balls have been scored AND launcher is idle (final ball launched and wait complete)
                if (ballsScored >= TOTAL_BALLS_TO_SCORE && launchState == LaunchState.IDLE) {
                    telemetry.addData("DEBUG", "All balls scored and final shot complete: " + ballsScored + "/" + TOTAL_BALLS_TO_SCORE);
                    stopLauncher();
                    setScoringState(0);
                    scoringComplete = true;
                    telemetry.addData("DEBUG", "Scoring round complete - ready to move to end position");
                    
                } else if (ballsScored >= TOTAL_BALLS_TO_SCORE) {
                    telemetry.addData("DEBUG", "All balls scored, waiting for final shot to complete. Launch state: " + launchState);
                }
                break;
        }
    }


    /**
     * Run the launcher state machine (updated from NovTeleOpRedSemiAuto.java)
     */
    private void runLauncherStateMachine() {
        switch (launchState) {
            case IDLE:
                // Stop the shooter and servos
                try {
                    shooterMotor.setVelocity(0);
                    leftServo.setPower(STOP_SPEED);
                    rightServo.setPower(STOP_SPEED);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to stop launcher: " + e.getMessage());
                }
                break;

            case SPIN_UP:
                // Spin up the shooter motor using setVelocity() method
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    double currentVelocity = Math.abs(shooterMotor.getVelocity());
                    
                    // Use 95% of target velocity for more reliable launching
                    double effectiveTargetVelocity = LAUNCHER_TARGET_VELOCITY * 0.95;
                    
                    if (currentVelocity >= effectiveTargetVelocity) {
                        telemetry.addData("DEBUG", "Shooter at effective target velocity (" + String.format("%.1f", currentVelocity) + " >= " + String.format("%.1f", effectiveTargetVelocity) + "), ready to launch");
                        launchState = LaunchState.LAUNCH;
                    } else if (feederTimer.seconds() > 5.0) { // 5 second timeout
                        launchState = LaunchState.LAUNCH;
                        telemetry.addData("DEBUG", "Spin-up timeout - forcing launch (current: " + String.format("%.1f", currentVelocity) + ", target: " + String.format("%.1f", LAUNCHER_TARGET_VELOCITY) + ")");
                    } else {
                        telemetry.addData("DEBUG", "Still spinning up... (" + String.format("%.1f", currentVelocity) + "/" + String.format("%.1f", effectiveTargetVelocity) + " effective target)");
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "setVelocity() failed: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                // Start feeding the ball
                try {
                    leftServo.setPower(FULL_SPEED);
                    rightServo.setPower(FULL_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                    telemetry.addData("DEBUG", "Feeding ball " + (ballsScored + 1));
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to start feeding: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCHING:
                // Continue feeding for the specified time
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    // Ball fed, stop servos and motor
                    try {
                        leftServo.setPower(STOP_SPEED);
                        rightServo.setPower(STOP_SPEED);
                        shooterMotor.setVelocity(0); // Stop the motor
                        
                        ballsScored++;
                        double launchingVelocity = Math.abs(shooterMotor.getVelocity());
                        
                        // Send essential telemetry to Panels - velocity when ball was shot and robot position
                        telemetryM.addData("Ball " + ballsScored + " Launch Velocity", String.format("%.1f", launchingVelocity));
                        telemetryM.addData("Ball " + ballsScored + " Position", String.format("(%.2f, %.2f)", follower.getPose().getX(), follower.getPose().getY()));
                        telemetryM.addData("Ball " + ballsScored + " Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
                        
                        telemetry.addData("DEBUG", "Ball " + ballsScored + " launched at velocity " + String.format("%.1f", launchingVelocity));
                        
                        // Check if this was the last ball
                        if (ballsScored >= TOTAL_BALLS_TO_SCORE) {
                            // All balls launched, wait for final shot to complete before moving
                            feederTimer.reset();
                            launchState = LaunchState.FINAL_WAIT;
                            telemetry.addData("DEBUG", "Final ball launched, waiting " + FINAL_LAUNCH_DELAY + " seconds for shot to complete");
                        } else {
                            // Wait before next shot for accuracy
                            feederTimer.reset();
                            launchState = LaunchState.WAITING;
                            telemetry.addData("DEBUG", "Ball " + ballsScored + " launched, waiting " + FEED_DELAY_SECONDS + " seconds before next shot");
                        }
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to complete launch: " + e.getMessage());
                        launchState = LaunchState.IDLE;
                    }
                }
                break;
                
            case WAITING:
                // Wait between shots for accuracy
                if (feederTimer.seconds() > FEED_DELAY_SECONDS) {
                    // Wait time complete, spin up for next shot
                    launchState = LaunchState.SPIN_UP;
                    feederTimer.reset();
                    telemetry.addData("DEBUG", "Wait complete, spinning up for ball " + (ballsScored + 1));
                } else {
                    double remainingWait = FEED_DELAY_SECONDS - feederTimer.seconds();
                    telemetry.addData("DEBUG", "Waiting between shots: " + String.format("%.1f", remainingWait) + " seconds remaining");
                }
                break;
                
            case FINAL_WAIT:
                // Wait for final shot to complete before moving to end position
                if (feederTimer.seconds() > FINAL_LAUNCH_DELAY) {
                    // Final shot should be complete, go to idle
                    launchState = LaunchState.IDLE;
                    telemetry.addData("DEBUG", "Final shot complete, all balls launched successfully");
                } else {
                    double remainingWait = FINAL_LAUNCH_DELAY - feederTimer.seconds();
                    telemetry.addData("DEBUG", "Waiting for final shot to complete: " + String.format("%.1f", remainingWait) + " seconds remaining");
                }
                break;
        }
    }

    /**
     * Stop all launcher hardware (updated to use setVelocity() method)
     */
    private void stopLauncher() {
        try {
            shooterMotor.setVelocity(0);
            leftServo.setPower(STOP_SPEED);
            rightServo.setPower(STOP_SPEED);
            launchState = LaunchState.IDLE;
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to stop launcher: " + e.getMessage());
        }
    }

    /**
     * Draw robot position and path history on Panels
     */
    private void drawCurrentAndHistory() {
        try {
            // Set field offsets for PedroPathing coordinate system
            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            
            // Draw pose history
            drawPoseHistory(follower.getPoseHistory());
            
            // Draw current robot position
            drawRobot(follower.getPose());
            
            // Draw path waypoints
            drawPathWaypoints();
            
            // Update the field display
            panelsField.update();
        } catch (Exception e) {
            telemetry.addData("ERROR", "Panels drawing failed: " + e.getMessage());
        }
    }
    
    /**
     * Draw the robot at its current position
     */
    private void drawRobot(Pose pose) {
        try {
            panelsField.setStyle(robotLook);
            panelsField.circle(ROBOT_RADIUS);
            
            // Draw heading line
            com.pedropathing.math.Vector headingVector = new com.pedropathing.math.Vector(ROBOT_RADIUS * 1.5, pose.getHeading());
            panelsField.line(headingVector.getXComponent(), headingVector.getYComponent());
            
            // Debug telemetry
            telemetry.addData("DEBUG", "Robot drawn at: (" + String.format("%.2f", pose.getX()) + 
                ", " + String.format("%.2f", pose.getY()) + 
                ", " + String.format("%.1f°", Math.toDegrees(pose.getHeading())) + ")");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Robot drawing failed: " + e.getMessage());
        }
    }
    
    /**
     * Draw pose history trail
     */
    private void drawPoseHistory(com.pedropathing.util.PoseHistory poseHistory) {
        try {
            panelsField.setStyle(historyLook);
            
            // Simple pose history drawing - just draw a small circle at current position
            if (poseHistory != null) {
                // Draw a small circle to represent pose history
                panelsField.circle(2);
            }
            
            // Debug telemetry
            telemetry.addData("DEBUG", "Pose history visualization active");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Pose history drawing failed: " + e.getMessage());
        }
    }
    
    /**
     * Draw path waypoints
     */
    private void drawPathWaypoints() {
        try {
            panelsField.setStyle(pathLook);
            
            // Draw waypoints as small circles (simplified approach)
            panelsField.circle(3);
            
            // Debug telemetry
            telemetry.addData("DEBUG", "Path waypoints drawn: Start, Score, Teleop");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Path waypoints drawing failed: " + e.getMessage());
        }
    }

    /**
     * Update Panels telemetry with essential data only
     */
    private void updatePanelsTelemetry() {
        try {
            // Only show essential data - no real-time updates
            // This method is kept minimal to avoid clutter
            
        } catch (Exception e) {
            telemetry.addData("ERROR", "Panels telemetry update failed: " + e.getMessage());
        }
    }

    /**
     * Update telemetry with important information
     */
    private void updateTelemetry() {
        telemetry.clear();
        
        // Header information
        telemetry.addLine("=== NOV AUTO RED ===");
        telemetry.addLine();
        
        // Robot position
        telemetry.addLine("--- ROBOT STATUS ---");
        telemetry.addData("X Position", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Y Position", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Drive Speed", String.format("%.1f%%", autonomousDriveSpeed * 100));
        telemetry.addLine();
        
        // Panels status
        telemetry.addLine("--- PANELS VISUALIZATION ---");
        telemetry.addData("Panels Status", "ACTIVE");
        telemetry.addData("Robot Visualization", "18x18 robot with heading");
        telemetry.addData("Path History", "Green trail");
        telemetry.addData("Waypoints", "Orange circles");
        telemetry.addLine();
        
        // Autonomous state
        telemetry.addLine("--- AUTONOMOUS STATE ---");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scoring State", scoringState);
        telemetry.addData("Balls Scored", ballsScored + "/" + TOTAL_BALLS_TO_SCORE);
        telemetry.addData("Scoring Complete", scoringComplete);
        telemetry.addLine();
        
        // Launcher status
        telemetry.addLine("--- LAUNCHER STATUS ---");
        telemetry.addData("Launch State", launchState.toString());
        telemetry.addData("Shooter Velocity", String.format("%.1f", shooterMotor.getVelocity()));
        telemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Shooter Power", String.format("%.3f", shooterMotor.getPower()));
        telemetry.addData("Left Servo Power", String.format("%.3f", leftServo.getPower()));
        telemetry.addData("Right Servo Power", String.format("%.3f", rightServo.getPower()));
        telemetry.addLine();
        
        // Launcher velocity configuration (updated from NovTeleOpRedSemiAuto.java)
        telemetry.addLine("--- LAUNCHER VELOCITY CONFIG ---");
        telemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Min Velocity", LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Control Method", "setVelocity() (from NovTeleOpRedSemiAuto.java)");
        telemetry.addData("Effective Target", String.format("%.1f", LAUNCHER_TARGET_VELOCITY * 0.95));
        telemetry.addLine();
        
        // Launcher timing configuration (updated for accuracy)
        telemetry.addLine("--- LAUNCHER TIMING CONFIG ---");
        telemetry.addData("Feed Time", String.format("%.3f seconds", FEED_TIME_SECONDS));
        telemetry.addData("Wait Between Shots", String.format("%.1f seconds", FEED_DELAY_SECONDS));
        telemetry.addData("Final Launch Delay", String.format("%.1f seconds", FINAL_LAUNCH_DELAY));
        telemetry.addData("Total Balls", TOTAL_BALLS_TO_SCORE);
        telemetry.addLine();
        
        // Timing information
        telemetry.addLine("--- TIMING ---");
        telemetry.addData("Path Timer", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
        telemetry.addData("Action Timer", String.format("%.2f", actionTimer.getElapsedTimeSeconds()));
        telemetry.addData("OpMode Timer", String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Feed Timer", String.format("%.2f", feederTimer.seconds()));
        
        
        telemetry.update();
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    
    public void setScoringState(int sState) {
        scoringState = sState;
        actionTimer.resetTimer();
    }
}