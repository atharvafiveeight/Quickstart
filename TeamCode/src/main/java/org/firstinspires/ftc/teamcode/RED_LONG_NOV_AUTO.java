package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "RED-LONG-NOV-AUTO", group = "Autonomous")
public class RED_LONG_NOV_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Robot hardware - motors and servos for shooting
    private DcMotorEx shooterMotor;  // Motor that spins the shooter wheel
    private CRServo leftServo;       // Left servo that feeds balls into shooter
    private CRServo rightServo;      // Right servo that feeds balls into shooter
    
    // State tracking variables - keep track of what the robot is doing
    private int pathState;           // Which movement step we're on (0, 1, 2, 3)
    private int scoringState;        // Which shooting step we're on (0, 1, 2)
    private int ballsScored;        // How many balls we've shot so far
    private boolean scoringComplete; // Whether we're done shooting
    
    // Robot positions on the field (in inches) - MODIFIED FOR LONG DISTANCE
    private final Pose startPose = new Pose(86.793, 9.012, Math.toRadians(90)); // Where robot starts (NEW POSITION)
    private final Pose scorePose = new Pose(80.219, 19.288, Math.toRadians(65)); // Long range scoring position
    private final Pose startTeleopPose = new Pose(81.096, 38.795, Math.toRadians(0)); // Where robot ends (SAME AS SHORT)

    // Paths the robot will follow
    private PathChain startToScore;    // Path from start to shooting position
    private PathChain scoreToTeleop;   // Path from shooting to final position
    
    // Shooting timing and speed settings
    private final double FEED_TIME_SECONDS = 0.05;        // How long servos run to feed each ball
    private final double FEED_DELAY_SECONDS = 2;          // Wait time between shots
    private final double FINAL_LAUNCH_DELAY = 2.00;       // Extra wait after last ball
    private final double FULL_SPEED = 1.0;                // Maximum servo speed
    private final double STOP_SPEED = 0.0;                // Servo stopped
    
    // Shooter motor speed settings - MODIFIED FOR LONG DISTANCE
    private final double LAUNCHER_TARGET_VELOCITY = 1750; // Target speed for long distance shooting
    private final double LAUNCHER_MIN_VELOCITY = 1625;    // Minimum speed before shooting
    private final int TOTAL_BALLS_TO_SCORE = 3;           // How many balls to shoot
    
    // Robot movement speed for autonomous (0.0 to 1.0)
    private double autonomousDriveSpeed = 0.3;            // 30% speed for precision
    
    // Different states the shooter can be in
    private enum LaunchState {
        IDLE,        // Shooter stopped, waiting to start
        SPIN_UP,     // Shooter speeding up to target speed
        LAUNCH,      // Ready to shoot, start feeding ball
        LAUNCHING,   // Currently feeding ball into shooter
        WAITING,     // Waiting between shots
        FINAL_WAIT   // Waiting for last ball to launch
    }
    
    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();

    /** This method is called once when the robot starts up. **/
    @Override
    public void init() {
        // Set up timers to track how long things take
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize the robot's movement system
        follower = Constants.createFollower(hardwareMap);
        
        // Set robot to move slower for precision in autonomous
        applyAutonomousSpeedReduction();
        
        // Set up the shooter motor and servos
        initializeLauncherHardware();

        // Create the paths the robot will follow
        buildAutonomousPaths();

        // Tell the robot where it starts on the field
        follower.setStartingPose(startPose);

        // Set initial state - robot starts at step 0, not shooting
        pathState = 0;
        scoringState = 0;
        ballsScored = 0;
        scoringComplete = false;
        launchState = LaunchState.IDLE;
    }

    /** This method runs while waiting for the driver to press "Play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to start - LONG DISTANCE");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Scoring State", scoringState);
        telemetry.addData("Balls to Score", TOTAL_BALLS_TO_SCORE);
        telemetry.addData("Shooting Mode", "LONG DISTANCE");
        telemetry.update();
    }

    /** This method is called once when the driver presses "Play". **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setScoringState(0);
    }

    /** This is the main loop that runs continuously during autonomous. **/
    @Override
    public void loop() {
        // Update robot movement and position tracking
        follower.update();
        
        // Handle robot movement between positions
        autonomousPathUpdate();
        
        // Handle shooting balls
        autonomousScoringUpdate();

        // Show information on the driver station
        updateTelemetry();
    }

    /** This method is called when autonomous ends. **/
    @Override
    public void stop() {
        // Stop the shooter when autonomous is done
        stopLauncher();
    }

    /**
     * Set up the shooter motor and servos
     */
    private void initializeLauncherHardware() {
        try {
            // Set up the shooter motor
            // Using hardcoded hardware name instead of Constants (Constants.SHOOTER_MOTOR_NAME was removed)
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            
            // Set motor control settings for smooth speed control
            shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(3, 0, 0, 10));
            
            // Set up the ball feeding servos
            // Using hardcoded hardware names instead of Constants (Constants.LEFT_SERVO_NAME and Constants.RIGHT_SERVO_NAME were removed)
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");
            leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
            
            // Start with servos stopped
            leftServo.setPower(0);
            rightServo.setPower(0);
            
            telemetry.addData("Status", "Shooter hardware ready - LONG DISTANCE");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to set up shooter: " + e.getMessage());
        }
    }
    
    /**
     * Set robot to move slower for precision in autonomous
     */
    private void applyAutonomousSpeedReduction() {
        try {
            // Robot moves at 30% speed for precision
            telemetry.addData("Status", "Autonomous speed: " + String.format("%.1f%%", autonomousDriveSpeed * 100));
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to set speed: " + e.getMessage());
        }
    }
    
    
    /**
     * Create the paths the robot will follow
     */
    private void buildAutonomousPaths() {
        try {
            // Path 1: Move from start position to long range shooting position
            startToScore = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
            telemetry.addData("Status", "Path 1 created: Start to Long Range Shooting");
                    
            // Path 2: Move from shooting position to final position (same as short)
            scoreToTeleop = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, startTeleopPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), startTeleopPose.getHeading())
                    .build();
            telemetry.addData("Status", "Path 2 created: Long Range Shooting to Final");
            telemetry.addData("Status", "All paths ready!");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to create paths: " + e.getMessage());
        }
    }

    /**
     * Handle robot movement between positions
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move from start to long range shooting position
                if (startToScore != null) {
                    follower.followPath(startToScore);
                    setPathState(1);
                } else {
                    telemetry.addData("ERROR", "Path not created!");
                }
                break;

            case 1:
                // Wait for movement to finish, then start shooting
                if (!follower.isBusy()) {
                    setPathState(2);
                    setScoringState(1); // Start shooting
                }
                break;
                
            case 2:
                // Wait for shooting to finish, then move to final position
                if (scoringComplete) {
                    follower.followPath(scoreToTeleop);
                    setPathState(3);
                }
                break;
                
            case 3:
                // Wait for final movement to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Done with autonomous
                }
                break;
        }
    }

    /**
     * Handle shooting balls
     */
    public void autonomousScoringUpdate() {
        switch (scoringState) {
            case 0:
                // Not shooting
                break;
                
            case 1:
                // Start shooting sequence
                ballsScored = 0;
                launchState = LaunchState.SPIN_UP;
                feederTimer.reset();
                setScoringState(2);
                break;
                
            case 2:
                // Shooting in progress
                runLauncherStateMachine();
                
                // Check if all balls have been shot
                if (ballsScored >= TOTAL_BALLS_TO_SCORE && launchState == LaunchState.IDLE) {
                    stopLauncher();
                    setScoringState(0);
                    scoringComplete = true;
                }
                break;
        }
    }


    /**
     * Control the shooter motor and ball feeding - MODIFIED FOR LONG DISTANCE
     */
    private void runLauncherStateMachine() {
        switch (launchState) {
            case IDLE:
                // Stop everything
                try {
                    shooterMotor.setVelocity(0);
                    leftServo.setPower(STOP_SPEED);
                    rightServo.setPower(STOP_SPEED);
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to stop shooter: " + e.getMessage());
                }
                break;

            case SPIN_UP:
                // Speed up shooter motor to LONG DISTANCE velocity
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    double currentVelocity = Math.abs(shooterMotor.getVelocity());
                    
                    // Wait until motor reaches 95% of target speed
                    double effectiveTargetVelocity = LAUNCHER_TARGET_VELOCITY * 0.95;
                    
                    if (currentVelocity >= effectiveTargetVelocity) {
                        launchState = LaunchState.LAUNCH;
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to speed up shooter: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                // Start feeding ball
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    
                    // Turn on servos to feed ball
                    leftServo.setPower(FULL_SPEED);
                    rightServo.setPower(FULL_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed to start feeding: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCHING:
                // Keep feeding ball for specified time
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    double launchingVelocity = Math.abs(shooterMotor.getVelocity());
                    
                    // Stop feeding after specified time
                    if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                        leftServo.setPower(STOP_SPEED);
                        rightServo.setPower(STOP_SPEED);
                        ballsScored++;
                        
                        // Record ball launch data
                        telemetry.addData("Ball " + ballsScored + " Shot (LONG)", "Velocity: " + String.format("%.1f", launchingVelocity));
                        
                        // Wait before next shot
                        feederTimer.reset();
                        launchState = LaunchState.WAITING;
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed during ball feeding: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;
                
            case WAITING:
                // Wait between shots
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    
                    // Wait specified time between shots
                    if (feederTimer.seconds() > FEED_DELAY_SECONDS) {
                        if (ballsScored >= TOTAL_BALLS_TO_SCORE) {
                            // Last ball shot, wait extra time
                            feederTimer.reset();
                            launchState = LaunchState.FINAL_WAIT;
                        } else {
                            // Ready for next shot
                            launchState = LaunchState.LAUNCH;
                        }
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed while waiting: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;
                
            case FINAL_WAIT:
                // Wait extra time for last ball to launch
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    
                    // Wait extra time for final ball
                    if (feederTimer.seconds() > FINAL_LAUNCH_DELAY) {
                        launchState = LaunchState.IDLE;
                    }
                } catch (Exception e) {
                    telemetry.addData("ERROR", "Failed during final wait: " + e.getMessage());
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    /**
     * Stop the shooter completely
     */
    private void stopLauncher() {
        try {
            shooterMotor.setVelocity(0);
            leftServo.setPower(STOP_SPEED);
            rightServo.setPower(STOP_SPEED);
            launchState = LaunchState.IDLE;
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to stop shooter: " + e.getMessage());
        }
    }


    /**
     * Show robot status on driver station
     */
    private void updateTelemetry() {
        telemetry.clear();
        
        // Header
        telemetry.addLine("=== AUTONOMOUS RED LONG ===");
        telemetry.addLine();
        
        // Robot position
        telemetry.addLine("--- ROBOT POSITION ---");
        telemetry.addData("X Position", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Y Position", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Moving", follower.isBusy());
        telemetry.addLine();
        
        // Autonomous progress
        telemetry.addLine("--- AUTONOMOUS PROGRESS ---");
        telemetry.addData("Path Step", pathState);
        telemetry.addData("Shooting Step", scoringState);
        telemetry.addData("Balls Shot", ballsScored + "/" + TOTAL_BALLS_TO_SCORE);
        telemetry.addData("Shooting Done", scoringComplete);
        telemetry.addData("Shooting Mode", "LONG DISTANCE");
        telemetry.addLine();
        
        // Shooter status
        telemetry.addLine("--- SHOOTER STATUS ---");
        telemetry.addData("Shooter State", launchState.toString());
        telemetry.addData("Motor Speed", String.format("%.1f", shooterMotor.getVelocity()));
        telemetry.addData("Target Speed", LAUNCHER_TARGET_VELOCITY + " (LONG)");
        telemetry.addData("Left Servo", String.format("%.3f", leftServo.getPower()));
        telemetry.addData("Right Servo", String.format("%.3f", rightServo.getPower()));
        telemetry.addLine();
        
        // Timing
        telemetry.addLine("--- TIMING ---");
        telemetry.addData("Total Time", String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Feed Timer", String.format("%.2f", feederTimer.seconds()));
        
        telemetry.update();
    }

    /** Change the movement step and reset timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    
    /** Change the shooting step and reset timer **/
    public void setScoringState(int sState) {
        scoringState = sState;
        actionTimer.resetTimer();
    }
}
