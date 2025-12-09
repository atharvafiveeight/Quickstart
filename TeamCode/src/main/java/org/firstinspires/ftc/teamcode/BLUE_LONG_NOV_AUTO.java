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

@Autonomous(name = "BLUE-LONG-NOV-AUTO", group = "Autonomous")
public class BLUE_LONG_NOV_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private DcMotorEx shooterMotor;
    private CRServo leftServo;
    private CRServo rightServo;

    private int pathState;
    private int scoringState;
    private int ballsScored;
    private boolean scoringComplete;

    // Blue long: mirror RED long start, use Blue long scoring pose, end at BLUE teleop start
    private final Pose startPose = new Pose(57.207, 9.012, Math.toRadians(90));
    private final Pose scorePose = new Pose(63.781, 19.288, Math.toRadians(115));
    private final Pose startTeleopPose = new Pose(62.904, 38.795, Math.toRadians(180));

    private PathChain startToScore;
    private PathChain scoreToTeleop;

    private final double FEED_TIME_SECONDS = 0.05;
    private final double FEED_DELAY_SECONDS = 2;
    private final double FINAL_LAUNCH_DELAY = 2.00;
    private final double FULL_SPEED = 1.0;
    private final double STOP_SPEED = 0.0;

    private final double LAUNCHER_TARGET_VELOCITY = 1750;
    private final double LAUNCHER_MIN_VELOCITY = 1625;
    private final int TOTAL_BALLS_TO_SCORE = 3;

    private double autonomousDriveSpeed = 0.3;

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING, WAITING, FINAL_WAIT }
    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void init() {
        pathTimer = new Timer(); actionTimer = new Timer(); opmodeTimer = new Timer(); opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        applyAutonomousSpeedReduction();
        initializeLauncherHardware();
        buildAutonomousPaths();
        follower.setStartingPose(startPose);
        pathState = 0; scoringState = 0; ballsScored = 0; scoringComplete = false; launchState = LaunchState.IDLE;
    }

    @Override public void init_loop() { telemetry.addData("Status","Ready to start - BLUE LONG"); telemetry.update(); }
    @Override public void start() { opmodeTimer.resetTimer(); setPathState(0); setScoringState(0); }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        autonomousScoringUpdate();
        updateTelemetry();
    }

    @Override public void stop() { stopLauncher(); }

    private void initializeLauncherHardware() {
        try {
            // Using hardcoded hardware name instead of Constants (Constants.SHOOTER_MOTOR_NAME was removed)
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(3,0,0,10));
            // Using hardcoded hardware names instead of Constants (Constants.LEFT_SERVO_NAME and Constants.RIGHT_SERVO_NAME were removed)
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");
            leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
            leftServo.setPower(0); rightServo.setPower(0);
        } catch (Exception e) { telemetry.addData("ERROR","Shooter init failed: "+e.getMessage()); }
    }

    private void applyAutonomousSpeedReduction() { telemetry.addData("Status","Autonomous speed: "+String.format("%.1f%%", autonomousDriveSpeed*100)); }

    private void buildAutonomousPaths() {
        try {
            startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
            scoreToTeleop = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, startTeleopPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), startTeleopPose.getHeading())
                .build();
        } catch (Exception e) { telemetry.addData("ERROR", "Path build failed: "+e.getMessage()); }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (startToScore != null) { follower.followPath(startToScore); setPathState(1); }
                break;
            case 1:
                if (!follower.isBusy()) { setPathState(2); setScoringState(1); }
                break;
            case 2:
                if (scoringComplete) { follower.followPath(scoreToTeleop); setPathState(3); }
                break;
            case 3:
                if (!follower.isBusy()) { setPathState(-1); }
                break;
        }
    }

    public void autonomousScoringUpdate() {
        switch (scoringState) {
            case 0: break;
            case 1:
                ballsScored = 0; launchState = LaunchState.SPIN_UP; feederTimer.reset(); setScoringState(2); break;
            case 2:
                runLauncherStateMachine();
                if (ballsScored >= TOTAL_BALLS_TO_SCORE && launchState == LaunchState.IDLE) {
                    stopLauncher(); setScoringState(0); scoringComplete = true;
                }
                break;
        }
    }

    private void runLauncherStateMachine() {
        switch (launchState) {
            case IDLE:
                try { shooterMotor.setVelocity(0); leftServo.setPower(STOP_SPEED); rightServo.setPower(STOP_SPEED);} catch (Exception ignored) {}
                break;
            case SPIN_UP:
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    double currentVelocity = Math.abs(shooterMotor.getVelocity());
                    double effectiveTargetVelocity = LAUNCHER_TARGET_VELOCITY * 0.95;
                    if (currentVelocity >= effectiveTargetVelocity) launchState = LaunchState.LAUNCH;
                } catch (Exception e) { launchState = LaunchState.IDLE; }
                break;
            case LAUNCH:
                try { shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY); leftServo.setPower(FULL_SPEED); rightServo.setPower(FULL_SPEED); feederTimer.reset(); launchState = LaunchState.LAUNCHING; } catch (Exception e) { launchState = LaunchState.IDLE; }
                break;
            case LAUNCHING:
                try {
                    shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (feederTimer.seconds() > FEED_TIME_SECONDS) { leftServo.setPower(STOP_SPEED); rightServo.setPower(STOP_SPEED); ballsScored++; feederTimer.reset(); launchState = LaunchState.WAITING; }
                } catch (Exception e) { launchState = LaunchState.IDLE; }
                break;
            case WAITING:
                try { shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY); if (feederTimer.seconds() > FEED_DELAY_SECONDS) { if (ballsScored >= TOTAL_BALLS_TO_SCORE) { feederTimer.reset(); launchState = LaunchState.FINAL_WAIT; } else { launchState = LaunchState.LAUNCH; } } } catch (Exception e) { launchState = LaunchState.IDLE; }
                break;
            case FINAL_WAIT:
                try { shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY); if (feederTimer.seconds() > FINAL_LAUNCH_DELAY) { launchState = LaunchState.IDLE; } } catch (Exception e) { launchState = LaunchState.IDLE; }
                break;
        }
    }

    private void stopLauncher() { try { shooterMotor.setVelocity(0); leftServo.setPower(STOP_SPEED); rightServo.setPower(STOP_SPEED); launchState = LaunchState.IDLE; } catch (Exception ignored) {} }

    private void updateTelemetry() {
        telemetry.clear(); telemetry.addLine("=== AUTONOMOUS BLUE LONG ==="); telemetry.addData("Path Step", pathState); telemetry.addData("Balls Shot", ballsScored+"/"+TOTAL_BALLS_TO_SCORE); telemetry.update();
    }

    public void setPathState(int pState) { pathState = pState; pathTimer.resetTimer(); }
    public void setScoringState(int sState) { scoringState = sState; actionTimer.resetTimer(); }
}



