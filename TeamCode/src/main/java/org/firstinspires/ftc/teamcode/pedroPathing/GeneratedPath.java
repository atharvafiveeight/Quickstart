package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Generated Path Auto", group = "Examples")
public class GeneratedPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Define poses for autonomous routine
    private final Pose startPose = new Pose(55.542, 31.442, Math.toRadians(90));
    private final Pose endPose = new Pose(80.472, 56.930, Math.toRadians(90));

    // Path variables
    private PathChain paths;

    /**
     * Build all paths for the autonomous routine
     */
    public void buildPaths() {
        /* This is our generated path using a BezierLine */
        paths = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.542, 31.442), new Pose(80.472, 56.930)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    /**
     * Autonomous path update method - manages state transitions
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running any new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This method is called once at the init of the OpMode
     */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /**
     * This method is called continuously after Init while waiting for "play"
     */
    @Override
    public void init_loop() {}

    /**
     * This method is called once at the start of the OpMode
     * It runs all the setup actions, including building paths and starting the path system
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play"
     */
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * We do not use this because everything should automatically disable
     */
    @Override
    public void stop() {}
}