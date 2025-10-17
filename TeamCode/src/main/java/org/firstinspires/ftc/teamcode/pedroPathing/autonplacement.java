package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auton Placement", group = "Examples")
public class autonplacement extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Define poses for autonomous routine
    private final Pose startPose = new Pose(28.751, 7.442, Math.toRadians(90));
    private final Pose curveMidPose = new Pose(0.372, 118.698, Math.toRadians(90));
    private final Pose curveEndPose = new Pose(118.140, 119.442, Math.toRadians(270));
    private final Pose finalPose = new Pose(123.721, 123.349, Math.toRadians(220));

    // Path variables
    private PathChain paths;

    /**
     * Build all paths for the autonomous routine
     */
    public void buildPaths() {
        /* This is our generated path using a BezierCurve and BezierLine */
        paths = follower.pathBuilder()
                // Path 1: BezierCurve
                .addPath(new BezierCurve(
                        new Pose(28.751, 7.442),
                        new Pose(0.372, 118.698),
                        new Pose(118.140, 119.442)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))
                // Path 2: BezierLine
                .addPath(new BezierLine(
                        new Pose(118.140, 119.442),
                        new Pose(123.721, 123.349)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(220))
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
                    /* Path completed - hold final position as failsafe */
                    follower.holdPoint(new BezierPoint(finalPose.getX(), finalPose.getY()), Math.toRadians(220));
                    setPathState(2);
                }
                break;
            case 2:
                /* Continuously hold position to correct any drift */
                /* This state will keep the robot locked at the final position */
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