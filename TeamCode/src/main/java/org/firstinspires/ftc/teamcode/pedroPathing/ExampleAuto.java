package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Define poses for the autonomous path (from your generated path)
    private final Pose startPose = new Pose(33.055, 24.791, Math.toRadians(90)); // Start Pose from visualizer
    private final Pose endPose = new Pose(64.000, 55.912, Math.toRadians(90)); // End Pose from visualizer

    // Define the path chain from your generated path
    private PathChain line1;

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize the Follower using Constants (recommended approach)
        follower = Constants.createFollower(hardwareMap);

        // Build the generated path from visualizer
        buildGeneratedPath();

        // Set the starting pose
        follower.setStartingPose(startPose);

        // Set the initial path state
        pathState = 0;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    /**
     * Build the generated path from Pedro Pathing Visualizer
     */
    private void buildGeneratedPath() {
        // Using follower.pathBuilder() as shown in the official docs
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    /**
     * Autonomous path update method - manages the state machine
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the generated path
                follower.followPath(line1);
                setPathState(1);
                break;

            case 1:
                // Wait for the path to complete
                if (!follower.isBusy()) {
                    // Path is complete - autonomous finished
                    setPathState(-1); // Set to unused state to stop
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}