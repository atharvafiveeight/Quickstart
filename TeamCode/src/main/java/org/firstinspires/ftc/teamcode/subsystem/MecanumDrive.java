package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * MecanumDrive - Drivetrain subsystem using PedroPathing Follower
 * 
 * This subsystem wraps the PedroPathing Follower to provide teleop drive control.
 * The Follower handles all motor control, odometry, and path following automatically.
 * 
 * Key Features:
 * - Uses PedroPathing Follower for motor control
 * - Supports robot-centric and field-centric drive modes
 * - Provides pose tracking and velocity information
 * - Handles odometry updates automatically
 * 
 * @author Team
 * @version 1.0 - Initial implementation based on DuneStrider
 */
public class MecanumDrive {
    
    // PedroPathing Follower that handles all drivetrain control
    public Follower follower;
    
    // Last known pose (updated in periodic)
    public static Pose lastPose = new Pose(0, 0, 0);
    
    /**
     * Constructor - Initializes the Follower with starting pose
     * 
     * @param map HardwareMap for initializing motors and sensors
     * @param startingPose Initial robot pose (x, y, heading). If null, uses (0, 0, 0)
     */
    public MecanumDrive(HardwareMap map, Pose startingPose) {
        // Create follower using Constants configuration
        this.follower = Constants.createFollower(map);
        
        // Set starting pose (use (0, 0, 0) if not provided)
        follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
        
        // Initial update to sync pose
        follower.update();
    }
    
    /**
     * Periodic update - should be called every loop iteration
     * Updates pose tracking and follower state
     */
    public void periodic() {
        // Update last known pose
        lastPose = follower.getPose();
        
        // Update follower (handles odometry and motor control)
        follower.update();
    }
    
    /**
     * Set teleop drive commands
     * This is the main method for controlling the robot during teleop
     * 
     * @param forward Forward/backward input (-1.0 to 1.0, positive = forward)
     * @param strafe Left/right strafe input (-1.0 to 1.0, positive = right)
     * @param rotation Rotation input (-1.0 to 1.0, positive = counterclockwise)
     */
    public void setTeleOpDrive(double forward, double strafe, double rotation) {
        // Call follower's teleop drive method
        // Last parameter (false) = robot-centric mode (not field-centric)
        // Change to true for field-centric drive if desired
        follower.setTeleOpDrive(forward, strafe, rotation, false);
    }
    
    /**
     * Set teleop drive with field-centric option
     * 
     * @param forward Forward/backward input (-1.0 to 1.0, positive = forward)
     * @param strafe Left/right strafe input (-1.0 to 1.0, positive = right)
     * @param rotation Rotation input (-1.0 to 1.0, positive = counterclockwise)
     * @param fieldCentric If true, uses field-centric drive (relative to field). If false, uses robot-centric (relative to robot)
     */
    public void setTeleOpDrive(double forward, double strafe, double rotation, boolean fieldCentric) {
        follower.setTeleOpDrive(forward, strafe, rotation, fieldCentric);
    }
    
    /**
     * Start teleop drive mode
     * Should be called once at the start of teleop
     */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }
    
    /**
     * Get the current estimated robot pose
     * 
     * @return Current pose (x, y, heading in radians)
     */
    public Pose getPose() {
        return follower.getPose();
    }
    
    /**
     * Reset robot heading to a new value
     * Useful for relocalization or IMU resets
     * 
     * @param newHeading New heading in radians
     */
    public void resetHeading(double newHeading) {
        Pose currentPose = follower.getPose();
        follower.setPose(currentPose.setHeading(newHeading));
    }
    
    /**
     * Get current robot velocity
     * 
     * @return Velocity vector (x, y components in inches per second)
     */
    public Vector getVelocity() {
        return follower.getVelocity();
    }
    
    /**
     * Get current angular velocity
     * 
     * @return Angular velocity in radians per second
     */
    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }
    
    /**
     * Get current acceleration
     * 
     * @return Acceleration vector (x, y components)
     */
    public Vector getAcceleration() {
        return follower.getAcceleration();
    }
}
