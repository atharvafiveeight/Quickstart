/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
//@Disabled
public class StarterBotTeleopMecanums extends OpMode {
    final double FEED_TIME_SECONDS = 0.05; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1250;
    final double LAUNCHER_MIN_VELOCITY = 1200;

    // Declare OpMode members.
  
    private DcMotorEx shooterMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

   
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
  
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
      
        shooterMotor.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftServo.setPower(STOP_SPEED);
        rightServo.setPower(STOP_SPEED);

        // PIDF coefficients for velocity control - GoBilda 6000rpm motor
        // P=150, I=0, D=0, F=10 (reduced from 300 to prevent overshooting)
        // If motor overshoots target velocity, try reducing P value further (e.g., 100 or 75)
        // If motor is slow to reach target, try increasing P value
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(3, 0, 100, 10));
        
        // Debug: Verify motor is in correct mode for velocity control
        telemetry.addData("DEBUG", "Motor mode set to: " + shooterMotor.getMode());
        telemetry.addData("DEBUG", "PIDF coefficients set for GoBilda 6000rpm motor");

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("DEBUG", "Y pressed: setVelocity(" + LAUNCHER_TARGET_VELOCITY + ") called");
        } else if (gamepad1.b) { // stop flywheel
            shooterMotor.setVelocity(STOP_SPEED);
            telemetry.addData("DEBUG", "B pressed: setVelocity(" + STOP_SPEED + ") called, resetting state to IDLE");
            // Reset launch state to IDLE when manually stopping
            launchState = LaunchState.IDLE;
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Current Velocity", shooterMotor.getVelocity());
        telemetry.addData("Abs Velocity", Math.abs(shooterMotor.getVelocity()));
        telemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Min Velocity", LAUNCHER_MIN_VELOCITY);
        
        // Only show velocity error when motor is actually running
        double absVelocity = Math.abs(shooterMotor.getVelocity());
        if (launchState != LaunchState.IDLE || absVelocity > 0) {
            telemetry.addData("Velocity Error", LAUNCHER_TARGET_VELOCITY - absVelocity);
        } else {
            telemetry.addData("Velocity Error", "Motor stopped");
        }
        
        telemetry.addData("Motor Power", shooterMotor.getPower());
        telemetry.addData("Motor Mode", shooterMotor.getMode());
        
        // Debug: Show if velocity control is working (fixed logic using absolute value)
        double currentVel = Math.abs(shooterMotor.getVelocity());
        if (currentVel > LAUNCHER_TARGET_VELOCITY + 50) {
            telemetry.addData("Status", "WARNING: Motor overshooting target velocity!");
        } else if (currentVel >= LAUNCHER_TARGET_VELOCITY - 25 && currentVel <= LAUNCHER_TARGET_VELOCITY + 25) {
            telemetry.addData("Status", "Motor at target velocity");
        } else if (currentVel < LAUNCHER_TARGET_VELOCITY - 50) {
            telemetry.addData("Status", "Motor below target velocity");
        } else {
            telemetry.addData("Status", "Motor approaching target velocity");
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    telemetry.addData("DEBUG", "Launch requested - entering SPIN_UP");
                }
                break;
            case SPIN_UP:
                shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                double currentVel = Math.abs(shooterMotor.getVelocity()); // Use absolute value
                telemetry.addData("DEBUG", "SPIN_UP: setVelocity(" + LAUNCHER_TARGET_VELOCITY + ") called");
                telemetry.addData("DEBUG", "SPIN_UP: Current velocity = " + shooterMotor.getVelocity() + " (abs: " + currentVel + ")");
                telemetry.addData("DEBUG", "SPIN_UP: Need " + LAUNCHER_MIN_VELOCITY + " to launch");
                // Check if motor has reached minimum velocity for launching (using absolute value)
                if (currentVel >= LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    telemetry.addData("DEBUG", "Velocity reached - entering LAUNCH");
                } else {
                    telemetry.addData("DEBUG", "Still spinning up... (" + currentVel + "/" + LAUNCHER_MIN_VELOCITY + ")");
                }
                break;
            case LAUNCH:
                leftServo.setPower(FULL_SPEED);
                rightServo.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                telemetry.addData("DEBUG", "LAUNCH: Servos activated, entering LAUNCHING");
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftServo.setPower(STOP_SPEED);
                    rightServo.setPower(STOP_SPEED);
                    telemetry.addData("DEBUG", "LAUNCHING: Feed complete, returning to IDLE");
                }
                break;
        }
    }
}