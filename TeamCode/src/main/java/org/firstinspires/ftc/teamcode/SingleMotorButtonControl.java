package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Simple single motor control with button press
 * Motor runs when button is pressed, stops when released
 */
@TeleOp(name = "Single Motor Button Control", group = "TeleOp")
public class SingleMotorButtonControl extends LinearOpMode {

    // Declare the motor
    private DcMotorEx intakeMotor;

    // Motor settings - adjust these as needed
    private static final double MOTOR_POWER = 0.75;    // Motor power (0.0 to 1.0)
    
    // Button to control motor - change gamepad1.a to any button you want
    // Options: gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y
    //          gamepad1.left_bumper, gamepad1.right_bumper
    //          gamepad1.dpad_up, gamepad1.dpad_down, etc.

    @Override
    public void runOpMode() {
        // Initialize the motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake motor");

        // Set motor direction (FORWARD or REVERSE)
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set motor behavior when power is 0 (BRAKE or FLOAT)
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set motor mode
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Stop motor initially
        intakeMotor.setPower(0);

        // Display status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor Name", intakeMotor);
        telemetry.addData("Motor Power", MOTOR_POWER);
        telemetry.addData("Control", "Press X button to run motor");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Main loop - runs until driver presses STOP
        while (opModeIsActive()) {
            
            // Check if X button is pressed
            if (gamepad1.x) {
                // X button is pressed - run the motor
                intakeMotor.setPower(MOTOR_POWER);
            } else {
                // X button is released - stop the motor
                intakeMotor.setPower(0);
            }

            // Display motor status
            telemetry.addData("Status", "Running");
            telemetry.addData("Button Pressed", gamepad1.x);
            telemetry.addData("Motor Power", "%.2f", intakeMotor.getPower());
            telemetry.update();
        }

        // Stop motor when op mode ends
        intakeMotor.setPower(0);
    }
}

