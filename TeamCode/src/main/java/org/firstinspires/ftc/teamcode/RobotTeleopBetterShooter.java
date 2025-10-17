package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot TeleOp Better Shooter", group = "TeleOp")
public class RobotTeleopBetterShooter extends LinearOpMode {

    // Hardware declarations
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx shooterMotor;  // Changed to DcMotorEx for velocity control
    private CRServo leftServo;
    private CRServo rightServo;

    // PID and timing constants
    private final double FEED_TIME_SECONDS = 0.20;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double LAUNCHER_TARGET_VELOCITY = 1125;
    private final double LAUNCHER_MIN_VELOCITY = 1075;

    // State machine for launcher
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;
    private ElapsedTime feederTimer = new ElapsedTime();

    // Variables for control
    private boolean prevRightBumper = false;

    @Override
    public void runOpMode() {
        // Initialize state
        launchState = LaunchState.IDLE;

        // Initialize motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // Initialize servos (using CRServo for continuous rotation)
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        // Set motor directions (adjust as needed for your robot)
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up shooter motor with encoder-based velocity control
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set custom PIDF coefficients for better launcher control
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stop servos initially
        leftServo.setPower(0);
        rightServo.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        stopAllMotors();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // ========== DRIVETRAIN CONTROL ==========
            // Forward/backward control using left stick Y
            double driveY = -gamepad1.left_stick_y;

            // Strafe control using left stick X
            double driveX = -gamepad1.left_stick_x;

            // Apply drive powers
            frontRight.setPower(driveY + driveX);
            backRight.setPower(driveY + driveX);
            frontLeft.setPower(driveY - driveX);
            backLeft.setPower(driveY - driveX);

            // ========== LAUNCHER CONTROL WITH PID ==========
            // Detect right bumper press (edge detection)
            boolean rightBumperPressed = gamepad1.x && !prevRightBumper;
            prevRightBumper = gamepad1.x;

            // Run the launch state machine
            launch(rightBumperPressed);

            // Telemetry for debugging
            telemetry.addData("Launch State", launchState);
            telemetry.addData("Drive Y Power", "%.2f", driveY);
            telemetry.addData("Drive X Power", "%.2f", driveX);
            telemetry.addData("Shooter Velocity", "%.2f", shooterMotor.getVelocity());
            telemetry.addData("Target Velocity", LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Front Right Motor", "%.2f", frontRight.getPower());
            telemetry.addData("Back Right Motor", "%.2f", backRight.getPower());
            telemetry.update();
        }

        stopAllMotors();
    }


    void launch(boolean shotRequested) {
        telemetry.addData("shotRequested",1);

        switch (launchState) {
            case IDLE:
                // When idle, stop the shooter
                shooterMotor.setVelocity(STOP_SPEED);
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    telemetry.addData("changing to spinUp",1);

                }
                break;

            case SPIN_UP:
                // Spin up the shooter to target velocity
                shooterMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (shooterMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Start feeding the game pieces
                leftServo.setPower(FULL_SPEED);
                rightServo.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                // Continue feeding for the specified time
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftServo.setPower(STOP_SPEED);
                    rightServo.setPower(STOP_SPEED);
                }
                break;
        }
    }

    void stopAllMotors() {
        // Stop all motors and reset servos when OpMode ends
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        shooterMotor.setVelocity(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}