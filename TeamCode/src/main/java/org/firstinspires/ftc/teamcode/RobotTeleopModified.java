package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Robot TeleOp Modified", group = "TeleOp")
public class RobotTeleopModified extends LinearOpMode {

    // Hardware declarations
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor shooterMotor;
    private CRServo leftServo;
    private CRServo rightServo;

    // Variables for control
    private boolean servoActive = false;
    private boolean prevRightBumper = false;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

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


        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            // Motor control using left stick (y-axis for forward/backward)
            double drive = -gamepad1.left_stick_y;

            // Apply power to motors
            frontRight.setPower(drive);
            backRight.setPower(drive);
            frontLeft.setPower(drive);
            backLeft.setPower(drive);

            drive = -gamepad1.left_stick_x;

            frontRight.setPower(drive);
            backRight.setPower(drive);
            frontLeft.setPower(-drive);
            backLeft.setPower(-drive);
            if (gamepad1.right_bumper) {
                shooterMotor.setPower(0.75);
                sleep(500);
                leftServo.setPower(1);
                rightServo.setPower(1);


            } else {
                leftServo.setPower(0);
                rightServo.setPower(0);
                shooterMotor.setPower(0);
            }
            // Telemetry for debugging
            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Front Right Motor", "%.2f", frontRight.getPower());
            telemetry.addData("Back Right Motor", "%.2f", backRight.getPower());
            telemetry.addData("Servo Status", servoActive ? "POSITION 1.0" : "POSITION 0.0");
            telemetry.update();
        }

        stopAllMotors();
    }

    void stopAllMotors() {
        // Stop all motors and reset servos when OpMode ends

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }
}