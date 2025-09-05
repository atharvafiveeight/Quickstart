package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FourButtonMovementTest", group="Testing")
public class FourButtonMovementTest extends OpMode {

    // Motor declarations
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Motor power setting
    private double motorPower = 0.5; // Adjust this value as needed (0.0 to 1.0)
    private double reverseMotorPower = -0.5; // Adjust this value as needed (0.0 to 1.0)


    @Override
    public void init() {
        // Initialize motors from hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set all motor directions to forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Motor Test Initialized");
        telemetry.addData("Instructions", "Square=FL, Triangle=FR, Circle=BR, X=BL");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stop all motors first
        stopAllMotors();

        // Check button presses and run corresponding motor
        if (gamepad1.y) {
            frontLeft.setPower(motorPower);
            backLeft.setPower(motorPower);
            frontRight.setPower(motorPower);
            backRight.setPower(motorPower);
            telemetry.addData("Active Motor", "Front Left");
        }
        else if (gamepad1.x) {
            frontLeft.setPower(reverseMotorPower);
            backLeft.setPower(reverseMotorPower);
            frontRight.setPower(motorPower);
            backRight.setPower(motorPower);
            telemetry.addData("Active Motor", "Front Right");
        }
        else if (gamepad1.b) {
            frontLeft.setPower(motorPower);
            backLeft.setPower(motorPower);
            frontRight.setPower(reverseMotorPower);
            backRight.setPower(reverseMotorPower);
            telemetry.addData("Active Motor", "Back Right");
        }
        else if (gamepad1.a) {
            frontLeft.setPower(reverseMotorPower);
            backLeft.setPower(reverseMotorPower);
            frontRight.setPower(reverseMotorPower);
            backRight.setPower(reverseMotorPower);
            telemetry.addData("Active Motor", "Back Left");
        }
        else {
            telemetry.addData("Active Motor", "None");
        }

        // Display current motor power and button mappings
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.addData("Button Mapping", "Square=FL, Triangle=FR, Circle=BR, X=BL");
        telemetry.addData("", "");
        telemetry.addData("Motor Status", "FL=%.2f, FR=%.2f, BL=%.2f, BR=%.2f",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());

        telemetry.update();
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}
