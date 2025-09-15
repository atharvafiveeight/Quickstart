package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "AdvancedTeleop")
public class AdvancedTeleop extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<LynxModule> allHubs;
    private IMU imu;
    private static final boolean FIELD_CENTRIC = false; // Set to true for field-centric driving, false for POV
    //private Servo clawservo;
    //private Servo pivotservo;

    // Anti-drift constants
    private static final double JOYSTICK_DEADZONE = 0.15; // Increased from 0.05
    private static final double MIN_MOTOR_POWER = 0.08;   // Minimum power to overcome motor friction

    @Override
    public void runOpMode() throws InterruptedException {
        // Enables bulk reading for faster access to sensors/motors
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware map motor names to code
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Reverse left motors to match direction with right motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake when no power is applied (important for anti-drift)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and set to use them
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the IMU orientation (adjust if Control Hub is mounted differently)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);

        // Initialize the servo and map it
       // clawservo = hardwareMap.get(Servo.class, "clawservo");
        //pivotservo = hardwareMap.get(Servo.class, "pivotservo");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Clear cached data from control hubs
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // Get joystick input and apply improved deadzone
            double y = applyAdvancedDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x = applyAdvancedDeadzone(gamepad1.left_stick_x);  // Strafing
            double rx = applyAdvancedDeadzone(gamepad1.right_stick_x); // Rotation

            // Square inputs for smoother control while preserving sign
            y = Math.copySign(y * y, y);
            x = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);

            // Convert joystick direction to field-centric frame using rotation matrix
            if (FIELD_CENTRIC) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double botHeading = orientation.getYaw(AngleUnit.RADIANS);

                // Rotation transformation for joystick direction
                double temp = y * Math.cos(botHeading) - x * Math.sin(botHeading);
                x = y * Math.sin(botHeading) + x * Math.cos(botHeading);
                y = temp;
            }

            // Mecanum wheel calculations:
            // Combines forward (y), strafe (x), and turn (rx) into individual motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator; // front left
            double blPower = (y - x + rx) / denominator; // back left
            double frPower = (y - x - rx) / denominator; // front right
            double brPower = (y + x - rx) / denominator; // back right

            // Apply minimum power threshold to prevent stalling/drift
            flPower = applyMinPower(flPower);
            blPower = applyMinPower(blPower);
            frPower = applyMinPower(frPower);
            brPower = applyMinPower(brPower);

            // Apply motor power
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // Servo controls
            if(gamepad1.a) {
         //       clawservo.setPosition(0);
            }
            if(gamepad1.b) {
           //     clawservo.setPosition(1);
            }
            if(gamepad1.x) {
           //     pivotservo.setPosition(0);
            }
            if(gamepad1.y) {
            //    pivotservo.setPosition(1);
            }

            // Show telemetry
            telemetry.addData("Drive Mode", FIELD_CENTRIC ? "Field-Centric" : "POV");
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("FL", "%.3f", flPower);
            telemetry.addData("FR", "%.3f", frPower);
            telemetry.addData("BL", "%.3f", blPower);
            telemetry.addData("BR", "%.3f", brPower);
            //telemetry.addData("Claw Pos", "%.2f", clawservo.getPosition());
            //telemetry.addData("Pivot Pos", "%.2f", pivotservo.getPosition());
            telemetry.addData("Deadzone", JOYSTICK_DEADZONE);
            telemetry.update();
        }
    }

    // Improved deadzone function that scales remaining input
    private double applyAdvancedDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        // Scale the remaining input to maintain full range
        return Math.signum(value) * ((Math.abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE));
    }

    // Apply minimum power threshold to prevent motor stalling
    private double applyMinPower(double power) {
        if (Math.abs(power) > 0 && Math.abs(power) < MIN_MOTOR_POWER) {
            return Math.signum(power) * MIN_MOTOR_POWER;
        }
        return power;
    }

    // Legacy deadzone function (kept for reference)
    private double applyDeadzone(double value) {
        return Math.abs(value) > 0.05 ? value : 0.0;
    }
}