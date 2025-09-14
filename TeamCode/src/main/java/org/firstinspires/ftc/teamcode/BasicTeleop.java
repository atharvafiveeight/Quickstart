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


@TeleOp(name = "BasicTeleop")
public class BasicTeleop extends LinearOpMode {


    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<LynxModule> allHubs;
    private IMU imu;
    private static final boolean FIELD_CENTRIC = false; // Set to true for field-centric driving, false for POV
    //private Servo clawservo;
    //private Servo pivotservo;




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


        // Set motors to brake when no power is applied
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
        //clawservo = hardwareMap.get(Servo.class, "clawservo");
        //pivotservo = hardwareMap.get(Servo.class, "pivotservo");


        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            // Clear cached data from control hubs
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            // Get joystick input and apply deadzone
            double y = applyDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x = applyDeadzone(gamepad1.left_stick_x);  // Strafing
            double rx = applyDeadzone(gamepad1.right_stick_x); // Rotation


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


            // Apply motor power
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);


            if(gamepad1.a){;
               // clawservo.setPosition(0);
//            servo.setPower(0.5);
            }
            if(gamepad1.b){
                //clawservo.setPosition(1);
//            servo.setPower(0.6);
            }
            if(gamepad1.x){;
                //pivotservo.setPosition(0);
//            servo.setPower(0.5);
            }
            if(gamepad1.y){
                //pivotservo.setPosition(1);
//            servo.setPower(1);
            }


            // Show telemetry
            telemetry.addData("Drive Mode", FIELD_CENTRIC ? "Field-Centric" : "POV");
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("FL", flPower);
            telemetry.addData("FR", frPower);
            telemetry.addData("BL", blPower);
            telemetry.addData("BR", brPower);
            //telemetry.addData("Servo Pos", pivotservo.getPosition());
            //telemetry.addData("Servo Pos", clawservo.getPosition());
            telemetry.update();
        }
    }


    // Ignores tiny joystick inputs to prevent drift
    private double applyDeadzone(double value) {
        return Math.abs(value) > 0.05 ? value : 0.0;
    }
}

