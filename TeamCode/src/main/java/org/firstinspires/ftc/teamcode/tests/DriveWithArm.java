package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive with Arm", group = "TeleOp")
public class DriveWithArm extends OpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set arm motor to brake and hold position
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive controls
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize drive powers
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Arm controls
        double armPower = 0;
        if (gamepad1.dpad_up) {
            armPower = 0.5;  // Raise arm
        } else if (gamepad1.dpad_down) {
            armPower = -0.5; // Lower arm
        }

        armMotor.setPower(armPower);

        // Telemetry
        telemetry.addData("Left Drive", "%.2f", leftPower);
        telemetry.addData("Right Drive", "%.2f", rightPower);
        telemetry.addData("Arm Power", "%.2f", armPower);
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.update();
    }
}
