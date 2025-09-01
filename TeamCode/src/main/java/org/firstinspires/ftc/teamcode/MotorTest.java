package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "MotorTest", group = "Test")
public class MotorTest extends LinearOpMode {


    private DcMotor backLeft;


    @Override
    public void runOpMode() {


        backLeft = hardwareMap.get(DcMotor.class, "backLeft");


        backLeft.setDirection(DcMotor.Direction.REVERSE);


        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (opModeIsActive()) {


            double joystickInput = -this.gamepad1.left_stick_y;
            double motorPower = joystickInput;


            backLeft.setPower(motorPower);


            telemetry.addData("Motor Power", backLeft.getPower());
            telemetry.update();
        }
    }
}


