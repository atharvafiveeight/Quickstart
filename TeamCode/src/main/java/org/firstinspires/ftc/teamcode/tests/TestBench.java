package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// TestBench will run test for motor, limelight and servo based on what button is pressed
// y : motor movement ; b : limelight ; a : servo
 public class TestBench extends OpMode {

        // Motor declarations
        private DcMotor motor;
        private Limelight3A limelight3A;
        public Servo servo;
        private double motorPower = 0.5; // Adjust this value as needed (0.0 to 1.0)

        @Override
        public void init() {
            // motor initialization
           motor = hardwareMap.get(DcMotor.class, "motor");
           motor.setDirection(DcMotor.Direction.FORWARD);

           // initialize limlight
            limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
            limelight3A.pipelineSwitch(-0);

            // initialize servo
            servo = hardwareMap.get(Servo.class, "servo");

        }

        @Override
        public void loop() {
            // Stop all motors first
            stopAllTestBench();

            // motor test
            if (gamepad1.y) {
                motor.setPower(motorPower);
                telemetry.addData("Active Motor", "Motor");

            }
            // limelight test
            if (gamepad1.b) {
                // add limelight
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    telemetry.addData("Target X offset", llResult.getTx());
                    telemetry.addData("Target Y offset", llResult.getTy());
                    telemetry.addData("Target Area offset", llResult.getTa());
                }
            }
            if (gamepad1.a) {
                servo.setPosition(0);
                servo.setDirection(Servo.Direction.FORWARD);
                servo.setPosition(1.0);

            }
        }

        private void stopAllTestBench() {
            motor.setPower(0);
            limelight3A.stop();
        }
 }

