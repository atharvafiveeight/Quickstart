package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class servotest extends OpMode {
    //    public CRServo servo;
    public Servo pivotservo;
    @Override
    public void init() {
        pivotservo = hardwareMap.get(Servo.class, "servo");
//        servo = hardwareMap.get(CRServo.class, "servo");
    }


    @Override
    public void loop() {


        if(gamepad1.dpad_down){;
            pivotservo.setPosition(0);
//            servo.setPower(1);
        }
        if(gamepad1.dpad_down){
            pivotservo.setPosition(1);
//            servo.setPower(-1);
        }


//        servo.setPower(0);
    }
}

