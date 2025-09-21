package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class LimeLightTest extends OpMode {
private Limelight3A limelight3A;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9);
        limelight3A.start();
    }
    @Override
    public void start() {
        //limelight3A.start();
    }
    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();

        telemetry.addData("Version V:", 2);
        if (llResult != null && llResult.isValid()) {
            telemetry.addData(" Got results for llResult", 1);
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
        } else {
            telemetry.addData("Did not get results for llResult", -1);
        }
        telemetry.update();
    }
}
