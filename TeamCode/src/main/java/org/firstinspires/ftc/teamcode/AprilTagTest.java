package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagTest extends OpMode {
    private Limelight3A limelight3A;
    private AprilTagProcessor aprilTag;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(-0);
        limelight3A.start();

    }
    @Override
    public void start() {
        //limelight3A.start();
    }
    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        telemetry.addData("Version 3", 1);

        if (llResult == null) {
            telemetry.addData("LL result null", 1);
        } else {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }

        if (llResult != null && llResult.isValid()) {
                telemetry.addData(" Got results for llResult", 1);
                telemetry.addData("Target X offset", llResult.getTx());
                telemetry.addData("Target Y offset", llResult.getTy());
                telemetry.addData("Target Area offset", llResult.getTa());
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }


        } else {
            // No AprilTag detected - stop and search
            telemetry.addData("Status", "Searching for AprilTag...");
        }
        telemetry.update();
    }
}
