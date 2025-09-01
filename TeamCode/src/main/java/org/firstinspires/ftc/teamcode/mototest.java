package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "mototest")
public class mototest extends LinearOpMode {


    private DcMotorEx slide;
    private ElapsedTime runtime = new ElapsedTime();
    private VoltageSensor voltageSensor;


    // Test configuration
    private static final int TARGET_POSITION = 10000; // 10,000 encoder counts
    private static final double MOTOR_POWER = 1.0;    // Full speed (100%)
    private static final int NUM_RUNS = 5;             // Number of test runs
    private static final double PAUSE_BETWEEN_RUNS = 2.0; // Seconds between runs


    private boolean testCompleted = false;
    private boolean testRunning = false;
    private double totalTestDuration = 0; // Store total duration for all runs
    private double[] runSpeeds = new double[NUM_RUNS]; // Store speed for each run
    private int currentRun = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize the slide motor
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setDirection(DcMotorEx.Direction.REVERSE); // Match your main code direction
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        // Reset encoder and set mode
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use RUN_USING_ENCODER initially


        // Read initial battery voltage
        double batteryVoltage = voltageSensor.getVoltage();


        // Display initialization info
        telemetry.addLine("=== MOTOR TEST READY ===");
        telemetry.addData("Battery Voltage", String.format("%.2f V", batteryVoltage));
        telemetry.addData("Target Position", TARGET_POSITION);
        telemetry.addData("Motor Power", MOTOR_POWER);
        telemetry.addLine("Press X button to start test");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            // Wait for X button press to start test
            if (gamepad1.cross && !testRunning && !testCompleted) {
                runMotorTest();
            }


            // Allow resetting the test with Y button
            if (gamepad1.triangle && testCompleted) {
                resetTest();
            }


            // Display waiting message if test hasn't started
            if (!testRunning && !testCompleted) {
                double currentVoltage = voltageSensor.getVoltage();
                telemetry.addLine("=== MOTOR TEST READY ===");
                telemetry.addData("Battery Voltage", String.format("%.2f V", currentVoltage));
                telemetry.addData("Target Position", TARGET_POSITION);
                telemetry.addData("Motor Power", MOTOR_POWER);
                telemetry.addData("Number of Runs", NUM_RUNS);
                telemetry.addData("Pause Between Runs", PAUSE_BETWEEN_RUNS + " sec");
                telemetry.addLine("Press X button to start test");
                telemetry.update();
            }


            // Display results if test is completed
            if (testCompleted && !testRunning) {
                displayTestResults();
            }


            sleep(50); // Small delay to prevent excessive polling
        }
    }


    private void runMotorTest() {
        testRunning = true;
        currentRun = 0;
        totalTestDuration = 0;
        double batteryVoltage = voltageSensor.getVoltage();


        ElapsedTime totalTimer = new ElapsedTime();
        totalTimer.reset();


        if (opModeIsActive()) {
            telemetry.addLine("=== STARTING 5-RUN TEST ===");
            telemetry.addData("Battery Voltage", String.format("%.2f V", batteryVoltage));
            telemetry.update();
            sleep(1000); // Brief pause before starting


            // Run the test 5 times
            for (int run = 1; run <= NUM_RUNS && opModeIsActive() && testRunning; run++) {
                currentRun = run;


                // Reset motor position for each run
                slide.setPower(0);
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500); // Wait for encoder reset


                // Start individual run
                telemetry.addLine("=== RUN " + run + " OF " + NUM_RUNS + " ===");
                telemetry.addData("Battery Voltage", String.format("%.2f V", batteryVoltage));
                telemetry.addLine("Starting...");
                telemetry.update();


                runtime.reset();
                int startPosition = slide.getCurrentPosition();


                // Set target and start motor
                slide.setTargetPosition(TARGET_POSITION);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(MOTOR_POWER);


                // Monitor this run
                while (opModeIsActive() && slide.isBusy() && testRunning) {
                    int currentPosition = slide.getCurrentPosition();
                    double currentTime = runtime.seconds();


                    telemetry.addLine("=== RUN " + run + " OF " + NUM_RUNS + " ===");
                    telemetry.addData("Battery Voltage", String.format("%.2f V", batteryVoltage));
                    telemetry.addData("Current Position", currentPosition);
                    telemetry.addData("Progress", String.format("%.1f%%", (currentPosition * 100.0) / TARGET_POSITION));
                    telemetry.addData("Run Time", String.format("%.2f sec", currentTime));
                    telemetry.update();


                    sleep(100);
                }


                // Record results for this run
                double runTime = runtime.seconds();
                int finalPosition = slide.getCurrentPosition();
                double speed = finalPosition / runTime;
                runSpeeds[run - 1] = speed;


                slide.setPower(0);


                // Display run results
                telemetry.addLine("=== RUN " + run + " COMPLETED ===");
                telemetry.addData("Run Duration", String.format("%.3f sec", runTime));
                telemetry.addData("Counts/Second", String.format("%.1f", speed));
                telemetry.addData("Final Position", finalPosition);
                telemetry.update();


                // Pause between runs (except after last run)
                if (run < NUM_RUNS && opModeIsActive() && testRunning) {
                    for (int i = (int)PAUSE_BETWEEN_RUNS; i > 0 && opModeIsActive() && testRunning; i--) {
                        telemetry.addLine("=== PAUSE BETWEEN RUNS ===");
                        telemetry.addData("Next run starts in", i + " seconds");
                        telemetry.addData("Completed runs", run + " of " + NUM_RUNS);
                        telemetry.update();
                        sleep(1000);
                    }
                }
            }


            // All runs completed
            totalTestDuration = totalTimer.seconds();
            testRunning = false;
            testCompleted = true;


            displayTestResults();
        }
    }


    private void displayTestResults() {
        double batteryVoltage = voltageSensor.getVoltage();


        // Calculate average speed
        double totalSpeed = 0;
        for (double speed : runSpeeds) {
            totalSpeed += speed;
        }
        double averageSpeed = totalSpeed / NUM_RUNS;


        telemetry.clear();
        telemetry.addLine("=== 5-RUN TEST COMPLETED ===");
        telemetry.addData("🔋 Battery Voltage", String.format("%.2f V", batteryVoltage));
        telemetry.addData("⏱️ TOTAL DURATION", String.format("%.3f seconds", totalTestDuration));
        telemetry.addLine("");


        // Display individual run speeds
        telemetry.addLine("📊 INDIVIDUAL RUN SPEEDS:");
        for (int i = 0; i < NUM_RUNS; i++) {
            telemetry.addData("Run " + (i + 1), String.format("%.1f counts/sec", runSpeeds[i]));
        }
        telemetry.addLine("");


        telemetry.addData("⚡ AVERAGE SPEED", String.format("%.1f counts/sec", averageSpeed));


        telemetry.addLine("");
        telemetry.addLine("🔄 Press Y to reset test");
        telemetry.update();
    }


    private void resetTest() {
        testCompleted = false;
        testRunning = false;
        totalTestDuration = 0;
        currentRun = 0;
        // Clear all run speeds
        for (int i = 0; i < NUM_RUNS; i++) {
            runSpeeds[i] = 0;
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

