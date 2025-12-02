package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * TOPPARTROBOT - Top part of robot control system
 * 
 * Controls:
 * - Right Bumper: Intake motor runs at 0.75 power in reverse
 * - Left Bumper: Simultaneously runs:
 *   - Outtake motor at 0.5 power
 *   - Two shooter motors with velocity control (similar to RED_NOVTELEOP)
 *   - Intake motor at 0.5 power
 * 
 * @author Sahas Kumar
 */
@TeleOp(name = "TOPPARTROBOT", group = "TeleOp")
public class TOPPARTROBOT extends LinearOpMode {

    // Motor declarations
    private DcMotorEx intakeMotor;           // Intake motor
    private DcMotorEx outtakeMotor;          // Outtake motor (first level)
    private DcMotorEx shooterMotor1;          // First shooter motor (velocity control)
    private DcMotorEx shooterMotor2;          // Second shooter motor (velocity control)
    
    // Power constants
    private static final double INTAKE_POWER_HIGH = 0.95;    // Intake motor power for right bumper
    private static final double INTAKE_POWER_LOW = 0.8;    // Intake motor power for X button
    private static final double OUTTAKE_MOTOR_POWER = 1;  // Power for outtake motor
    
    // Velocity constants (similar to RED_NOVTELEOP)
    // Both velocity motors use the same target to ensure equal power when combined
    private static final double VELOCITY_TARGET = 1500;     // Target velocity in RPM (shared by both motors)
    
    // Outtake motor control - velocity-based only (no timer)
    private boolean shootersAtSpeed = false;  // Track if shooters have reached target speed
    private boolean leftBumperWasPressed = false;  // Track previous state to detect button press
    private static final double MIN_VELOCITY_THRESHOLD = 0.80;  // 60% of target velocity (900 RPM for 1500 target) - lower for faster response
    private static final double MIN_ABSOLUTE_VELOCITY = 500.0;  // Minimum absolute velocity to ensure motors are actually spinning
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // ========================================
        // STEP 1: INITIALIZE MOTORS
        // ========================================
        boolean motorsInitialized = true;
        
        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            if (intakeMotor == null) {
                telemetry.addData("ERROR", "Intake motor 'intakeMotor' not found in hardware map!");
                motorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize intake motor: " + e.getMessage());
            motorsInitialized = false;
        }
        
        // Initialize outtake motor
        try {
            outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
            if (outtakeMotor == null) {
                telemetry.addData("ERROR", "Outtake motor 'outtakeMotor' not found in hardware map!");
                motorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize outtake motor: " + e.getMessage());
            motorsInitialized = false;
        }
        
        // Initialize shooter motor 1
        try {
            shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
            if (shooterMotor1 == null) {
                telemetry.addData("ERROR", "Shooter motor 1 'shooterMotor1' not found in hardware map!");
                motorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor 1: " + e.getMessage());
            motorsInitialized = false;
        }
        
        // Initialize shooter motor 2
        try {
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
            if (shooterMotor2 == null) {
                telemetry.addData("ERROR", "Shooter motor 2 'shooterMotor2' not found in hardware map!");
                motorsInitialized = false;
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize shooter motor 2: " + e.getMessage());
            motorsInitialized = false;
        }
        
        if (!motorsInitialized) {
            telemetry.addData("CRITICAL ERROR", "Some motors failed to initialize!");
        }
        
        // ========================================
        // STEP 2: CONFIGURE MOTOR DIRECTIONS
        // ========================================
        // Set motor directions
        if (intakeMotor != null) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse direction for intake
        }
        if (outtakeMotor != null) {
            outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            // Set to RUN_WITHOUT_ENCODER for maximum speed (no encoder limiting)
            outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (shooterMotor2 != null) {
            shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        
        // ========================================
        // STEP 3: CONFIGURE MOTOR BEHAVIOR
        // ========================================
        // Set motors to brake when no power is applied
        if (intakeMotor != null) intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (outtakeMotor != null) outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shooterMotor1 != null) shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shooterMotor2 != null) shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // ========================================
        // STEP 4: SET UP SHOOTER MOTORS (VELOCITY CONTROL)
        // ========================================
        // Configure shooter motors (similar to RED_NOVTELEOP)
        // Both motors use identical settings to ensure equal power when combined
        if (shooterMotor1 != null) {
            try {
                shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // Set PIDF coefficients for stable velocity control
                // P=3 (proportional), I=0 (integral), D=0 (derivative), F=15 (feedforward - increased for more power)
                // Same coefficients for both motors to ensure synchronized behavior
                // Increased F value from 10 to 15 to provide more initial power
                shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(30, 0, 0, 15));
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to configure shooter motor 1: " + e.getMessage());
            }
        }
        
        if (shooterMotor2 != null) {
            try {
                shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // Set PIDF coefficients for stable velocity control
                // IDENTICAL to motor 1 to ensure both motors reach the same velocity
                // Increased F value from 10 to 15 to provide more initial power
                shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(10, 0, 0, 15));
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to configure shooter motor 2: " + e.getMessage());
            }
        }
        
        // ========================================
        // STEP 5: STOP ALL MOTORS INITIALLY
        // ========================================
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (outtakeMotor != null) outtakeMotor.setPower(0);
        if (shooterMotor1 != null) {
            try {
                shooterMotor1.setVelocity(0);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to stop shooter motor 1: " + e.getMessage());
            }
        }
        if (shooterMotor2 != null) {
            try {
                shooterMotor2.setVelocity(0);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Failed to stop shooter motor 2: " + e.getMessage());
            }
        }
        
        // ========================================
        // STEP 6: READY TO START
        // ========================================
        telemetry.addData("Status", "TOPPARTROBOT Ready!");
        telemetry.addData("Controls", "Right Bumper: Intake reverse (0.75)");
        telemetry.addData("Controls", "Left Bumper: Outtake + Shooters + Intake");
        telemetry.update();
        
        waitForStart();
        
        // ========================================
        // STEP 7: MAIN CONTROL LOOP
        // ========================================
        while (opModeIsActive() && !isStopRequested()) {
            
            // ========================================
            // LEFT BUMPER: Simultaneous control of outtake, shooters, and intake
            // Left bumper takes precedence - when pressed, it controls all motors
            // ========================================
            if (gamepad1.left_bumper) {
                // Reset state when button is first pressed (edge detection)
                if (!leftBumperWasPressed) {
                    shootersAtSpeed = false;  // Reset state when button is first pressed
                }
                leftBumperWasPressed = true;
                
                // Step 1: Start shooter motors and set target velocity
                if (shooterMotor1 != null) {
                    try {
                        if (shooterMotor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                            shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                        shooterMotor1.setVelocity(VELOCITY_TARGET);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Shooter 1 error: " + e.getMessage());
                    }
                }
                
                if (shooterMotor2 != null) {
                    try {
                        if (shooterMotor2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                            shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                        shooterMotor2.setVelocity(VELOCITY_TARGET);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Shooter 2 error: " + e.getMessage());
                    }
                }
                
                // Step 2: Check if shooters have reached target velocity
                // NEW APPROACH: Velocity-only check with dual criteria
                // 1. Must be above percentage threshold (60% of target)
                // 2. Must be above absolute minimum (500 RPM) to ensure motors are actually spinning
                double minVelocity = VELOCITY_TARGET * MIN_VELOCITY_THRESHOLD; // 60% threshold (900 RPM for 1500 target)
                boolean velocityCheckPassed = false;
                
                if (shooterMotor1 != null && shooterMotor2 != null) {
                    try {
                        double vel1 = Math.abs(shooterMotor1.getVelocity());
                        double vel2 = Math.abs(shooterMotor2.getVelocity());
                        
                        // Both motors must meet BOTH criteria:
                        // 1. Above percentage threshold (60% of target)
                        // 2. Above absolute minimum (500 RPM) to ensure they're actually spinning
                        boolean motor1Ready = (vel1 >= minVelocity) && (vel1 >= MIN_ABSOLUTE_VELOCITY);
                        boolean motor2Ready = (vel2 >= minVelocity) && (vel2 >= MIN_ABSOLUTE_VELOCITY);
                        velocityCheckPassed = motor1Ready && motor2Ready;
                        
                        // Show detailed status
                        telemetry.addData("Shooter Status", "M1: " + String.format("%.0f", vel1) + 
                            " | M2: " + String.format("%.0f", vel2) + " RPM");
                        telemetry.addData("Threshold", String.format("%.0f", minVelocity) + " RPM (60% of " + 
                            String.format("%.0f", VELOCITY_TARGET) + ")");
                        telemetry.addData("M1 Ready", motor1Ready ? "YES" : "NO");
                        telemetry.addData("M2 Ready", motor2Ready ? "YES" : "NO");
                        telemetry.addData("Velocity Check", velocityCheckPassed ? "PASSED ✓" : "WAITING...");
                        
                    } catch (Exception e) {
                        velocityCheckPassed = false;
                        telemetry.addData("ERROR", "Can't read velocities: " + e.getMessage());
                    }
                } else if (shooterMotor1 != null) {
                    try {
                        double vel1 = Math.abs(shooterMotor1.getVelocity());
                        velocityCheckPassed = (vel1 >= minVelocity) && (vel1 >= MIN_ABSOLUTE_VELOCITY);
                        telemetry.addData("Shooter 1", String.format("%.0f RPM", vel1));
                        telemetry.addData("Shooter 1 Ready", velocityCheckPassed ? "YES" : "NO");
                    } catch (Exception e) {
                        velocityCheckPassed = false;
                        telemetry.addData("ERROR", "Can't read shooter 1: " + e.getMessage());
                    }
                } else if (shooterMotor2 != null) {
                    try {
                        double vel2 = Math.abs(shooterMotor2.getVelocity());
                        velocityCheckPassed = (vel2 >= minVelocity) && (vel2 >= MIN_ABSOLUTE_VELOCITY);
                        telemetry.addData("Shooter 2", String.format("%.0f RPM", vel2));
                        telemetry.addData("Shooter 2 Ready", velocityCheckPassed ? "YES" : "NO");
                    } catch (Exception e) {
                        velocityCheckPassed = false;
                        telemetry.addData("ERROR", "Can't read shooter 2: " + e.getMessage());
                    }
                } else {
                    velocityCheckPassed = false;
                    telemetry.addData("ERROR", "No shooter motors available!");
                }
                
                // Step 3: Set shooters ready status (velocity-only, no timer)
                shootersAtSpeed = velocityCheckPassed;
                
                telemetry.addData("Shooters Ready", shootersAtSpeed ? "YES - Outtake RUNNING" : "NO - Outtake WAITING");
                
                // Step 4: Control outtake motor based on shooter status
                // When shooters are ready, run outtake at maximum power for fastest ball ejection
                if (outtakeMotor != null) {
                    if (shootersAtSpeed) {
                        // Set to maximum power (1.0) for fastest outtake speed
                        outtakeMotor.setPower(1.0);  // Maximum power for fastest ball pushing
                        telemetry.addData("Outtake Power", "MAX (1.0) - Running at full speed");
                    } else {
                        outtakeMotor.setPower(0);
                        telemetry.addData("Outtake Power", "0.0 - Waiting for shooters");
                    }
                }
                
                // Step 6: Intake motor at 0.5 power (always runs when left bumper is active)
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_POWER_LOW);
                }
                
            } else {
                // Left bumper released - reset state and stop motors
                if (leftBumperWasPressed) {
                    shootersAtSpeed = false;
                    leftBumperWasPressed = false;
                }
                
                if (outtakeMotor != null) {
                    outtakeMotor.setPower(0);
                }
            }
            
            // ========================================
            // RIGHT BUMPER: Intake motor reverse at 0.75 power (only intake)
            // Only active if left bumper is NOT pressed
            // ========================================
            if (!gamepad1.left_bumper && gamepad1.right_bumper) {
                if (intakeMotor != null) {
                    intakeMotor.setPower(INTAKE_POWER_HIGH);
                }
            }
            
            // ========================================
            // STOP MOTORS WHEN BUTTONS ARE RELEASED
            // ========================================
            // Stop intake motor if neither bumper is pressed
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                if (intakeMotor != null) {
                    intakeMotor.setPower(0);
                }
            }
            
            // Stop shooter motors if left bumper is not pressed
            // (Outtake motor is already handled in the left bumper else block above)
            if (!gamepad1.left_bumper) {
                if (shooterMotor1 != null) {
                    try {
                        shooterMotor1.setVelocity(0);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to stop shooter motor 1: " + e.getMessage());
                    }
                }
                
                if (shooterMotor2 != null) {
                    try {
                        shooterMotor2.setVelocity(0);
                    } catch (Exception e) {
                        telemetry.addData("ERROR", "Failed to stop shooter motor 2: " + e.getMessage());
                    }
                }
            }
            
            // ========================================
            // UPDATE TELEMETRY
            // ========================================
            telemetry.clear();
            telemetry.addLine("=== TOPPARTROBOT ===");
            telemetry.addData("Right Bumper", gamepad1.right_bumper ? "PRESSED" : "RELEASED");
            telemetry.addData("Left Bumper", gamepad1.left_bumper ? "PRESSED" : "RELEASED");
            
            if (intakeMotor != null) {
                telemetry.addData("Intake Motor Power", String.format("%.2f", intakeMotor.getPower()));
            }
            
            if (outtakeMotor != null) {
                telemetry.addData("Outtake Motor Power", String.format("%.2f", outtakeMotor.getPower()));
                if (gamepad1.left_bumper && outtakeMotor.getPower() == 0) {
                    telemetry.addData("Outtake Status", "WAITING for shooters to reach speed");
                } else if (gamepad1.left_bumper && outtakeMotor.getPower() > 0) {
                    telemetry.addData("Outtake Status", "RUNNING");
                }
            }
            
            // Display velocity for both shooter motors to verify they're synchronized
            if (shooterMotor1 != null && shooterMotor2 != null) {
                try {
                    double vel1 = Math.abs(shooterMotor1.getVelocity());
                    double vel2 = Math.abs(shooterMotor2.getVelocity());
                    double target = VELOCITY_TARGET;
                    double effectiveTarget = VELOCITY_TARGET * 0.90; // Same threshold used for outtake logic
                    
                    telemetry.addData("Shooter Motor 1", String.format("%.1f RPM (target: %.0f)", vel1, target));
                    telemetry.addData("Shooter Motor 2", String.format("%.1f RPM (target: %.0f)", vel2, target));
                    
                    // Show if motors are ready for outtake
                    boolean bothReady = (vel1 >= effectiveTarget && vel2 >= effectiveTarget);
                    if (gamepad1.left_bumper) {
                        telemetry.addData("Shooters Ready", bothReady ? "YES (outtake can run)" : "NO (waiting...)");
                        telemetry.addData("Threshold", String.format("%.0f RPM (90%% of target)", effectiveTarget));
                    }
                    
                    // Show synchronization status
                    double difference = Math.abs(vel1 - vel2);
                    if (difference < 50) { // Within 50 RPM is considered synchronized
                        telemetry.addData("Shooters Sync", "SYNCHRONIZED (diff: " + String.format("%.1f", difference) + " RPM)");
                    } else {
                        telemetry.addData("Shooters Sync", "NOT SYNCED (diff: " + String.format("%.1f", difference) + " RPM)");
                    }
                } catch (Exception e) {
                    telemetry.addData("Shooter Motors", "ERROR: " + e.getMessage());
                }
            } else {
                // Individual motor display if one is missing
                if (shooterMotor1 != null) {
                    try {
                        double vel1 = Math.abs(shooterMotor1.getVelocity());
                        double effectiveTarget = VELOCITY_TARGET * 0.90;
                        telemetry.addData("Shooter Motor 1", String.format("%.1f RPM", vel1));
                        if (gamepad1.left_bumper) {
                            telemetry.addData("Shooter 1 Ready", (vel1 >= effectiveTarget) ? "YES" : "NO");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Shooter Motor 1", "ERROR");
                    }
                }
                
                if (shooterMotor2 != null) {
                    try {
                        double vel2 = Math.abs(shooterMotor2.getVelocity());
                        double effectiveTarget = VELOCITY_TARGET * 0.90;
                        telemetry.addData("Shooter Motor 2", String.format("%.1f RPM", vel2));
                        if (gamepad1.left_bumper) {
                            telemetry.addData("Shooter 2 Ready", (vel2 >= effectiveTarget) ? "YES" : "NO");
                        }
                    } catch (Exception e) {
                        telemetry.addData("Shooter Motor 2", "ERROR");
                    }
                }
            }
            
            telemetry.update();
        }
        
        // ========================================
        // STEP 8: STOP ALL MOTORS ON EXIT
        // ========================================
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (outtakeMotor != null) outtakeMotor.setPower(0);
        if (shooterMotor1 != null) {
            try {
                shooterMotor1.setVelocity(0);
            } catch (Exception e) {
                // Ignore errors on shutdown
            }
        }
        if (shooterMotor2 != null) {
            try {
                shooterMotor2.setVelocity(0);
            } catch (Exception e) {
                // Ignore errors on shutdown
            }
        }
    }
}

