package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PIDF & Ramp Test TeleOp", group = "Tuning")
public class PIDF_Testing extends LinearOpMode {

    // --- INPUT YOUR TUNED PIDF VALUES HERE ---
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0.0, 0.0, 12.4117424);

    // Motor control modes
    private enum ControlMode {
        POWER,               // Open-loop voltage/power (-1.0 to 1.0)
        RAW_VELOCITY,        // RUN_WITHOUT_ENCODER with proportional manual power
        PIDF_VELOCITY        // RUN_USING_ENCODER with internal PIDF regulation
    }

    private DcMotorEx motor;
    private ControlMode currentMode = ControlMode.PIDF_VELOCITY;

    // Timer variables
    private ElapsedTime testTimer = new ElapsedTime();
    private boolean isTesting = false;
    private double timeToTargetSeconds = -1.0;

    // Target values
    private double targetVelocity = 1500.0; // Ticks per second
    private double targetPower = 0.8;       // Power for open-loop mode
    private final double TOLERANCE_PERCENT = 0.05; // 5% speed tolerance

    // Button edge-detection flags
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motor.setDirection(DcMotor.Direction.REVERSE);

        // Allow max achievable RPM
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply starting mode and coefficients
        setMotorMode(currentMode);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        telemetry.addLine("Ready. Press Start to run.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleModeSwitching();
            handleTargetAdjustments();
            handleTestLogic();
            updateTelemetryData();
        }
    }

    private void handleModeSwitching() {
        // Toggle control modes with the X button
        if (gamepad1.x && !lastX) {
            motor.setPower(0);
            isTesting = false;
            timeToTargetSeconds = -1.0;

            switch (currentMode) {
                case PIDF_VELOCITY:
                    currentMode = ControlMode.RAW_VELOCITY;
                    break;
                case RAW_VELOCITY:
                    currentMode = ControlMode.POWER;
                    break;
                case POWER:
                    currentMode = ControlMode.PIDF_VELOCITY;
                    break;
            }
            setMotorMode(currentMode);
        }
        lastX = gamepad1.x;
    }

    private void setMotorMode(ControlMode mode) {
        if (mode == ControlMode.PIDF_VELOCITY) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void handleTargetAdjustments() {
        // Adjust targets using D-pad
        if (currentMode == ControlMode.POWER) {
            if (gamepad1.dpad_up && !lastDpadUp) targetPower = Math.min(1.0, targetPower + 0.1);
            if (gamepad1.dpad_down && !lastDpadDown) targetPower = Math.max(0.0, targetPower - 0.1);
        } else {
            // Large adjustments with Up/Down (100 ticks/sec)
            if (gamepad1.dpad_up && !lastDpadUp) targetVelocity += 100.0;
            if (gamepad1.dpad_down && !lastDpadDown) targetVelocity = Math.max(0.0, targetVelocity - 100.0);

            // Fine adjustments with Right/Left (20 ticks/sec)
            if (gamepad1.dpad_right && !lastDpadRight) targetVelocity += 20.0;
            if (gamepad1.dpad_left && !lastDpadLeft) targetVelocity = Math.max(0.0, targetVelocity - 20.0);
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;
    }

    private void handleTestLogic() {
        // Start timed ramp test with button A
        if (gamepad1.a && !lastA && !isTesting) {
            isTesting = true;
            timeToTargetSeconds = -1.0;
            testTimer.reset();
            applyTargetOutput();
        }
        lastA = gamepad1.a;

        // Emergency / manual stop with button B
        if (gamepad1.b) {
            isTesting = false;
            motor.setPower(0);
        }

        // Manual override: hold Left Bumper to run at set target without running test timer
        if (gamepad1.left_bumper && !isTesting) {
            applyTargetOutput();
        } else if (!isTesting && !gamepad1.left_bumper) {
            motor.setPower(0);
        }

        // Monitor speed during active timer test
        if (isTesting) {
            double currentVelocity = motor.getVelocity();

            if (currentMode == ControlMode.POWER) {
                if (testTimer.seconds() > 3.0) {
                    isTesting = false;
                }
            } else {
                double lowerThreshold = targetVelocity * (1.0 - TOLERANCE_PERCENT);
                if (currentVelocity >= lowerThreshold && timeToTargetSeconds < 0) {
                    timeToTargetSeconds = testTimer.seconds();
                    isTesting = false;
                }

                // 5-second safety timeout
                if (testTimer.seconds() > 5.0 && timeToTargetSeconds < 0) {
                    timeToTargetSeconds = -999.0;
                    isTesting = false;
                }
            }
        }
    }

    private void applyTargetOutput() {
        switch (currentMode) {
            case PIDF_VELOCITY:
                motor.setVelocity(targetVelocity);
                break;
            case RAW_VELOCITY:
                double estimatedPower = Math.min(1.0, targetVelocity / 2500.0);
                motor.setPower(estimatedPower);
                break;
            case POWER:
                motor.setPower(targetPower);
                break;
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("Mode", currentMode.name());
        telemetry.addData("Motor Power", "%.3f", motor.getPower());
        telemetry.addData("Current Velocity (ticks/s)", "%.1f", motor.getVelocity());

        if (currentMode == ControlMode.POWER) {
            telemetry.addData("Target Power", "%.2f", targetPower);
        } else {
            telemetry.addData("Target Velocity (ticks/s)", "%.1f", targetVelocity);
            telemetry.addData("Velocity Error", "%.1f", targetVelocity - motor.getVelocity());
        }

        if (isTesting) {
            telemetry.addData("Test Status", "Ramping... Time: %.3f s", testTimer.seconds());
        } else if (timeToTargetSeconds > 0) {
            telemetry.addData("Test Result", "Reached target in: %.4f s", timeToTargetSeconds);
        } else if (timeToTargetSeconds == -999.0) {
            telemetry.addData("Test Result", "Timed out (> 5.0 s)");
        } else {
            telemetry.addData("Test Status", "Idle (Press A to run test)");
        }

        telemetry.update();
    }
}