package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Programmer", group = "Tuning")
public class ServoProgrammer extends LinearOpMode {

    private Servo servoLeft;
    private Servo servoRight;

    private double minPosition = 0.0;
    private double maxPosition = 1.0;
    private double currentPosition = 0.5;

    private boolean dpadUpPressedLast = false;
    private boolean dpadDownPressedLast = false;

    @Override
    public void runOpMode() {
        // Initialize servos
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        // Reverse right servo so they move in unison
        servoRight.setDirection(Servo.Direction.REVERSE);

        // Move to default center position
        setPosition(0.5);

        telemetry.addData("Status", "Initialized. Press play to tune.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Fine adjustments with D-pad
            if (gamepad1.dpad_up && !dpadUpPressedLast) {
                adjustPosition(0.01);
            }
            if (gamepad1.dpad_down && !dpadDownPressedLast) {
                adjustPosition(-0.01);
            }
            dpadUpPressedLast = gamepad1.dpad_up;
            dpadDownPressedLast = gamepad1.dpad_down;

            // Quick preset jumps
            if (gamepad1.x) setPosition(0.2);
            if (gamepad1.y) setPosition(0.5);
            if (gamepad1.b) setPosition(0.8);

            // Telemetry
            telemetry.addData("Current Position", "%.3f", currentPosition);
            telemetry.addData("Current Position Left:",  servoLeft.getPosition());
            telemetry.addData("Current Position Right",  servoRight.getPosition());
            telemetry.addLine("Controls:");
            telemetry.addLine("  D-Pad Up/Down: ±0.01");
            telemetry.addLine("  X: 0.20 | Y: 0.50 | B: 0.80");
            telemetry.update();
        }
    }

    public void setPosition(double targetPosition) {
        currentPosition = Range.clip(targetPosition, minPosition, maxPosition);
        servoLeft.setPosition(currentPosition);
        servoRight.setPosition(currentPosition);
    }

    public void adjustPosition(double delta) {
        setPosition(currentPosition + delta);
    }
}