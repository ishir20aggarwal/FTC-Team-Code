package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.TuningController;

@Config
@TeleOp(name = "VeloPIDTuner", group = "Tuning")

public class VeloPIDTuner extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 20.17037037037037037);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private VoltageSensor batteryVoltageSensor;

    // Tuning selection states
    private enum TuningParam { P, I, D, F }
    private TuningParam selectedParam = TuningParam.P;

    private final double[] STEP_FACTORS = {0.01, 0.1, 1.0, 10.0, 100.0};
    private int stepFactorIndex = 2; // Default to 1.0

    // State trackers for debouncing edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastRTrigger = false;
    private boolean lastLTrigger = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "motorLeft");

        // Reverse as appropriate
        //myMotor.setDirection(DcMotor.Direction.REVERSE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        TuningController tuningController = new TuningController();

        double lastKp = 0.0;
        double lastKi = 0.0;
        double lastKd = 0.0;
        double lastKf = getMotorVelocityF();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        tuningController.start();

        while (!isStopRequested() && opModeIsActive()) {
            // --- Gamepad Tuning Adjustments ---
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean rTrigger = gamepad1.right_trigger > 0.3;
            boolean lTrigger = gamepad1.left_trigger > 0.3;

            // Cycle parameter selection
            if (dpadDown && !lastDpadDown) {
                selectedParam = TuningParam.values()[(selectedParam.ordinal() + 1) % TuningParam.values().length];
            } else if (dpadUp && !lastDpadUp) {
                selectedParam = TuningParam.values()[(selectedParam.ordinal() - 1 + TuningParam.values().length) % TuningParam.values().length];
            }

            // Cycle step factor
            if (dpadRight && !lastDpadRight) {
                stepFactorIndex = Math.min(STEP_FACTORS.length - 1, stepFactorIndex + 1);
            } else if (dpadLeft && !lastDpadLeft) {
                stepFactorIndex = Math.max(0, stepFactorIndex - 1);
            }

            double factor = STEP_FACTORS[stepFactorIndex];

            // Increase / decrease value on trigger edge
            double delta = 0.0;
            if (rTrigger && !lastRTrigger) {
                delta += factor;
            }
            if (lTrigger && !lastLTrigger) {
                delta -= factor;
            }

            if (delta != 0.0) {
                switch (selectedParam) {
                    case P: MOTOR_VELO_PID.p = Math.max(0, MOTOR_VELO_PID.p + delta); break;
                    case I: MOTOR_VELO_PID.i = Math.max(0, MOTOR_VELO_PID.i + delta); break;
                    case D: MOTOR_VELO_PID.d = Math.max(0, MOTOR_VELO_PID.d + delta); break;
                    case F: MOTOR_VELO_PID.f = Math.max(0, MOTOR_VELO_PID.f + delta); break;
                }
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastRTrigger = rTrigger;
            lastLTrigger = lTrigger;


            // Target speed set to ~0.75 of total motor RPM
            double targetVelo = tuningController.update();
            myMotor.setVelocity(targetVelo);

            double motorVelo = myMotor.getVelocity();


            telemetry.addData("Selected Param", selectedParam.name());
            telemetry.addData("Step Factor", factor);
            telemetry.addData("P", MOTOR_VELO_PID.p);
            telemetry.addData("I", MOTOR_VELO_PID.i);
            telemetry.addData("D", MOTOR_VELO_PID.d);
            telemetry.addData("F", MOTOR_VELO_PID.f);
            telemetry.addData("targetVelocity", targetVelo);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);
            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM * 0.75 * 1.15));
            telemetry.addData("lowerBound", 0);

            // Update PID coefficients on the motor if values changed
            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p,
                coefficients.i,
                coefficients.d,
                coefficients.f * (12.0 / batteryVoltageSensor.getVoltage())
        ));
    }

    public static double getMotorVelocityF() {
        return (32767.0 * 60.0) / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }
}