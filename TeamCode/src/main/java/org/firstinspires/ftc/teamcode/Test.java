package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Just-A-Test", group="TeleOp")
public class Test extends LinearOpMode {

    private DcMotorEx topLeft, bottomLeft, topRight, bottomRight;
    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private ServoImplEx intakeServo;

    private boolean servoBusy = false;
    private boolean bPressedLast = false;

    private boolean launcherOn = false;
    private double launchPower = 0.69;

    private final double POWER_STEP = 0.02;
    private final long ADJUST_DELAY_MS = 120;
    private long lastAdjustTime = 0;

    private boolean intakeMotorsOn = false;
    private boolean xPressedLast = false;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
            handleLauncherPowerAdjust();
            handleLaunchSequence();
            handleIntakeMotorsToggle();
            launch();
            sendTelemetry();
        }
    }

    private void initHardware() {
        topLeft    = hardwareMap.get(DcMotorEx.class, "leftFront");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        topRight   = hardwareMap.get(DcMotorEx.class, "rightFront");
        bottomRight= hardwareMap.get(DcMotorEx.class, "rightBack");

        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLeft  = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        motorRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        intakeServo.setPosition(1.00);

        intakeLeft  = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        intakeRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void handleDriving() {
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.dpad_right? 1: gamepad1.dpad_left? -1: 0;
        double r = gamepad1.right_stick_x;

        topLeft.setPower(y + x + r);
        bottomLeft.setPower(y - x + r);
        topRight.setPower(y - x - r);
        bottomRight.setPower(y + x - r);
    }

    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustTime < ADJUST_DELAY_MS) return;

        if (gamepad1.right_bumper) {
            launchPower = Math.min(1.0, launchPower + POWER_STEP);
            lastAdjustTime = now;
        }
        if (gamepad1.left_bumper) {
            launchPower = Math.max(0.0, launchPower - POWER_STEP);
            lastAdjustTime = now;
        }
    }

    private void launch() {
        double p = launcherOn? -launchPower: 0;
        motorLeft.setPower(p);
        motorRight.setPower(p);
    }

    private void handleIntakeMotorsToggle() {
        if (gamepad1.x && !xPressedLast)
            intakeMotorsOn = !intakeMotorsOn;
        xPressedLast = gamepad1.x;

        intakeLeft.setPower(intakeMotorsOn ? 0.3 : 0.0);
        intakeRight.setPower(intakeMotorsOn ? 0.3 : 0.0);
    }

    private void handleLaunchSequence() {
        boolean b = gamepad1.b;

        if (b && !bPressedLast && !servoBusy) {
            servoBusy = true;

            new Thread(() -> {
                try {
                    launcherOn = true;
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.82);
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.51);
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.25);
                    sleepQuiet(200);
                    intakeServo.setPosition(1.00);

                    launcherOn = false;
                } finally {
                    servoBusy = false;
                }
            }).start();
        }

        bPressedLast = b;
    }


    private void sleepQuiet(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    private void sendTelemetry() {
        telemetry.addData("drivetrain speed", bottomRight.getPower());
        telemetry.addData("Launcher", launcherOn);
        telemetry.addData("Power", launchPower);
        telemetry.addData("Servo Busy", servoBusy);
        telemetry.addData("Servo Pos", intakeServo.getPosition());
        telemetry.addData("outtake", motorLeft.getPower());
        telemetry.update();
    }
}