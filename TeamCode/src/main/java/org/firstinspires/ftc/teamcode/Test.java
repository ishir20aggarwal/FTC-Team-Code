package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@TeleOp(name="SAHIL USE THIS", group="TeleOp")
public class Test extends LinearOpMode {

    private DcMotorEx topLeft, bottomLeft, topRight, bottomRight;
    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private ServoImplEx intakeServo;
    private ServoImplEx servoStopper;

    private volatile boolean isLaunching = false;
    private MecanumDrive drive;

    private boolean servoBusy = false;
    private boolean bPressedLast = false;

    private boolean launcherOn = false;
    private double launchPower = 1500;
    private double launchError = 10;

    double FR = 14.11332330;
    double PR = 500;

    double FL = 14.05918546;
    double PL = 500;

    private final double POWER_STEP = 10.00;
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
            // REQUIRED: keep RoadRunner odometry updated every loop
            drive.updatePoseEstimate();
            launcherOn = true;

            if(!isLaunching == true) {
                servoStopper.setPosition(0.53);
            }
            else{
                servoStopper.setPosition(0.1);

            }


            //handleLockToggle();

            /*if (parkLocked) {
                lockWheels();
            } else {

            }*/
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
        //motorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficientsL = new PIDFCoefficients(PL, 0, 0, FL);
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR, 0, 0, FR);

        motorLeft.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficientsL
        );
        motorRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficientsR
        );


        //motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsL);
        //motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);


        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        intakeServo.setPosition(1.00);

        servoStopper = hardwareMap.get(ServoImplEx.class, "servoStopper");


        intakeLeft  = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

        // REQUIRED: initialize RoadRunner drive so lock can command motors + read odometry
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    private void handleDriving() {
        double y = -gamepad1.left_stick_y;      // forward/back
        double r = gamepad1.right_stick_x;      // rotation

        // Triggers for TRUE strafe
        double x = gamepad1.right_trigger - gamepad1.left_trigger;
        x *= 0.60;

        // Mecanum math (correct)
        double fl = y + x + r;
        double bl = y - x + r;
        double fr = y - x - r;
        double br = y + x - r;

        // Normalize so nothing exceeds 1
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))
        );
        if (Math.abs(x) > 0.05) {
            r = 0;
        }

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        topLeft.setPower(fl);
        bottomLeft.setPower(bl);
        topRight.setPower(fr);
        bottomRight.setPower(br);
    }

    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustTime < ADJUST_DELAY_MS) return;

        if (gamepad1.right_bumper) {
            launchPower = Math.min(1.0, launchPower + POWER_STEP);
            lastAdjustTime = now;
        }
        if (gamepad1.left_bumper) {
            launchPower = Math.max(1.0, launchPower - POWER_STEP);
            lastAdjustTime = now;
        }
    }

    private void launch() {
        motorLeft.setVelocity(launchPower);
        motorRight.setVelocity(launchPower);
        //motorLeft.setVelocityPIDFCoefficients(PL, 0, 0, FL);
        //motorRight.setVelocityPIDFCoefficients(PR, 0, 0, FR);


    }



    private void handleIntakeMotorsToggle() {
        if (gamepad1.x && !xPressedLast) {
            intakeMotorsOn = !intakeMotorsOn;
        }
        xPressedLast = gamepad1.x;
        if(!servoBusy) {
            intakeLeft.setPower(intakeMotorsOn ? 0.8 : 0.0);
            intakeRight.setPower(intakeMotorsOn ? 0.8 : 0.0);
        }
    }

    private void handleLaunchSequence() {
        boolean b = gamepad1.b;

        if (b && !bPressedLast && !servoBusy) {
            servoBusy = true;

            new Thread(() -> {
                try {
                    isLaunching = true;
                    launcherOn = true;
                    intakeRight.setPower(0.2);
                    intakeLeft.setPower(0.2);
                    servoStopper.setPosition(0.4);

                    int shoots = 0;
                    double[] shooting = {0.72, 0.50, 0.24};

                    while (opModeIsActive() && shoots < 3) {
                        if (Math.abs(motorLeft.getVelocity() - 0.9*launchPower) < launchError && Math.abs(motorRight.getVelocity() - launchPower) < launchError || gamepad1.dpad_up) {
                            intakeServo.setPosition(shooting[shoots]);
                            shoots++;
                            sleepQuiet(450);
                        }
                    }
                    intakeServo.setPosition(1.00);
                } finally {
                    isLaunching = false;
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
        telemetry.addData("intake", intakeLeft.getPower());
        telemetry.addData("Servo Stopper Pos",servoStopper.getPosition());
        telemetry.addData("|", "|");
        telemetry.addData("|", "|");
        telemetry.addData("|", "|");
        telemetry.addData("KAVIN - left outtake velcoity", motorLeft.getVelocity());
        telemetry.addData("KAVIN - right outtake velocity", motorRight.getVelocity());
        telemetry.addData("SUNMAY - left outtake rpm", motorLeft.getVelocity() / 28 * 60);
        telemetry.addData("SUNMAY - right outtake rpm", motorRight.getVelocity() / 28 * 60);
        telemetry.addData("|", "|");
        telemetry.addData("|", "|");
        telemetry.addData("|", "|");
        telemetry.addData("Target RPM", launchPower);
        telemetry.addData("Left RPM", motorLeft.getVelocity());
        telemetry.addData("Right RPM", motorRight.getVelocity());
        telemetry.addData("L-R Diff", motorLeft.getVelocity() - motorRight.getVelocity());

        telemetry.update();
    }
}
