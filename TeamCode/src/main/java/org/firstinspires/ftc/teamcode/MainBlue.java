package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Main Blue", group="TeleOp")
public class MainBlue extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private ServoImplEx intakeServo;
    private ServoImplEx servoStopper;

    private MecanumDrive drive;

    // ================= LOCKING =================
    private boolean locking = false;
    private boolean dpadLeftLast = false;

    // This is the pose we want to HOLD (do NOT overwrite every loop)
    private Pose2d lockPose = new Pose2d(0, 0, 0);

    // Gains for lockTo
    private double xyP = .2;
    private double headingP = .2;

    // ================= LAUNCH ALIGN =================
    private volatile boolean isLaunching = false;
    private volatile Vector2d launchTarget = new Vector2d(-58, -45);

    private double LAUNCH_XY_P = 0.10;
    private double LAUNCH_HEADING_P = 0.5;

    // ================= LAUNCHER / INTAKE =================
    private double launchPower = -0.77;

    private boolean servoBusy = false;
    private boolean bPressedLast = false;

    private boolean intakeMotorsOn = false;
    private boolean xPressedLast = false;

    private final double POWER_STEP = 0.02;
    private final long ADJUST_DELAY_MS = 100;
    private long lastAdjustTime = 0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            PoseStorage.lastPose = drive.localizer.getPose();

            // Stopper position depends on launching
//            if (!isLaunching) servoStopper.setPosition(0.53);
//            else servoStopper.setPosition(0.10);

            // Toggle lock with dpad_left (captures pose ONCE)
            boolean dpadLeft = gamepad1.dpad_left;
            if (dpadLeft && !dpadLeftLast && !servoBusy) {
                locking = !locking;
                if (locking) {
                    lockPose = drive.localizer.getPose(); // capture once
                }
            }
            dpadLeftLast = dpadLeft;

            // MAIN CONTROL: only one thing commands drive each loop
            if (isLaunching) {
                // Hold the ORIGINAL lockPose and face launchTarget
                lockXYAndFacePoint(lockPose, launchTarget);
            } else if (locking) {
                // Hold lockPose (return if pushed)
                lockTo(lockPose);
            } else {
                handleDriving();
            }

            if(gamepad1.dpad_down && gamepad1.a){
                drive.localizer.setPose(new Pose2d(0,0,Math.toRadians(180)));

            }

            handleLauncherPowerAdjust();
            handleLaunchSequence();
            handleIntakeMotorsToggle();
            updateLauncherMotors();
            HumanPlayer();
            manualOverride();
            farShooting();

            sendTelemetry();
        }
    }

    private void initHardware() {
        motorLeft  = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");

        intakeLeft  = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        intakeServo.setPosition(1.00);

        servoStopper = hardwareMap.get(ServoImplEx.class, "servoStopper");
        servoStopper.setPosition(0.53);

        // RoadRunner drive init
        drive = new MecanumDrive(hardwareMap, PoseStorage.lastPose);
        PoseStorage.lastPose = drive.localizer.getPose();

    }

    // ===================== DRIVING (through RR) =====================
    private void handleDriving() {
        double y = -gamepad1.left_stick_y; // forward
        double x = -gamepad1.left_stick_x; // strafe
        double r =  -gamepad1.right_stick_x; // turn

        // Optional: reduce strafe
        x *= 0.60;

        // If strafing, optionally reduce rotation to keep it clean
        //if (Math.abs(x) > 0.05) //r = 0;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), r));
    }

    // ===================== LOCK METHOD (FIXED) =====================
    // Call this EVERY LOOP while locking == true
    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = drive.localizer.getPose();

        // Field-space difference
        Vector2d diffField = new Vector2d(
                targetPos.position.x - currPos.position.x,
                targetPos.position.y - currPos.position.y
        );

        // Convert field error to robot frame so drive powers make sense
        Vector2d xyRobot = rotate(diffField, -currPos.heading.toDouble());

        // Heading error
        double headingErr = wrapAngle(targetPos.heading.toDouble() - currPos.heading.toDouble());

        // Command drive back toward the saved pose
        drive.setDrivePowers(
                new PoseVelocity2d(
                        xyRobot.times(xyP),
                        headingErr * headingP
                )
        );
    }

    // Hold XY at lockPose and face a point (for launching)
    private void lockXYAndFacePoint(Pose2d xyLockPose, Vector2d targetPoint) {
        Pose2d curr = drive.localizer.getPose();

        Vector2d diffField = new Vector2d(
                xyLockPose.position.x - curr.position.x,
                xyLockPose.position.y - curr.position.y
        );
        Vector2d xyRobot = rotate(diffField, -curr.heading.toDouble());

        Vector2d toTarget = new Vector2d(
                targetPoint.x - curr.position.x,
                targetPoint.y - curr.position.y
        );

        double targetHeading = Math.atan2(toTarget.y, toTarget.x);
        double headingErr = wrapAngle(targetHeading - curr.heading.toDouble());

        drive.setDrivePowers(
                new PoseVelocity2d(
                        xyRobot.times(LAUNCH_XY_P),
                        headingErr * LAUNCH_HEADING_P
                )
        );
    }

    // ===================== LAUNCHER =====================
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

    private void updateLauncherMotors() {
        motorLeft.setPower(launchPower);
        motorRight.setPower(launchPower);
    }

    // ===================== INTAKE =====================
    private void handleIntakeMotorsToggle() {
        if (gamepad1.x && !xPressedLast) {
            intakeMotorsOn = !intakeMotorsOn;
        }
        xPressedLast = gamepad1.x;

        if (!servoBusy) {
            intakeLeft.setPower(intakeMotorsOn ? 0.8 : 0.0);
            intakeRight.setPower(intakeMotorsOn ? 0.8 : 0.0);
        }
    }

    // ===================== LAUNCH SEQUENCE =====================
    private void handleLaunchSequence() {
        boolean b = gamepad1.b;

        if (b && !bPressedLast && !servoBusy) {
            servoBusy = true;

            // CAPTURE THE POSE ONCE HERE (this is what you will return to)
            lockPose = drive.localizer.getPose();

            // Choose launch target once
            //launchTarget = new Vector2d(-180, -40);

            // Start launch-align mode
            isLaunching = true;

            new Thread(() -> {
                try {
                    intakeRight.setPower(0.2);
                    intakeLeft.setPower(0.2);

                    servoStopper.setPosition(0.05);

                    intakeServo.setPosition(0.72);
                    sleepQuiet(500);

                    intakeServo.setPosition(0.50);
                    sleepQuiet(500);

                    intakeServo.setPosition(0.24);
                    sleepQuiet(600);

                    intakeServo.setPosition(1.00);
                    servoStopper.setPosition(0.53);
                } finally {
                    isLaunching = false;
                    servoBusy = false;
                }
            }).start();
        }

        bPressedLast = b;
    }

    // ===================== HELPERS =====================
    private static double wrapAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
    private void HumanPlayer(){
        boolean a = gamepad1.a;

        if (gamepad1.aWasPressed() && !gamepad1.dpad_down) {
            launchPower = -launchPower;
            if (servoStopper.getPosition() == 0.05) servoStopper.setPosition(0.53);
            else servoStopper.setPosition(0.05);
        }

//        if(gamepad1.aWasPressed() == true && !(launchPower % 2 == 1)&& !gamepad1.dpad_down) {
//            launchPower = -launchPower;
//        }
//
//        if(gamepad1.aWasPressed() == true && servoStopper.getPosition() == .05 && !gamepad1.dpad_down) {
//            //isLaunching = false;
//            servoStopper.setPosition(0.53);
//        }
//        if(gamepad1.aWasPressed() == true && !(launchPower % 2 == -1)&& !gamepad1.dpad_down) {
//            launchPower = -launchPower;
//        }
//
//        if(gamepad1.aWasPressed() == true && servoStopper.getPosition() == 0.53 && !gamepad1.dpad_down) {
//            servoStopper.setPosition(0.05);
//        }

    }

    private void manualOverride(){


        boolean up = gamepad1.dpad_up;


        if (up && !dpadLeftLast && !servoBusy) {
            servoBusy = true;

            new Thread(() -> {
                try {
                    lockPose = drive.localizer.getPose();
                    locking = true;
                    intakeRight.setPower(0.2);
                    intakeLeft.setPower(0.2);
                    servoStopper.setPosition(0.05);

                    int shoots = 0;
                    double[] shooting = {0.72, 0.50, 0.24};

                    while (opModeIsActive() && shoots < 3) {
                        if (gamepad1.dpad_up) {
                            intakeServo.setPosition(shooting[shoots]);
                            shoots++;
                            sleepQuiet(450);
                        }
                    }
                    intakeServo.setPosition(1.00);
                } finally {
                    locking = false;

                    isLaunching = false;
                    servoBusy = false;
                }
            }).start();
        }



    }

    private void farShooting() {
        boolean y = gamepad1.y;
        if(gamepad1.yWasPressed()) {
            lockPose = drive.localizer.getPose();
            locking = true;

            launchPower = 1;
            intakeRight.setPower(0.2);
            intakeLeft.setPower(0.2);
            servoStopper.setPosition(0.05);
            isLaunching = true;
            intakeServo.setPosition(0.72);
            sleepQuiet(750);
            //launchPower += 0.03;
            intakeServo.setPosition(0.50);
            //launchPower += 0.03;
            sleepQuiet(750);
            intakeServo.setPosition(0.24);
            sleepQuiet(600);
            intakeServo.setPosition(1.00);
            locking = false;
            //isLaunching = false;
        }
    }


    private static Vector2d rotate(Vector2d v, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return new Vector2d(
                v.x * c - v.y * s,
                v.x * s + v.y * c
        );
    }

    private void sleepQuiet(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    private void sendTelemetry() {
        telemetry.addData("LaunchPower", launchPower);
        telemetry.addData("ServoBusy", servoBusy);
        telemetry.addData("IntakeServoPos", intakeServo.getPosition());
        telemetry.addData("IntakeOn", intakeMotorsOn);
        telemetry.addData("Locking", locking);
        telemetry.addData("Launching", isLaunching);
        telemetry.addData("LockPose", lockPose);
        telemetry.addLine("|");
        telemetry.addLine("|");
        telemetry.addLine("|");
        telemetry.addLine("|");
        telemetry.addData("Pose", drive.localizer.getPose());
        telemetry.addData("LaunchTarget", launchTarget);
        telemetry.addData("StopperPos", servoStopper.getPosition());
        telemetry.update();
    }
}
