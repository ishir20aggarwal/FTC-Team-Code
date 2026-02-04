package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled

@TeleOp(name="Tuning Opmode", group="TeleOp")
public class tankukanaforloking extends LinearOpMode {

    private DcMotorEx topLeft, bottomLeft, topRight, bottomRight;
    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private ServoImplEx intakeServo;

    private volatile boolean isLaunching = false;


    // ================= PARK LOCK =================
    private boolean parkLocked = false;
    private boolean dpadDownLast = false;
    private Pose2d lockPose = new Pose2d(0, 0, 0);

    private boolean autoDriving = false;
    private Action autoAction = null;

    // choose your target point (in RR field units)
    private final Vector2d targetBVector = new Vector2d(38, 34);
    private final Vector2d targetAVector = new Vector2d(38, -34);



    // Tune these
    private static final double LOCK_KP_XY = 2.0;
    private static final double LOCK_KP_HEADING = 4.0;
    private MecanumDrive drive;

    // Limits
    private static final double LOCK_MAX_VEL = 0.8;
    private static final double LOCK_MAX_OMEGA = 1.5;

    private boolean servoBusy = false;
    private boolean dPressedLast = false;
    private boolean cPressedLast = false;
    private boolean bPressedLast = false;

    private static double wrapAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a)); // [-pi, pi]
    }

    private static Vector2d rotate(Vector2d v, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return new Vector2d(
                v.x * c - v.y * s,
                v.x * s + v.y * c
        );
    }


    private boolean launcherOn = false;
    private double launchPower = 0.65;
    private boolean locking = false;


    private final double POWER_STEP = 0.02;
    private final long ADJUST_DELAY_MS = 120;
    private long lastAdjustTime = 0;

    private boolean intakeMotorsOn = false;
    private boolean xPressedLast = false;

    private boolean dpadLeftLast = false;


    double xyP = 0.1;
    double headingP = 1;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            // REQUIRED: keep RoadRunner odometry updated every loop
                /*drive.updatePoseEstimate();
                boolean c = gamepad1.dpad_down;
                if (c && !cPressedLast && !autoDriving) {
                    Pose2d startPose = drive.localizer.getPose();

                    autoAction = drive.actionBuilder(startPose)
                            //.lineToX(targetBVector.x)      // or .lineTo(new Vector2d(...)) depending on your RR version
                            //.lineToY(targetBVector.y)
                            .strafeTo(targetBVector)
                            .build();

                    autoDriving = true;
                }
                cPressedLast = c;
                boolean d = gamepad1.dpad_down;
                if (d && !dPressedLast && !autoDriving) {
                    Pose2d startPose = drive.localizer.getPose();

                    autoAction = drive.actionBuilder(startPose)
                            //.lineToX(targetBVector.x)      // or .lineTo(new Vector2d(...)) depending on your RR version
                            //.lineToY(targetBVector.y)
                            .strafeTo(targetAVector)
                            .build();

                    autoDriving = true;
                }
                dPressedLast = d;*/


            //handleLockToggle();

            /*if (parkLocked) {
                lockWheels();
            } else {

            }*/
            // Update pose every loop (REQUIRED for lock)
            drive.updatePoseEstimate();

            // Toggle lock with dpad_left (edge detect)
            boolean dpadLeft = gamepad1.dpad_left;
            if (dpadLeft && !dpadLeftLast) {
                if (!locking) {
                    lockPose = drive.localizer.getPose();
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
                }
                locking = !locking;

            }
            dpadLeftLast = dpadLeft;

            // Only ONE of these runs each loop (no fighting)
            if (locking) {
                lockTo(lockPose);
            } else {
                handleDriving();
            }


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

        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        intakeServo.setPosition(1.00);

        intakeLeft  = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

        // REQUIRED: initialize RoadRunner drive so lock can command motors + read odometry
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    // Current pose helper (use anywhere)
    private Pose2d getCurrentPose() {
        return drive.localizer.getPose();
    }

    /*private void handleLockToggle() {
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadDown && !dpadDownLast) {
            parkLocked = !parkLocked;

            if (parkLocked) {
                // store CURRENT pose (not 0)
                lockPose = getCurrentPose();
                // stop motion immediately
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        }

        dpadDownLast = dpadDown;
    }

    private void lockWheels() {
        Pose2d cur = getCurrentPose();

        double ex = lockPose.position.x - cur.position.x;
        double ey = lockPose.position.y - cur.position.y;

        double ehe = lockPose.heading.toDouble() - cur.heading.toDouble();
        ehe = Math.atan2(Math.sin(ehe), Math.cos(ehe)); // wrap [-pi, pi]

        double vxField = LOCK_KP_XY * ex;
        double vyField = LOCK_KP_XY * ey;
        double omega   = LOCK_KP_HEADING * ehe;

        vxField = clamp(vxField, -LOCK_MAX_VEL, LOCK_MAX_VEL);
        vyField = clamp(vyField, -LOCK_MAX_VEL, LOCK_MAX_VEL);
        omega   = clamp(omega,   -LOCK_MAX_OMEGA, LOCK_MAX_OMEGA);

        double h = cur.heading.toDouble();
        double vxRobot =  vxField * Math.cos(h) + vyField * Math.sin(h);
        double vyRobot = -vxField * Math.sin(h) + vyField * Math.cos(h);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(vxRobot, vyRobot),
                        omega
                )
        );
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }*/

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

    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = drive.localizer.getPose(); // <- your equivalent of getPoseEstimate()

        // same idea as targetPos.minus(currPos) but using your Pose2d structure
        Vector2d diffField = new Vector2d(
                targetPos.position.x - currPos.position.x,
                targetPos.position.y - currPos.position.y
        );

        // same as difference.vec().rotated(-currHeading)
        Vector2d xy = rotate(diffField, -currPos.heading.toDouble());

        // same as Angle.normDelta(target - curr)
        double headingErr = wrapAngle(targetPos.heading.toDouble() - currPos.heading.toDouble());

        // same “weighted drive power” concept, but your drive uses setDrivePowers(PoseVelocity2d)
        drive.setDrivePowers(
                new PoseVelocity2d(
                        xy.times(xyP),
                        headingErr * headingP
                )
        );
    }


    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustTime < ADJUST_DELAY_MS) return;

        if (gamepad1.right_bumper) {
            motorLeft.setPower(motorRight.getPower());
            launchPower = Math.min(1.0, launchPower + POWER_STEP);
            lastAdjustTime = now;
        }
        if (gamepad1.left_bumper) {
            motorLeft.setPower(motorRight.getPower());
            launchPower = Math.max(0.0, launchPower - POWER_STEP);
            lastAdjustTime = now;
        }
        if (gamepad1.dpad_down){
            motorLeft.setPower(0.72);
            motorRight.setPower(0.64);
        }
    }

    private void launch() {
        double p = launcherOn? -launchPower: 0;
        motorLeft.setPower(p);
        motorRight.setPower(p);
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
                    lockPose = drive.localizer.getPose();
                    locking = true;

                    lockTo(lockPose);
                    isLaunching = true;

                    launcherOn = true;
                    intakeRight.setPower(0.2);
                    intakeLeft.setPower(0.2);
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.72);
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.5);
                    sleepQuiet(1000);
                    intakeServo.setPosition(0.24);
                    sleepQuiet(750);
                    intakeServo.setPosition(1.00);
                    locking = false;




                    launcherOn = false;
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
        telemetry.addData("outtake", motorLeft.getPower());
        telemetry.addData("intake", intakeLeft.getPower());

        telemetry.update();
    }
}
