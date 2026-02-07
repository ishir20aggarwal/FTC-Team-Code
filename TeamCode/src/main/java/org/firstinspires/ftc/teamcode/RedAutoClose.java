package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Auto Close", group = "Auto")
public class RedAutoClose extends LinearOpMode {
    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;
    private ServoImplEx servoStopper;
    private ElapsedTime autoTimer = new ElapsedTime();

    //public static volatile Pose2d lastPose = new Pose2d(0, 0, 0); // volatile so other threads read latest
    private double launchPower = 0.68;
    private double servoPosUp = 0.12;

    private double servoPosDown = 0.53;

    // ===== Pose tracker thread control =====
    private Thread poseThread;

    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

        servoStopper = hardwareMap.get(ServoImplEx.class, "servoStopper");
        servoStopper.setPosition(servoPosDown);

        Pose2d beginPose = new Pose2d(-50, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        autoTimer.reset();

        if (!opModeIsActive()) return;

        // Start continuous pose tracking AFTER start
        startPoseTracking(drive);

        // (Optional) initialize lastPose immediately
        PoseStorage.lastPose = drive.localizer.getPose();

        intakeLeft.setPower(0.8);
        intakeRight.setPower(0.8);
        motorLeft.setPower(launchPower);
        motorRight.setPower(launchPower);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)


                            .setReversed(true)
                            .lineToX(-11)
                            .build()
            );
        }

        lebron();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .setReversed(true)
                            .strafeToLinearHeading(new Vector2d(10, 20), Math.toRadians(-90))
                            .strafeTo(new Vector2d(10, 55))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(2, 36), Math.toRadians(180))
                            .strafeTo(new Vector2d(2, 46))
                            .waitSeconds(0.50)
                            .strafeTo(new Vector2d(0, 30))
                            .strafeToLinearHeading(new Vector2d(-15, 20), Math.toRadians(135))

                            .build()
            );
        }

        launchPower = .64;

        lebron();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(-12, 26), Math.toRadians(-90))
                            .setReversed(true)
                            .strafeTo(new Vector2d(-12, 60))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-20, 35), Math.toRadians(140))

                            .build()
            );
        }

        launchPower = .64;

        lebron();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(27, 30), Math.toRadians(-90))
                            .setReversed(true)
                            .strafeTo(new Vector2d(27, 80))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-30, 35), Math.toRadians(120))

                            .build()
            );
        }

        launchPower=.64;

        lebron();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            //.strafeTo(new Vector2d(-15, -40))
                            //.strafeTo(new Vector2d(0,0))
                            .build()
            );
        }

        // Stop thread cleanly at the end
        stopPoseTracking();
    }

    private void startPoseTracking(MecanumDrive drive) {
        poseThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested() && !Thread.currentThread().isInterrupted()) {
                // Keep lastPose always fresh
                PoseStorage.lastPose = drive.localizer.getPose();

                // (Optional) live telemetry
                telemetry.addData("x", PoseStorage.lastPose.position.x);
                telemetry.addData("y", PoseStorage.lastPose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(PoseStorage.lastPose.heading.toDouble()));
                telemetry.addData("Runtime", getRuntime());
                telemetry.update();

                try {
                    Thread.sleep(20); // 50 Hz updates; adjust 10-30ms as desired
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        poseThread.setName("PoseTracker");
        poseThread.start();
    }

    private void stopPoseTracking() {
        if (poseThread != null) {
            poseThread.interrupt();
            poseThread = null;
        }
    }

    private void delayedServoReset() {
        new Thread(() -> {
            try {
                Thread.sleep(500);

                if (opModeIsActive()) {
                    intakeServo.setPosition(1.00);
                    servoStopper.setPosition(servoPosDown);
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    private void lebron() {
        if (!opModeIsActive()) return;

        intakeRight.setPower(0.2);
        intakeLeft.setPower(0.2);
        servoStopper.setPosition(servoPosUp);
        launchPower -= 0.1;
        sleepQuiet(100);


        intakeServo.setPosition(0.72);
        sleepQuiet(500);
        launchPower += 0.1;

        intakeServo.setPosition(0.50);
        sleepQuiet(500);

        intakeServo.setPosition(0.24);
        sleepQuiet(200);
        delayedServoReset();

        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);
    }

    private void sleepQuiet(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
    }
}
