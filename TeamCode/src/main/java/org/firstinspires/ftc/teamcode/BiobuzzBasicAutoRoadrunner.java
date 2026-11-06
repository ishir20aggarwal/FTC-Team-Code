package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BiobuzzBasicAuto", group = "Auto")
public class BiobuzzBasicAutoRoadrunner extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private ServoImplEx servoLeft, servoRight;
    private ElapsedTime autoTimer = new ElapsedTime();

    //public static volatile Pose2d lastPose = new Pose2d(0, 0, 0); // volatile so other threads read latest
    private double launchPower = 1200;

    private final double pollenServoPos = 0.99;
    private final double nectarServoPos = 0.82;

    // ===== Pose tracker thread control =====
    private Thread poseThread;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);



        intake = hardwareMap.get(DcMotorEx.class, "intake");
        servoLeft = hardwareMap.get(ServoImplEx.class, "servoLeft");
        servoRight = hardwareMap.get(ServoImplEx.class, "servoRight");
        servoRight.setDirection(ServoImplEx.Direction.REVERSE);


        // Default starting positions
        servoLeft.setPosition(0.5);
        servoRight.setPosition(0.5);


        Pose2d beginPose = new Pose2d(-50, -50, Math.toRadians(-130));
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

        intake.setPower(0.8);
        flywheel.setPower(launchPower);
        servoLeft.setPosition(pollenServoPos);
        servoRight.setPosition(pollenServoPos);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)


                            .strafeToLinearHeading(new Vector2d(23,-46),Math.toRadians(-90))
                            .strafeTo(new Vector2d(23,-59))
                            .build()
            );
        }

        sleepQuiet(1000);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(23,-57))
                            .strafeToLinearHeading(new Vector2d(-36,-36),Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(-52,-18),Math.toRadians(0))
                            .build()
            );
        }

        launchPower = 1200;

        lebron();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turnTo(Math.toRadians(180),new TurnConstraints(10,-2,2))
                            .strafeToConstantHeading(new Vector2d(-59,-24))
                            .build()
            );
        }

        launchPower = 1200;

        sleepQuiet(1000);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(-50,-24),Math.toRadians(180))
                            .strafeToLinearHeading(new Vector2d(-52,-18),Math.toRadians(0))
                            .build()
            );
        }

        launchPower = 1200;

        lebron();
         if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(-36,-36),Math.toRadians(0))
                            .strafeToConstantHeading(new Vector2d(55,-61))
                            .build()
            );
        }
         sleepQuiet(500);
        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToConstantHeading(new Vector2d(62,-61))
                            .strafeToConstantHeading(new Vector2d(55,-61))
                            .strafeToLinearHeading(new Vector2d(56,-16),Math.toRadians(180))
                            .build()
            );
        }
        lebron();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToConstantHeading(new Vector2d(20,-50))
                            .strafeToConstantHeading(new Vector2d(-36,-60))
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



    private void lebron() {
        if (!opModeIsActive()) return;

        intake.setPower(0.2);
        launchPower -= 0.1;
        sleepQuiet(100);


        sleepQuiet(100);
        launchPower += 0.1;

        sleepQuiet(100);

        sleepQuiet(100);

        intake.setPower(0.8);
    }

    private void sleepQuiet(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
    }
}
