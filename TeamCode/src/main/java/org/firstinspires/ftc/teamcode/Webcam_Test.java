package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Webcam_Test", group = "TeleOp")
public class Webcam_Test extends LinearOpMode {

    // Launcher + intake hardware
    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private ServoImplEx intakeServo;

    // State flags
    private boolean launcherOn = false;
    private boolean intakeOn = false;
    private boolean servoBusy = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;

    private double launchPower = 0.69;
    private long lastAdjustTime = 0;
    private double lastError = 0;

    // Road Runner drive
    private MecanumDrive rrDrive;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Targeting
    private static final double TARGET_X = 48;
    private static final double TARGET_Y = -72;
    private static final int TAG_ID_24 = 24;

    // Variable to show successful scan of tag 24
    private boolean tag24Seen = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------------------------------------------
        // HARDWARE INIT
        // -------------------------------------------------------
        rrDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        motorRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        intakeRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        intakeServo.setPosition(1.0);

        // -------------------------------------------------------
        // APRILTAG VISION SETUP (standard black/white tag, ID 24)
        // -------------------------------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        // -------------------------------------------------------
        // MAIN TELEOP LOOP
        // -------------------------------------------------------
        while (opModeIsActive()) {

            // Update RR pose
            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.localizer.getPose();

            // -------------------------
            // MANUAL DRIVING INPUT
            // -------------------------
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x;   // FIX: invert turning direction

            // -------------------------
            // AUTO AIM on Y button
            // -------------------------
            if (gamepad1.y) {
                double dx = TARGET_X - pose.position.x;
                double dy = TARGET_Y - pose.position.y;

                double desiredHeading = Math.atan2(dy, dx);
                double error = normalize(desiredHeading - pose.heading.toDouble());
                double derivative = (error - lastError) / 0.02; // assuming 20ms loop
                lastError = error;

                double turnCmd = 2.0 * error + 0.25 * derivative;
                turnCmd = Math.max(-0.6, Math.min(0.6, turnCmd));

                turn = turnCmd; // override manual turn while Y is held
            }

            // -------------------------
            // APPLY DRIVE POWER (RR MecanumDrive method)
            // -------------------------
            rrDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(forward, strafe),
                            turn
                    )
            );

            // -------------------------
            // LAUNCHER POWER ADJUST
            // -------------------------
            long now = System.currentTimeMillis();
            if (now - lastAdjustTime >= 120) {
                if (gamepad1.right_bumper) {
                    launchPower = Math.min(1.0, launchPower + 0.02);
                    lastAdjustTime = now;
                }
                if (gamepad1.left_bumper) {
                    launchPower = Math.max(0.0, launchPower - 0.02);
                    lastAdjustTime = now;
                }
            }

            // -------------------------
            // INTAKE TOGGLE (X)
            // -------------------------
            if (gamepad1.x && !xPressedLast) {
                intakeOn = !intakeOn;
            }
            xPressedLast = gamepad1.x;

            intakeLeft.setPower(intakeOn ? 0.3 : 0.0);
            intakeRight.setPower(intakeOn ? 0.3 : 0.0);

            // -------------------------
            // LAUNCH MACRO (B)
            // -------------------------
            if (gamepad1.b && !bPressedLast && !servoBusy) {
                servoBusy = true;

                new Thread(() -> {
                    try {
                        launcherOn = true;
                        sleepQuiet(900);

                        intakeServo.setPosition(0.82);
                        sleepQuiet(300);

                        intakeServo.setPosition(0.51);
                        sleepQuiet(300);

                        intakeServo.setPosition(0.25);
                        sleepQuiet(200);

                        intakeServo.setPosition(1.00);
                        launcherOn = false;
                    } finally {
                        servoBusy = false;
                    }
                }).start();
            }
            bPressedLast = gamepad1.b;

            motorLeft.setPower(launcherOn ? -launchPower : 0);
            motorRight.setPower(launcherOn ? -launchPower : 0);

            // -------------------------
            // APRILTAG ID 24 DETECTION
            // -------------------------
            tag24Seen = false;
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == TAG_ID_24) {
                    tag24Seen = true;
                    break;
                }
            }

            // -------------------------
            // TELEMETRY
            // -------------------------
            telemetry.addData("Pose X", pose.position.x);
            telemetry.addData("Pose Y", pose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Launcher Power", launchPower);
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Auto Aim Active", gamepad1.y);
            telemetry.addData("Tag 24 Seen", tag24Seen);
            telemetry.update();
        }

        visionPortal.close();
    }

    // -------------------------
    // HELPERS
    // -------------------------
    private double normalize(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception ignored) {
        }
    }
}
