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


import java.util.Vector;


@Autonomous(name = "Blue Auto Far", group = "Auto")
public class BlueAutoFar extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private Servo servoStopper;
    private double launchPower = .85;
    private double servoPosUp = 0.12;

    private double servoPosDown = 0.53;
    private Thread poseThread;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        //motorRight.setDirection(DcMotor.Direction.REVERSE);
        //motorRight.setDirection(DcMotor.Direction.REVERSE);
        //motorLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
        servoStopper = hardwareMap.get(Servo.class, "servoStopper");

        Pose2d beginPose = new Pose2d(61.5, -15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.8);
        intakeRight.setPower(0.8);
        servoStopper.setPosition(servoPosDown);
        motorLeft.setPower(-launchPower);
        motorRight.setPower(-launchPower);

        startPoseTracking(drive);

        // (Optional) initialize lastPose immediately
        PoseStorage.lastPose = drive.localizer.getPose();


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.setReversed(true)
                            .strafeTo(new Vector2d(56, -12))
                            .turn(Math.toRadians(18))

                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();
        if (opModeIsActive()) {
            //drive.defaultVelConstraint.equals(20);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //turns to the point where it starts spline
                            .setReversed(true)
                            .strafeToLinearHeading(new Vector2d(57, -45), Math.toRadians(110))

                            //intake method
                            .strafeTo(new Vector2d(51,-25))
                            .strafeTo(new Vector2d(60,-55))
                            .strafeTo(new Vector2d(60,-25))

                            //.strafeTo(new Vector2d(40,-50))

                            .setReversed(false)

                            //.splineTo(new Vector2d(65, -50), Math.toRadians(180))

                            //comes back to shoot brrrrr
                            .strafeToLinearHeading(new Vector2d(53, -8), Math.toRadians(180))
                            .turn(Math.toRadians(20))




                            .build()
            );

        }



        if (opModeIsActive()) fire2Rings();


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.setReversed(true)
                            .strafeTo(new Vector2d(55, -25))

                            .build()

            );
        }


        if (opModeIsActive()) {
            //drive.defaultVelConstraint.equals(20);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //turns to the point where it starts spline
                            //.strafeTo(new Vector2d(,-5))
                            .strafeToLinearHeading(new Vector2d(55,-25),Math.toRadians(90))
                            .waitSeconds(2)
                            //.splineTo(new Vector2d(60, -65), Math.toRadians(130))
                            .strafeTo(new Vector2d(56,-50))
                            .strafeTo(new Vector2d(56,-25))
                            .strafeTo(new Vector2d(48,-50))



                            //comes back to shoot brrrrr

                            .strafeToLinearHeading(new Vector2d(47, -12), Math.toRadians(180))

                            .turn(Math.toRadians(18))




                            .build()
            );

        }



        if (opModeIsActive()) fire3Rings();
        if (opModeIsActive()) {
            //drive.defaultVelConstraint.equals(20);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //turns to the point where it starts spline
                            .strafeTo(new Vector2d(56,-25))





                            .build()
            );

        }
    }



    private void sleepQuiet(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
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





    private void fire3Rings() {
        if (!opModeIsActive()) return;
        intakeRight.setPower(0.2);
        intakeLeft.setPower(0.2);
        servoStopper.setPosition(servoPosUp);
        sleepQuiet(200);
        intakeServo.setPosition(0.72);
        sleepQuiet(1400);
        intakeServo.setPosition(0.50);
        //launchPower -= 0.05;
        sleepQuiet(1400);

        intakeServo.setPosition(0.24);
        sleepQuiet(200);
        //intakeServo.setPosition(1.00);
        delayedServoReset();
        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);

        //servoStopper.setPosition(0.4);

        //motorLeft.setPower(0);
        //motorRight.setPower(0);

    }

    private void fire2Rings() {
        if (!opModeIsActive()) return;
        intakeRight.setPower(0.2);
        intakeLeft.setPower(0.2);
        servoStopper.setPosition(servoPosUp);
        sleepQuiet(1400);
        intakeServo.setPosition(0.50);
        //launchPower -= 0.05;
        sleepQuiet(1400);

        intakeServo.setPosition(0.24);
        sleepQuiet(600);
        //intakeServo.setPosition(1.00);
        delayedServoReset();
        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);

        //servoStopper.setPosition(0.4);

        //motorLeft.setPower(0);
        //motorRight.setPower(0);

    }
}
