package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Auto Far", group = "Auto")
public class RedAutoFar extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;
    private Servo servoStopper;


    private double launchPower = 1.0;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        //motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        servoStopper = hardwareMap.get(Servo.class,"servoStopper");

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        //intakeRight.setDirection(DcMotor.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(61.5, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.3);
        intakeRight.setPower(0.3);
        servoStopper.setPosition(0.53);


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))

                            .build()
            );
        }
        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            launchPower = 1.0;
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(53,55),Math.toRadians(240))
                            .strafeTo(new Vector2d(53, 67))
                            .strafeTo(new Vector2d(53,55))
                            .strafeTo(new Vector2d(62, 67))
                            .strafeTo(new Vector2d(55,55))
                            .strafeTo(new Vector2d(68, 67))
                            .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .waitSeconds(5)
                            .strafeToLinearHeading(new Vector2d(60,45),Math.toRadians(-90))
                            .strafeTo(new Vector2d(60, 64))
                            .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))

                            .build()
            );
        }
        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(60,45),Math.toRadians(-90))
                            .strafeTo(new Vector2d(60, 64))

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

    private void fire3Rings() {
        if (!opModeIsActive()) return;

        motorLeft.setPower(-launchPower);
        motorRight.setPower(-launchPower);
        intakeRight.setPower(0.2);
        intakeLeft.setPower(0.2);
        servoStopper.setPosition(0.05);
        sleepQuiet(1500);
        intakeServo.setPosition(0.72);
        sleepQuiet(1000);
        intakeServo.setPosition(0.50);
        sleepQuiet(1000);
        intakeServo.setPosition(0.24);
        sleepQuiet(1500);
        intakeServo.setPosition(1.00);
        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);
        servoStopper.setPosition(0.4);

        //motorLeft.setPower(0);
        //motorRight.setPower(0);

    }
}
