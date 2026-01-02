package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Auto Close", group = "Auto")
public class RedAutoClose extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private double launchPower = 0.61;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(-50, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.3);
        intakeRight.setPower(0.3);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .setReversed(true)
                            .lineToX(-11)
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(140))
                            .lineToY(55)
                            .setReversed(false)
                            .lineToY(0)
                            .turn(Math.toRadians(-140))
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(140))
                            .setReversed(true)
                            .strafeTo(new Vector2d(13, 20))
                            .strafeTo(new Vector2d(13, 60))
                            .setReversed(true)
                            .lineToY(50)
                            .setReversed(false)
                            .lineToY(20)
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

        sleepQuiet(1000);
        intakeServo.setPosition(0.82);
        sleepQuiet(1000);
        intakeServo.setPosition(0.51);
        sleepQuiet(1000);
        intakeServo.setPosition(0.25);
        sleepQuiet(200);
        intakeServo.setPosition(1.00);

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
