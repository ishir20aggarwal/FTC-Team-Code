package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
//@Autonomous(name = "Blue Auto Far Leave", group = "Auto")
public class BlueAutoFarLeave extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private double launchPower = 0.67;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
//        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
//        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(-50, -50, Math.toRadians(-130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.8);
        intakeRight.setPower(0.8);

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
                            .turn(Math.toRadians(-140))
                            .strafeTo(new Vector2d(-14,-55))
                            .lineToY(-55)
                            .setReversed(false)
                            .lineToY(0)
                            .turn(Math.toRadians(145))
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(-141))
                            .setReversed(true)
                            .strafeTo(new Vector2d(12, -20))
                            .strafeTo(new Vector2d(12, -60))
                            .setReversed(true)
                            .lineToY(-52)
                            .setReversed(false)
                            .lineToY(-20)
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
        sleepQuiet(1000);
        intakeServo.setPosition(0.72);
        sleepQuiet(1000);
        intakeServo.setPosition(0.50);
        sleepQuiet(1000);
        intakeServo.setPosition(0.24);
        sleepQuiet(1500);
        intakeServo.setPosition(1.00);
        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }
}
//test
