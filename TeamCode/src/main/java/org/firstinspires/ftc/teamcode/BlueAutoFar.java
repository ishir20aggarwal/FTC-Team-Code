package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import java.util.Vector;


@Autonomous(name = "Blue Auto Far", group = "Auto")
public class BlueAutoFar extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private Servo servoStopper;
    private double launchPower = .95;

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
        servoStopper.setPosition(0.53);


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.setReversed(true)
                            .strafeTo(new Vector2d(56, -12))
                            .turn(Math.toRadians(20))

                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();
        if (opModeIsActive()) {
            //drive.defaultVelConstraint.equals(20);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //darin test
                            //.turn(-24)
                            //.strafeTo(new Vector2d(46, -3))
                            .setReversed(true)
                            .strafeToLinearHeading(new Vector2d(47, -43), Math.toRadians(90))



                            .build()
            );

        }
        if (opModeIsActive()) {
            drive.defaultVelConstraint.equals(20);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //darin test
                            //.turn(-24)
                            //.strafeTo(new Vector2d(46, -3))


                            .splineTo(new Vector2d(65, -62), Math.toRadians(190))

                            .build()
            );

        }
        if (opModeIsActive()) {
            drive.defaultVelConstraint.equals(60);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
//

                            //darin test
                            //.turn(-24)
                            //.strafeTo(new Vector2d(46, -3))


                            .strafeToLinearHeading(new Vector2d(60, -5), Math.toRadians(180))
                            .turn(Math.toRadians(20))


                            .build()
            );

        }


        if (opModeIsActive()) fire3Rings();


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.setReversed(true)
                            .strafeTo(new Vector2d(55, -25))

                            .build()

            );
        }


        if (opModeIsActive()) sleepQuiet(5000);
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
        sleepQuiet(1000);
        intakeServo.setPosition(0.72);
        sleepQuiet(1000);
        intakeServo.setPosition(0.50);
        launchPower -= 0.05;
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
