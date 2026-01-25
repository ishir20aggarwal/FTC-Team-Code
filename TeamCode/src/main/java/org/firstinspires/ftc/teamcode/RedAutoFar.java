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

    private double launchPower = 0.69;

    @Override
    public void runOpMode() {

        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        //motorRight.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(1.0);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        //intakeRight.setDirection(DcMotor.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(61.5, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.3);
        intakeRight.setPower(0.3);

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(56,12))
                            .turn(Math.toRadians(-21))
                            .build()
            );
        }
        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            launchPower = .7;
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            //.strafeTo(new Vector2d(60,40))
                            .turn(Math.toRadians(120))
                            .strafeTo(new Vector2d(35,25))
                            .waitSeconds(0.5)
                            .strafeTo(new Vector2d(35,65))
                            .strafeTo(new Vector2d(56,12))
                            .turn(Math.toRadians(-122))
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(15))

                            /*.strafeTo(new Vector2d(40,68))
                            .strafeTo(new Vector2d(68,68))*/
                            .strafeTo(new Vector2d(56,30))
                            //.turn(Math.toRadians(-90))
                            //.strafeTo(new Vector2d(-2,50))
                            //make this aligning
                            .turn(Math.toRadians(23))
                            .strafeTo(new Vector2d(57,-55))
                            .strafeTo(new Vector2d(57,-67))

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

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }
}
