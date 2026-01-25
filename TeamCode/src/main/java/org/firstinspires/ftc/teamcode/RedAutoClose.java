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

@Autonomous(name = "Red Auto Close", group = "Auto")
public class RedAutoClose extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private ServoImplEx servoStopper;


    private double launchPower = 0.52;

    private volatile boolean isLaunching = false;




    ;

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
        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
        servoStopper = hardwareMap.get(ServoImplEx.class, "servoStopper");


        Pose2d beginPose = new Pose2d(-50, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.8);
        intakeRight.setPower(0.8);
        motorLeft.setPower(launchPower);
        motorRight.setPower(launchPower);
        servoStopper.setPosition(0.53);


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
                            .setReversed(true)
                            .strafeToLinearHeading(new Vector2d(13, 26),Math.toRadians(-90))
                            .strafeTo(new Vector2d(13, 62))
                            .setReversed(true)

                            .setReversed(false)

                            .strafeToLinearHeading(new Vector2d(13,41),Math.toRadians(180))

                            .strafeTo(new Vector2d(3,58))
                            .strafeTo(new Vector2d(0,0))
                            .strafeToLinearHeading(new Vector2d(-11,3),Math.toRadians(130))
                            .build()
            );
        }

        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(-11,26),Math.toRadians(-90))
                            .strafeTo(new Vector2d(-11,56))
                            .strafeToLinearHeading(new Vector2d(-11,3),Math.toRadians(130))
                            .build()
            );
        }
        if (opModeIsActive()) fire3Rings();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(36,26),Math.toRadians(-90))
                            .strafeTo(new Vector2d(36,62))

                            .strafeToLinearHeading(new Vector2d(-11,3),Math.toRadians(130))
                            .build()
            );
        }
        if (opModeIsActive()) fire3Rings();


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(5,15))
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

        isLaunching = true;

        //servoStopper.setPosition(0.1);


        motorLeft.setPower(launchPower);
        motorRight.setPower(launchPower);
        intakeRight.setPower(0.2);
        intakeLeft.setPower(0.2);
        servoStopper.setPosition(0.4);
        while (opModeIsActive() && motorLeft.getVelocity() < 1150) idle();
        intakeServo.setPosition(0.72);
        sleepQuiet(450);
        while (opModeIsActive() && motorLeft.getVelocity() < 1150) idle();
        intakeServo.setPosition(0.5);
        sleepQuiet(450);
        while (opModeIsActive() && motorLeft.getVelocity() < 1150) idle();
        intakeServo.setPosition(0.24);
        sleepQuiet(250);
        intakeServo.setPosition(1.00);

                    /*while (opModeIsActive()) {
                        if(motorLeft.getVelocity() >= 1600){
                            intakeServo.setPosition(0.72) ;
                            sleepQuiet(500);
                        }
                        else{
                            idle();
                        }

                        if(motorLeft.getVelocity() >= 1600){
                            intakeServo.setPosition(0.5) ;
                            sleepQuiet(500);
                        }
                        else{
                            idle();
                        }

                        if(motorLeft.getVelocity() >= 1600){
                            intakeServo.setPosition(0.25) ;
                            sleepQuiet(500);
                        }
                        else{
                            idle();
                        }*/





        //launcherOn = false;


        isLaunching = false;
        servoStopper.setPosition(0.53);
        intakeRight.setPower(0.8);
        intakeLeft.setPower(0.8);
        motorLeft.setPower(launchPower);
        motorRight.setPower(launchPower);


    }
}
