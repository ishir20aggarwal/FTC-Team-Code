package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

// Change this import to YOUR drive class (often MecanumDrive in RR 1.0 quickstart)
// import org.firstinspires.ftc.teamcode.MecanumDrive;




@Autonomous(name = "Blue Auto Far DONT", group = "Auto")
public class BlueAutoFarTestAlign extends LinearOpMode {

    private DcMotorEx motorLeft, motorRight;
    private DcMotorEx intakeLeft, intakeRight;
    private Servo intakeServo;

    private double launchPower = 0.68;
    public static double ALIGN_kP = 4.0;
    public static double MAX_OMEGA = 6.0;
    private boolean alignActive = false;
    private Vector2d alignTarget = null;



    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void alignStart(Vector2d target) {
        alignTarget = target;
        alignActive = true;
    }

    public void alignStop(MecanumDrive drive) {
        alignActive = false;

        // Immediately stop rotation
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0, 0), 0.0)
        );
    }

    public void alignUpdate(MecanumDrive drive) {
        if (!alignActive || alignTarget == null) return;

        Pose2d pose = drive.localizer.getPose();

        double dx = alignTarget.x - pose.position.x;
        double dy = alignTarget.y - pose.position.y;

        double desiredHeading = Math.atan2(dy, dx);
        double headingError =
                normalizeAngle(desiredHeading - pose.heading.toDouble());

        double omega = ALIGN_kP * headingError;
        omega = clamp(omega, -MAX_OMEGA, MAX_OMEGA);

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0.0, 0.0), omega)
        );
    }





    public void alignToPoint(MecanumDrive drive, Vector2d target) {
        Pose2d pose = drive.localizer.getPose();

        double dx = target.x - pose.position.x;
        double dy = target.y - pose.position.y;

        double desiredHeading = Math.atan2(dy, dx);
        double headingError = normalizeAngle(desiredHeading - pose.heading.toDouble());

        double omega = ALIGN_kP * headingError;
        omega = clamp(omega, -MAX_OMEGA, MAX_OMEGA);

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0.0, 0.0), omega)
        );
    }



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

        Pose2d beginPose = new Pose2d(61.5, -15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (!opModeIsActive()) return;

        intakeLeft.setPower(0.3);
        intakeRight.setPower(0.3);
        //alignFor(drive,new Vector2d(-65,-56),800,10);
        //alignToPoint(drive,new Vector2d(-65,-56));


        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.setReversed(true)
                            .strafeTo(new Vector2d(56,-12))
                            //.turn(Math.toRadians(24))
                            .build()
            );
        }

        if (opModeIsActive()){
            alignFor(drive,new Vector2d(-65,-56),800,1);

            fire3Rings();}

        if (opModeIsActive()) {
            launchPower = 0.68;
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            //.strafeTo(new Vector2d(60,-40))
                            //.turn(Math.toRadians(-119))
                            .strafeTo(new Vector2d(35,-25))
                            //.waitSeconds(0.5d)
                            //.strafeTo(new Vector2d(35,-65))
                            .strafeTo(new Vector2d(56,-12))
                           // .turn(Math.toRadians(123))
                            .build()
            );
        }

        if (opModeIsActive()){
            alignAndBackUp(drive,new Vector2d(35,25),0.4,2000,1);

            fire3Rings();}

        if (opModeIsActive()){
            alignFor(drive,new Vector2d(-65,-56),800,10);

            fire3Rings();}

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(80))
                            .strafeTo(new Vector2d(56,-30))
                            //.strafeTo(new Vector2d(-1, -54))
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

    public void alignFor(MecanumDrive drive, Vector2d target, long timeoutMs, double stopRad) {
        alignStart(target);

        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - t0) < timeoutMs) {
            drive.updatePoseEstimate();   // keep pose fresh

            // compute error the same way you do in alignUpdate
            Pose2d pose = drive.localizer.getPose();
            double dx = target.x - pose.position.x;
            double dy = target.y - pose.position.y;
            double desiredHeading = Math.atan2(dy, dx);
            double headingError = normalizeAngle(desiredHeading - pose.heading.toDouble());

            // if close enough, stop early
            if (Math.abs(headingError) < stopRad) break;

            // apply your rotation command
            alignUpdate(drive);

            idle();
        }

        alignStop(drive);
    }

    public void alignAndBackUp(MecanumDrive drive, Vector2d target, double backPower,
                               long timeoutMs, double stopRad) {
        long t0 = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - t0) < timeoutMs) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double dx = target.x - pose.position.x;
            double dy = target.y - pose.position.y;

            double desiredHeading = Math.atan2(dy, dx);
            double headingError = normalizeAngle(desiredHeading - pose.heading.toDouble());

            double omega = clamp(ALIGN_kP * headingError, -MAX_OMEGA, MAX_OMEGA);

            // Stop early if we're basically straight
            if (Math.abs(headingError) < stopRad) omega = 0.0;

            // IMPORTANT: move backwards while keeping heading corrected
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(-Math.abs(backPower), 0.0), omega)
            );

            idle();
        }

        // stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
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
