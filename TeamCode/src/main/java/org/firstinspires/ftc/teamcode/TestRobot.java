package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.net.IDN;
@Disabled
@TeleOp(name="Test Robot", group="TeleOp")
public class TestRobot extends LinearOpMode {

    private DcMotorEx topLeft, bottomLeft, topRight, bottomRight;
    private DcMotorEx intake;
    public int intakePower;

    private MecanumDrive drive;


    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            //handleLockToggle();

            /*if (parkLocked) {
                lockWheels();
            } else {

            }*/
            intake.setPower(intakePower);
            handleDriving();
            handleLauncherPowerAdjust();
            sendTelemetry();
        }
    }

    private void initHardware() {
        topLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        topRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        bottomRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        //topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        //bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    private void handleDriving() {
        double y = -gamepad1.left_stick_y;      // forward/back
        double r = gamepad1.right_stick_x;      // rotation

        // Triggers for TRUE strafe
        double x = gamepad1.right_trigger - gamepad1.left_trigger;
        x *= 0.60;

        // Mecanum math (correct)
        double fl = y + x + r;
        double bl = y - x + r;
        double fr = y - x - r;
        double br = y + x - r;

        // Normalize so nothing exceeds 1
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))
        );
        if (Math.abs(x) > 0.05) {
            r = 0;
        }

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        topLeft.setPower(fl);
        bottomLeft.setPower(bl);
        topRight.setPower(fr);
        bottomRight.setPower(br);
    }

    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (gamepad1.right_bumper) {
            intakePower += 10;
        }
        if (gamepad1.left_bumper) {
            intakePower -= 10;
        }
    }

        private void sendTelemetry () {
            telemetry.addData("drivetrain speed", bottomRight.getPower());
            telemetry.addData("intake", intakePower);
            //telemetry.addData("Power");

            telemetry.update();
        }
    }

