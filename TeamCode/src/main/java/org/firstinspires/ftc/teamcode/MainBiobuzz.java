package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@TeleOp(name="MainBiobuzz", group="TeleOp")
public class MainBiobuzz extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel;
    private ServoImplEx servoLeft, servoRight;
    private DcMotorEx intake;

    private MecanumDrive drive;
    private boolean servoBusy = false;
    private boolean isPollen = false;
    private boolean bPressedLast = false;



    private boolean launcherOn = false;
    private double launchPower = 1;
    private final double pollenServoPos = 0.99;
    private final double nectarServoPos = 0.82;

    private final double POWER_STEP = 0.05;
    private final long ADJUST_DELAY_MS = 120;
    private long lastAdjustTime = 0;

    private boolean intakeMotorsOn = false;
    private boolean xPressedLast = false;

    private boolean yPressedLast = false;


    private ColorBlobLocatorProcessor pollenLocator;


    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            // REQUIRED: keep RoadRunner odometry updated every loop
            drive.updatePoseEstimate();
            launcherOn = true;

            handleDriving();
            handleLauncherPowerAdjust();
            handleLaunchSequence();
            handleIntakeMotorsToggle();
            handleSwitch();
            launch();
            sendTelemetry();
        }
    }

    private void initHardware() {
        leftFront    = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack= hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel  = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        /*PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PL, IL, DL, FL);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        */
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        servoLeft = hardwareMap.get(ServoImplEx.class, "servoLeft");
        servoRight = hardwareMap.get(ServoImplEx.class,"servoRight");
        servoRight.setDirection(ServoImplEx.Direction.REVERSE);
        servoLeft.setPosition(pollenServoPos);
        servoRight.setPosition(pollenServoPos);



        // REQUIRED: initialize RoadRunner drive so lock can command motors + read odometry
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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




        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }
        //increase speed by 10%
        if(gamepad1.a && gamepad1.dpad_up){
            fl*=1.1;
            bl*=1.1;
            fr*=1.1;
            br*=1.1;
        }
        //decrease speed by 10%
        if(gamepad1.a && gamepad1.dpad_down){
            fl*=0.9;
            bl*=0.9;
            fr*=0.9;
            br*=0.9;
        }

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }
    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustTime < ADJUST_DELAY_MS) return;

        if (gamepad1.right_bumper) {
            launchPower = Math.min(1.0, launchPower + POWER_STEP);
            launchPower+=POWER_STEP;
            lastAdjustTime = now;
        }
        if (gamepad1.left_bumper) {
            launchPower = Math.max(0.0, launchPower - POWER_STEP);
            launchPower-=POWER_STEP;
            lastAdjustTime = now;
        }
    }

    private void launch() {
        double p = launcherOn ? -launchPower : 0;
        if (gamepad1.b && !bPressedLast) {
            launcherOn = !launcherOn;
        }
        flywheel.setPower(launcherOn ? p : 0.0);
    }




    private void handleIntakeMotorsToggle() {
        if (gamepad1.x && !xPressedLast) {
            intakeMotorsOn = !intakeMotorsOn;
        }
        xPressedLast = gamepad1.x;
        intake.setPower(intakeMotorsOn ? 0.8 : 0.0);

    }



    private void handleLaunchSequence() {

    }
    private void handleSwitch() {
        boolean y = gamepad1.y;

        if (y && !yPressedLast) {
            isPollen = !isPollen;

        }
        yPressedLast = gamepad1.y;
        servoLeft.setPosition(isPollen ? nectarServoPos : pollenServoPos);
        servoRight.setPosition(isPollen ? nectarServoPos : pollenServoPos);

    }

    private void sleepQuiet(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    private void sendTelemetry() {
        telemetry.addData("Drivetrain speed:",rightBack.getPower());
        telemetry.addData("Current Pose",drive.localizer.getPose());
        telemetry.addLine("||||||||");
        telemetry.addData("Flywheel Power:",flywheel.getPower());
        telemetry.addData("Intake Power:",intake.getPower());
        telemetry.addData("Servo Left Position:",servoLeft.getPosition());
        telemetry.addData("Servo Right Position:",servoRight.getPosition());
        telemetry.addLine("||||||||");

        telemetry.update();
    }
}




