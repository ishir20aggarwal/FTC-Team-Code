package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name="MainBiobuzz", group="TeleOp")
public class MainBiobuzz extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel;
    private ServoImplEx servoLeft, servoRight;

    private DcMotorEx leftTransfer, rightTransfer;
    private DcMotorEx intake;

    private boolean isPollen = false;
    private boolean bPressedLast = false;

    private boolean launcherOn = false;
    private double launchPower = 1200;

    private final double pollenServoPos = 0.6;
    private final double nectarServoPos = 0.4;
    private final double initServoPos =
            0.3;

    private final double POWER_STEP = 50;
    private final long ADJUST_DELAY_MS = 120;
    private long lastAdjustTime = 0;

    private boolean intakeMotorsOn = false;
    private boolean xPressedLast = false;

    private boolean yPressedLast = false;

    // Vision Members

    /*private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor redLocator;
    private ColorBlobLocatorProcessor blueLocator;
    private ColorBlobLocatorProcessor yellowLocator;
*/
    private String detectedColor = "NONE";

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            handleDriving();
            handleLauncherPowerAdjust();
            handleLaunchSequence();
            handleIntakeMotorsToggle();
            handleSwitch();
            launch();
            sendTelemetry();
        }

        /*if (visionPortal != null) {
            visionPortal.close();
        }*/
    }

    private void initHardware() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(14,0,0,20.17037037037037037);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidf);


        servoLeft = hardwareMap.get(ServoImplEx.class, "servoLeft");
        servoRight = hardwareMap.get(ServoImplEx.class, "servoRight");
        servoRight.setDirection(ServoImplEx.Direction.REVERSE);

        leftTransfer = hardwareMap.get(DcMotorEx.class, "leftTransfer");
        rightTransfer = hardwareMap.get(DcMotorEx.class, "rightTransfer");
        rightTransfer.setDirection(DcMotorSimple.Direction.REVERSE);






        // Default starting positions
        servoLeft.setPosition(initServoPos);
        servoRight.setPosition(initServoPos);

        // Separate processors to track each color range
        /*redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(2)
                .build();

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(2)
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(2)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(redLocator)
                .addProcessor(blueLocator)
                .addProcessor(yellowLocator)
                .build();*/

        //drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    // 1. Put these at the top of your OpMode class (outside handleDriving)
    private double speedScale = 1.0;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // 2. Replace your existing method with this one
    private void handleDriving() {
        double y = -gamepad1.left_stick_y;      // forward/back
        double r = gamepad1.right_stick_x;      // rotation

        double x = gamepad1.right_trigger - gamepad1.left_trigger;
        x *= 0.60;

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

        if (gamepad1.a && gamepad1.dpad_up && !lastDpadUp) {
            speedScale = Math.min(1.0, speedScale + 0.1);
        }


        if (gamepad1.a && gamepad1.dpad_down && !lastDpadDown) {
            speedScale = Math.max(0.1, speedScale - 0.1);
        }


        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;


        leftFront.setPower(fl * speedScale);
        leftBack.setPower(bl * speedScale);
        rightFront.setPower(fr * speedScale);
        rightBack.setPower(br * speedScale);
    }

    private void handleLauncherPowerAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustTime < ADJUST_DELAY_MS) return;

        if (gamepad1.right_bumper) {
            launchPower += POWER_STEP;
            lastAdjustTime = now;
        }
        if (gamepad1.left_bumper) {
            launchPower -= POWER_STEP;
            lastAdjustTime = now;
        }
    }

    private void launch() {
        double p = launcherOn ? launchPower : 0;
        flywheel.setVelocity(p);
    }

    private void handleIntakeMotorsToggle() {
        if (gamepad1.x && !xPressedLast) {
            intakeMotorsOn = !intakeMotorsOn;
        }
        intake.setPower(intakeMotorsOn ? 1 : 0);
        rightTransfer.setPower(intakeMotorsOn ? 0.5 : 0);
        leftTransfer.setPower(intakeMotorsOn ? 0.5 : 0);
        xPressedLast = gamepad1.x;
    }

    private void handleLaunchSequence() {
        boolean b = gamepad1.b;

        if (b && !bPressedLast) {
            launcherOn = !launcherOn;
        }

        bPressedLast = b;
    }

    private boolean hasValidBlob(List<ColorBlobLocatorProcessor.Blob> blobs, double minArea) {
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (blob.getContourArea() > minArea) {
                return true;
            }
        }
        return false;
    }

    private void handleSwitch() {
        boolean y = gamepad1.y;
        if (y && !yPressedLast) {
            isPollen = !isPollen;
        }
        yPressedLast = y;

       /* boolean redFound = hasValidBlob(redLocator.getBlobs(), 50);
        boolean blueFound = hasValidBlob(blueLocator.getBlobs(), 50);
        boolean yellowFound = hasValidBlob(yellowLocator.getBlobs(), 50);

        // Determine current color seen
        if (redFound) {
            detectedColor = "RED";
        } else if (blueFound) {
            detectedColor = "BLUE";
        } else if (yellowFound) {
            detectedColor = "YELLOW";
        } else {
            detectedColor = "NONE";
        }*/


        if (isPollen) {
            servoLeft.setPosition(pollenServoPos);
            servoRight.setPosition(pollenServoPos);
        } else {
            servoLeft.setPosition(nectarServoPos);
            servoRight.setPosition(nectarServoPos);
        }
    }

    private void sendTelemetry() {
        telemetry.addData("Drivetrain speed:", rightBack.getPower());
        telemetry.addData("Speed scale:", speedScale);
        telemetry.addLine("||||||||");


        telemetry.addData("Detected Color:", detectedColor);


        telemetry.addData("Launcher Velocity:", flywheel.getVelocity());
        telemetry.addData("Launcher Power:",flywheel.getPower());
        telemetry.addData("Intake Power:", intake.getPower());
        telemetry.addData("Transfer Power:",leftTransfer.getPower());
        telemetry.addData("Servo Left Position:", servoLeft.getPosition());
        telemetry.addData("Servo Right Position:", servoRight.getPosition());
        telemetry.addLine("||||||||");

        telemetry.update();
    }
}