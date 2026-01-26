package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class LauncherTuning extends OpMode {

    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    public double highVelocity = 1200;
    public double lowVelocity = 0;

    boolean flywheelEnabled = false;
    boolean timingSpinUp = false;

    long spinUpStartTime = 0;
    double spinUpTimeMs = 0;

    static final double VELOCITY_TOLERANCE = 30; // ticks/sec


    double curTargetVelocity = highVelocity;
    double FR = 14.11332330;
    double PR = 500;

    double FL = 14.05918546;
    double PL = 500;

    double[] stepSizes ={10.0,1.0,0.1,0.01,0.001,0.0001,0.00001,0.000001,0.0000001,0.00000001};

    int stepIndex = 1;

    @Override
    public void init() {
        motorLeft  = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        intakeLeft  = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        //motorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficientsL = new PIDFCoefficients(PL,0,0,FL);
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR,0,0,FR);

        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsL);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);
        telemetry.addLine("init complete");


    }
    private void updateSpinUpTimer(double targetVelocity, double currentVelocity) {

        // Flywheel just started
        if (flywheelEnabled && !timingSpinUp) {
            timingSpinUp = true;
            spinUpStartTime = System.currentTimeMillis();
        }

        // Flywheel reached target speed
        if (timingSpinUp &&
                Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE) {

            spinUpTimeMs = System.currentTimeMillis() - spinUpStartTime;
            timingSpinUp = false;
        }
    }



    public void loop() {

        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
                flywheelEnabled = false;
            } else {curTargetVelocity = highVelocity;
                flywheelEnabled = true;
            }
        }
        if (flywheelEnabled) {
            motorLeft.setVelocity(curTargetVelocity);
            motorRight.setVelocity(curTargetVelocity);
        } else {
            motorLeft.setVelocity(0);
            motorRight.setVelocity(0);
        }



        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            FR-= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            FR+= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            PR+= stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            PR-= stepSizes[stepIndex];
        }


        if (gamepad1.leftBumperWasPressed()){
            FL-= stepSizes[stepIndex];
        }
        if (gamepad1.rightBumperWasPressed()){
            FL+= stepSizes[stepIndex];
        }
        if (gamepad1.rightStickButtonWasPressed()){
            PL+= stepSizes[stepIndex];
        }
        if (gamepad1.leftStickButtonWasPressed()){
            PL-= stepSizes[stepIndex];
        }

        intakeLeft.setPower(0.8);
        intakeRight.setPower(0.8);



        PIDFCoefficients pidfCoefficientsL = new PIDFCoefficients(PL,0,0,FL);
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR,0,0,FR);

        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsL);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);



        motorRight.setVelocity(curTargetVelocity);
        motorLeft.setVelocity(curTargetVelocity);

        double curVelocityL = motorLeft.getVelocity();
        double curVelocityR = motorRight.getVelocity();

// Track spin-up time
        updateSpinUpTimer(curTargetVelocity, curVelocityL);


        double errorL = curTargetVelocity -curVelocityL;
        double errorR = curTargetVelocity -curVelocityR;

        // AFTER gamepad input
// AFTER deciding if flywheel is enabled

        if (!flywheelEnabled) {
            timingSpinUp = false;
            spinUpTimeMs = 0;
        }










        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity Right", "%.8f",curVelocityR);
        telemetry.addData("Current Velocity Left", "%.8f",curVelocityL);
        telemetry.addData("Error Left", "%.8f",errorL);
        telemetry.addData("Error Right", "%.8f",errorR);
        telemetry.addLine("---------------------------------------------");
        telemetry.addData("Tuning P Right", "%.8f (D-Pad U/D)", PR);
        telemetry.addData("Tuning P Left", "%.8f (Right/Left Stick Pressed)", PL);
        telemetry.addData("Tuning F Right", "%.8f (D-Pad L/R)", FR);
        telemetry.addData("Tuning F Left", "%.8f (Right/Left Bumpers)", FL);
        telemetry.addData("Step Size", "%.8f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("Spin-Up Time (ms)", "%.0f", spinUpTimeMs);








    }




}
