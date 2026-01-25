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
    public double highVelocity = 760;
    public double lowVelocity = 0;

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


    public void loop() {

        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity;}
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



        PIDFCoefficients pidfCoefficientsL = new PIDFCoefficients(PL,0,0,FL);
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR,0,0,FR);

        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsL);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);



        motorRight.setVelocity(curTargetVelocity);
        motorLeft.setVelocity(curTargetVelocity);

        double curVelocityL = motorLeft.getVelocity();
        double curVelocityR = motorRight.getVelocity();

        double errorL = curTargetVelocity -curVelocityL;
        double errorR = curTargetVelocity -curVelocityR;









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







    }




}
