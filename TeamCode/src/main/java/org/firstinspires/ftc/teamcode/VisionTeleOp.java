package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "VisionTeleOp", group = "Vision")
public class VisionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null && detection.robotPose != null) {

                        // FIX: Use getPosition() and getOrientation()
                        double x = detection.robotPose.getPosition().x;
                        double y = detection.robotPose.getPosition().y;
                        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                        // Draw on Dashboard Field
                        packet.fieldOverlay().setStroke("Blue");
                        packet.fieldOverlay().strokeCircle(x, y, 9);

                        // Draw heading line
                        double lineX = x + Math.cos(Math.toRadians(heading)) * 10;
                        double lineY = y + Math.sin(Math.toRadians(heading)) * 10;
                        packet.fieldOverlay().strokeLine(x, y, lineX, lineY);

                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("Robot X", x);
                        telemetry.addData("Robot Y", y);
                        telemetry.addData("Heading", heading);
                    }
                }
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        visionPortal.close();
    }
}