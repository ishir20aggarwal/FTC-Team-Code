package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseStorage {
    private PoseStorage() {}
    public static volatile Pose2d lastPose = new Pose2d(0, 0, 0);
}
