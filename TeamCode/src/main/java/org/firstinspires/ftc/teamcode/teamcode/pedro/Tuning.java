package org.firstinspires.ftc.teamcode.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
