package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.InputStream;

public class BlueAutoFar {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        // Flipped start pose: same x, y reflected, heading negated
        Pose2d start = new Pose2d(
                61.5,
                -15,
                Math.toRadians(180)
        );

        myBot.runAction(
                myBot.getDrive().actionBuilder(start)
                        .strafeTo(new Vector2d(56,-12))
                        .turn(Math.toRadians(20))
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(47, -42), Math.toRadians(90))
                        .splineTo(new Vector2d(65, -62), Math.toRadians(165))
                        .strafeToLinearHeading(new Vector2d(60, -5), Math.toRadians(180))
                        .turn(Math.toRadians(20))

                        .build()
        );

        // Optional field image
        BufferedImage fieldImage = null;
        try {
            InputStream stream = RedAutoClose.class.getResourceAsStream("/decode.png");
            fieldImage = ImageIO.read(stream);
        } catch (Exception e) {
            System.out.println("decode.png missing. Place it in src/main/resources/");
        }

        if (fieldImage != null) {
            meepMeep.setBackground(fieldImage)
                    .setDarkMode(false)
                    .setBackgroundAlpha(1f);
        }

        meepMeep.addEntity(myBot).start();
    }
}
