package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.InputStream;

public class BlueAutoClose {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180), 11.4)
                .build();

        // Flipped start pose: same x, y reflected, heading negated
        Pose2d start = new Pose2d(
                -50,
                -50,
                Math.toRadians(-130)
        );

        myBot.runAction(
                myBot.getDrive().actionBuilder(start)
                        .setReversed(true)
                        // lineToX: x stays, y is irrelevant
                        .lineToX(-11)
                        // turn angles negated
                        .turn(Math.toRadians(-140))
                        // y values negated
                        .lineToY(-55)
                        .setReversed(false)
                        .lineToY(0)
                        .turn(Math.toRadians(140))
                        .turn(Math.toRadians(-140))
                        .setReversed(true)
                        .strafeTo(new Vector2d(13, -30))
                        .strafeTo(new Vector2d(13, -60))
                        .setReversed(true)
                        .lineToY(-50)
                        .strafeTo(new Vector2d(-15, 0))
                        .turn(Math.toRadians(150))
                        .setReversed(true)
                        .strafeTo(new Vector2d(20, 0))
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
