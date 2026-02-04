package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.InputStream;

public class RedAutoFar {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180), 11.4)
                .build();

        // Flipped start pose: same x, y reflected, heading negated
        Pose2d start = new Pose2d(
                61.5,
                15,
                Math.toRadians(180)
        );

        myBot.runAction(
                myBot.getDrive().actionBuilder(start)
                        .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))
                        //shoot
                        .strafeToLinearHeading(new Vector2d(55,45),Math.toRadians(-90))
                        .strafeTo(new Vector2d(55, 55))
                        .strafeTo(new Vector2d(55,60))
                        .strafeTo(new Vector2d(62, 67))
                        .strafeTo(new Vector2d(55,55))
                        .strafeTo(new Vector2d(68, 67))
                        .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))
                        //shoot
                        .waitSeconds(5)
                        .strafeToLinearHeading(new Vector2d(60,45),Math.toRadians(-90))
                        .strafeTo(new Vector2d(60, 64))
                        //.strafeTo(new Vector2d(60,45))
                        //.strafeTo(new Vector2d(60, 64))
                        .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))
                        //shoot
                        .strafeToLinearHeading(new Vector2d(60,45),Math.toRadians(-90))
                        .strafeTo(new Vector2d(60, 64))
                        //.strafeTo(new Vector2d(60,45))
                        //.strafeTo(new Vector2d(60, 64))

                        .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(150))
                        //shoot

                        .strafeToLinearHeading(new Vector2d(60,45),Math.toRadians(-90))
                        .strafeTo(new Vector2d(60, 64))

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
