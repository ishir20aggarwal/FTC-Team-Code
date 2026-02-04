/*package com.example.meepmeeptesting;

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
                        .lineToX(-11)
                        //shoot
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(13, -26),Math.toRadians(90))
                        .strafeTo(new Vector2d(13, -62))
                        .setReversed(true)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(13,-41),Math.toRadians(-180))
                        .strafeTo(new Vector2d(3,-54))
                        .strafeTo(new Vector2d(0,0))
                        .strafeToLinearHeading(new Vector2d(-11,-3),Math.toRadians(-130))
                        //shoot
                        .strafeToLinearHeading(new Vector2d(-11,-26),Math.toRadians(90))
                        .strafeTo(new Vector2d(-11,-56))
                        .strafeToLinearHeading(new Vector2d(-11,-3),Math.toRadians(-130))
                        //shoot
                        .strafeToLinearHeading(new Vector2d(36,-26),Math.toRadians(90))
                        .strafeTo(new Vector2d(36,-62))
                        .strafeToLinearHeading(new Vector2d(-11,-3),Math.toRadians(-130))
                        //shoot
                        .strafeTo(new Vector2d(5,-15))




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
}*/


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
                            .lineToX(-11)
                             .setReversed(true)
                            .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(90))
                            .strafeTo(new Vector2d(10, -55))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-2, -40), Math.toRadians(-180))
                            .strafeTo(new Vector2d(-1, -47))
                            .waitSeconds(0.20)
                            .strafeTo(new Vector2d(0, -30))
                            .strafeToLinearHeading(new Vector2d(-15, -20), Math.toRadians(-130))
                            .strafeToLinearHeading(new Vector2d(-13, -26), Math.toRadians(90))
                            .setReversed(true)
                            .strafeTo(new Vector2d(-13, -60))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-15, -20), Math.toRadians(-130))
                            .strafeToLinearHeading(new Vector2d(25, -35), Math.toRadians(90))
                            .setReversed(true)
                            .strafeTo(new Vector2d(28, -70))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-20, -35), Math.toRadians(-130))
                            .strafeTo(new Vector2d(-15, -40))




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

