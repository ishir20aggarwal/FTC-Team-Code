package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.InputStream;

import javax.imageio.ImageIO;

public class BlueAutoFar {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180), 11.4)
                .build();

        // Flipped start pose: same x, y reflected, heading negated
        Pose2d start = new Pose2d(
                61.5,
                -10,
                Math.toRadians(180)
        );

        /**/










        myBot.runAction(
                myBot.getDrive().actionBuilder(start)
                        .strafeTo(new Vector2d(56,-10))
                        .turn(Math.toRadians(20))
                        //shooting
                        .turn(Math.toRadians(-110))
                        .strafeTo(new Vector2d(35,-25))
                        //Add a sleep
                        .strafeTo(new Vector2d(35,-65))
                        .strafeTo(new Vector2d(56,-10))
                        .turn(Math.toRadians(110))
                        //shooting
                        .turn(Math.toRadians(-20))
                        .strafeTo(new Vector2d(30,-10))
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
