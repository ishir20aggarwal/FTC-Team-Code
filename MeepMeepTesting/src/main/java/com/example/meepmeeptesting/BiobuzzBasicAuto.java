package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.archive.RedAutoClose;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.InputStream;

import javax.imageio.ImageIO;

public class BiobuzzBasicAuto {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180), 11.4)
                .build();

        // Flipped start pose: same x, y reflected, heading negated
        Pose2d start = new Pose2d(
                62,
                -15,
                Math.toRadians(180)
        );

        myBot.runAction(
                myBot.getDrive().actionBuilder(start)
                        //shoot 4 pollen
                        .waitSeconds(2.5)
                        .strafeToLinearHeading(new Vector2d(23,-46),Math.toRadians(-90))
                        .strafeTo(new Vector2d(23,-59))
                        //intake
                        .waitSeconds(1.0)
//                        .strafeTo(new Vector2d(22,-46))
//                        .strafeTo(new Vector2d(22,-46))
//                        .splineToLinearHeading(new Pose2d(-52,-18,0),Math.toRadians(100))
                        .strafeTo(new Vector2d(23,-57))
                        .strafeToLinearHeading(new Vector2d(-36,-36),Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-52,-18),Math.toRadians(0))
                        //shoot 4 pollen
                        .waitSeconds(2.5)
                        .turnTo(Math.toRadians(180),new TurnConstraints(10,-2,2))
                        .strafeToConstantHeading(new Vector2d(-59,-24))
                        //intake
                        .waitSeconds(1.0)
                        .strafeToLinearHeading(new Vector2d(-50,-24),Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-52,-18),Math.toRadians(0))
                        //shoot 4 pollen
                        .waitSeconds(2.5)
                        .strafeToLinearHeading(new Vector2d(-36,-36),Math.toRadians(0))
                        .strafeToConstantHeading(new Vector2d(55,-61))
                        //intake
                        .strafeToConstantHeading(new Vector2d(62,-61))
                        .strafeToConstantHeading(new Vector2d(55,-61))
                        .strafeToLinearHeading(new Vector2d(56,-16),Math.toRadians(180))
                        //shoot 4 pollen
                        .waitSeconds(2.5)
                        .strafeToConstantHeading(new Vector2d(20,-50))
                        .strafeToConstantHeading(new Vector2d(-36,-60))




                        .build()
        );

        // Optional field image
        BufferedImage fieldImage = null;
        try {
            InputStream stream = RedAutoClose.class.getResourceAsStream("/biobuzz.png");
            fieldImage = ImageIO.read(stream);
        } catch (Exception e) {
            System.out.println("biobuzz.png missing. Place it in src/main/resources/");
        }

        if (fieldImage != null) {
            meepMeep.setBackground(fieldImage)
                    .setDarkMode(true)
                    .setBackgroundAlpha(1f);
        }

        meepMeep.addEntity(myBot).start();
    }
}
