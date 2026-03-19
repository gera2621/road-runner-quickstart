package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(15.375, 14.4)
                .build();

        double scorePause = 1200.0;
        double prescorePause = 1200.0;
        double smallPause = 250.0;
        double gateIntakePause = 1500.0;

        Pose2d InitPosition = new Pose2d(63, -26.29, Math.toRadians(-180));

        Pose2d ScorePositionPose = new Pose2d(55, -11, Math.toRadians(-155));
        Vector2d ScorePositionPos = new Vector2d(55, -11);

        Pose2d HumanAlignPose = new Pose2d(61.5,-52, -90);
        Vector2d HumanAlignPos = new Vector2d(61.5,-52);

        Pose2d HumanGrabPose = new Pose2d(61.5,-60, -90);
        Vector2d HumanGrabPos = new Vector2d(61.5,-60);

        Vector2d GPPAlignPos = new Vector2d(36, -35);
        Pose2d GPPAlignPose = new Pose2d(36, -35, Math.toRadians(-90));

        Vector2d GPPGrabPos = new Vector2d(36, -48);
        Pose2d GPPGrabPose = new Pose2d(36, -48, Math.toRadians(-90));


        myBot.runAction(myBot.getDrive().actionBuilder(InitPosition)
                //score preloaded
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(-155))
                .waitSeconds(scorePause/1000)

                //get human artifacts
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(-90))
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(-90))

                //go score
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(-132.5))
                .waitSeconds(scorePause/1000)

                //get gpp artifacts
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(GPPAlignPose, Math.toRadians(-90))
                .lineToYSplineHeading(GPPGrabPos.y, Math.toRadians(-90))

                //go score
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(-132.5))
                .waitSeconds(scorePause/1000)

                //get human artifacts
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(-90))
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(-90))

                //go score
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(-132.5))
                .waitSeconds(scorePause/1000)

                //get human artifacts
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(-90))
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(-90))

                //go score
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(-132.5))
                .waitSeconds(scorePause/1000)

                //get human artifacts
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(-90))
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(-90))


                //build !
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}