package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutoMap.poseAngle;
import static com.example.meepmeeptesting.AutoMap.trunc;

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

        final Pose2d InitPosition = AutoMap.BlueGoalInitPosition;

        final Pose2d PreScorePosition = AutoMap.BluePreScorePosition;

        final Pose2d ScorePosition = AutoMap.BlueScorePosition;

        final Pose2d CollectAlign = AutoMap.BlueCollectAlign;

        final Pose2d PPGAlign = AutoMap.BluePPGAlign;

        final Pose2d PPGGrab = AutoMap.BluePPGGrab;

        final Pose2d PGPAlign = AutoMap.BluePGPAlign;

        final Pose2d PGPGrab = AutoMap.BluePGPGrab;

        final Pose2d GPPAlign = AutoMap.BlueGPPAlign;

        final Pose2d GPPGrab = AutoMap.BlueGPPGrab;

        final Pose2d GatePark = AutoMap.BlueGatePark;

        final Pose2d GateIntake = AutoMap.BlueGateIntake;

        final Pose2d GateLeave = AutoMap.BlueGateLeave;

        final Pose2d Park = AutoMap.BlueParkOnly;

        int scoreAngle = -130;


        myBot.runAction(myBot.getDrive().actionBuilder(InitPosition)
                .strafeToLinearHeading(trunc(Park), poseAngle(Park))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}