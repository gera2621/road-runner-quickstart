package org.firstinspires.ftc.teamcode.team.internalLib;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoMap {
    
    public static final double LittlePause = 0.2;
    public static final double ScorePause = 1;
    public static final double GatePause = 2;

    public static final double IntakePause = 0.25;

    // Blue Positions
    public static final Pose2d BlueGoalInitPosition = new Pose2d(-49.4, -47.9, Math.toRadians(-125));

    public static final Pose2d BlueFarInitPosition = new Pose2d(63, -26.29, Math.toRadians(-180));

    public static final Pose2d BluePreScorePosition = new Pose2d(-30, -24, Math.toRadians(-125));

    public static final Pose2d BlueScorePosition = new Pose2d(-24, -24, Math.toRadians(-135));

    public static final Pose2d BlueFarScorePosition = new Pose2d(55, -23, Math.toRadians(-155));

    public static final Pose2d BlueCollectAlign = new Pose2d(-30, -18, Math.toRadians(-90));

    public static final Pose2d BluePPGAlign = new Pose2d(-10,-40, Math.toRadians(-90));

    public static final Pose2d BluePPGGrab = new Pose2d(-10,-44, Math.toRadians(-90));

    public static final Pose2d BluePGPAlign = new Pose2d(13,-32, Math.toRadians(-90));

    public static final Pose2d BluePGPGrab = new Pose2d(13,-49, Math.toRadians(-90));

    public static final Pose2d BlueGPPAlign = new Pose2d(36, -35, Math.toRadians(-90));

    public static final Pose2d BlueGPPGrab = new Pose2d(36, -48, Math.toRadians(-90));

    public static final Pose2d BlueHumanAlign = new Pose2d(63.5,-52, -90);

    public static final Pose2d BlueHumanGrab = new Pose2d(63.5,-72, -90);

    public static final Pose2d BlueGatePark = new Pose2d(3, -55.5, Math.toRadians(0));

    public static final Pose2d BlueGateIntake = new Pose2d(12.5, -56.5, Math.toRadians(-120));

    public static final Pose2d BlueGateLeave = new Pose2d(3, -25, Math.toRadians(0));

    public static final Pose2d BluePark = new Pose2d(-17, -32, Math.toRadians(-90));

    public static final Pose2d BlueParkOnly = new Pose2d(-34, -48, Math.toRadians(-90));

    public static final Pose2d BlueParkFar = new Pose2d(55, -35, Math.toRadians(-90));


    //Red Positions


    public static final Pose2d RedGoalInitPosition = new Pose2d(-49.4, 47.9, Math.toRadians(125));

    public static final Pose2d RedFarInitPosition = new Pose2d(63, 26.29, Math.toRadians(180));

    public static final Pose2d RedPreScorePosition = new Pose2d(-30, 24, Math.toRadians(125));

    public static final Pose2d RedScorePosition = new Pose2d(-24, 24, Math.toRadians(130));

    public static final Pose2d RedFarScorePosition = new Pose2d(55, 23, Math.toRadians(155));

    public static final Pose2d RedCollectAlign = new Pose2d(-30, 18, Math.toRadians(90));

    public static final Pose2d RedPPGAlign = new Pose2d(-10,40, Math.toRadians(90));

    public static final Pose2d RedPPGGrab = new Pose2d(-10,44, Math.toRadians(90));

    public static final Pose2d RedPGPAlign = new Pose2d(13,32, Math.toRadians(90));

    public static final Pose2d RedPGPGrab = new Pose2d(13,49, Math.toRadians(90));

    public static final Pose2d RedGPPAlign = new Pose2d(36, 35, Math.toRadians(90));

    public static final Pose2d RedGPPGrab = new Pose2d(36, 48, Math.toRadians(90));

    public static final Pose2d RedHumanAlign = new Pose2d(63.5,52, 90);

    public static final Pose2d RedHumanGrab = new Pose2d(63.5,72, 90);

    public static final Pose2d RedGatePark = new Pose2d(3, 55.5, Math.toRadians(0));

    public static final Pose2d RedGateIntake = new Pose2d(12.5, 56.5, Math.toRadians(120));

    public static final Pose2d RedGateLeave = new Pose2d(3, 25, Math.toRadians(0));

    public static final Pose2d RedPark = new Pose2d(-17, 32, Math.toRadians(90));

    public static final Pose2d RedParkOnly = new Pose2d(-34, 48, Math.toRadians(90));

    public static final Pose2d RedParkFar = new Pose2d(55, 35, Math.toRadians(90));

    public static Vector2d trunc(Pose2d InputPose) {
        return new Vector2d(InputPose.position.x, InputPose.position.y);
    }
    public static double poseAngle(Pose2d InputPose) {
        return InputPose.heading.toDouble();
    }
}
