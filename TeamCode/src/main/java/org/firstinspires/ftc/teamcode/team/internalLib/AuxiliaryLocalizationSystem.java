package org.firstinspires.ftc.teamcode.team.internalLib;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AuxiliaryLocalizationSystem {
    public static double getDistance(Pose2D Robot, Pose2D Target) {
        double RobotX = Robot.getX(DistanceUnit.MM);
        double RobotY = Robot.getY(DistanceUnit.MM);
        double TargetX = Target.getX(DistanceUnit.MM);
        double TargetY = Target.getY(DistanceUnit.MM);

        return Math.abs(Math.sqrt(Math.pow(TargetX - RobotX, 2) + Math.pow(TargetY - RobotY, 2)));
    }

    public static double getAngle(Pose2D Robot, Pose2D Target) {
        double RobotX = Robot.getX(DistanceUnit.MM);
        double RobotY = Robot.getY(DistanceUnit.MM);
        double TargetX = Target.getX(DistanceUnit.MM);
        double TargetY = Target.getY(DistanceUnit.MM);

        double RobotHeading = Robot.getHeading(AngleUnit.DEGREES);

        double BotAngle = Math.toDegrees(Math.atan2(-(TargetY - RobotY), (TargetX - RobotX)));

        double difference = -BotAngle - RobotHeading;

        while (difference > 180) difference -= 360;
        while (difference <= -180) difference += 360;

        return difference;
    }

    public static double getDistancefromAngle(double ty, double knownHeightMM){
        double LimelightAngle = 40;

        double limelightLensHeightMM = 355.6;

        double angleToGoalRadians = Math.toRadians(LimelightAngle + ty);

        return  (knownHeightMM - (limelightLensHeightMM ) / Math.tan(angleToGoalRadians));
    }
    public static Pose2D ConvertRRPoseToDriverPose(Pose2d inputPose){
        double x = inputPose.position.x * -25.4;
        double y = inputPose.position.y * -25.4;
        double head = Math.toDegrees(inputPose.heading.toDouble());
        head = DegreesRollover(head+180);

        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, head);
    }
    public static double DegreesRollover(double input){
        while (input > 180) input -= 360;
        while (input <= -180) input += 360;

        return input;
    }
}
