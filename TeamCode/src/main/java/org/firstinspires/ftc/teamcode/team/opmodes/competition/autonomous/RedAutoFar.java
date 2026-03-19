package org.firstinspires.ftc.teamcode.team.opmodes.competition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ServoGate;

@Autonomous(name = "6. Autonomous RED Far", group = "Autonomous OpMode")
public class RedAutoFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int littlePause = 200;
        int scorePause = 1000;
        int intakePause = 0;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ScoringSystem scoringSystem = new ScoringSystem(
                (DcMotorEx) hardwareMap.dcMotor.get("launcher"),
                (DcMotorEx) hardwareMap.dcMotor.get("intake"),
                (DcMotorEx) hardwareMap.dcMotor.get("turret"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcher2")
        );

        ServoGate ServoGate = new ServoGate(
                hardwareMap.servo.get("gate")
        );

        Pose2d InitPosition = new Pose2d(63, 26.29, Math.toRadians(180));

        Pose2d ScorePositionPose = new Pose2d(55, 11, Math.toRadians(155));
        Vector2d ScorePositionPos = new Vector2d(55, 11);

        Pose2d HumanAlignPose = new Pose2d(63.5,52, 90);
        Vector2d HumanAlignPos = new Vector2d(63.5,52);

        Pose2d HumanGrabPose = new Pose2d(63.5,72, 90);
        Vector2d HumanGrabPos = new Vector2d(63.5,72);

        Vector2d GPPAlignPos = new Vector2d(38, 35);
        Pose2d GPPAlignPose = new Pose2d(38, 35, Math.toRadians(90));

        Vector2d GPPGrabPos = new Vector2d(38, 50);
        Pose2d GPPGrabPose = new Pose2d(38, 50, Math.toRadians(90));

        Vector2d ParkPos = new Vector2d(55,35);

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        scoringSystem.setLaunchVel(2950);

        int scoreAngle = 155;

        TrajectoryActionBuilder auto = drivetrain.actionBuilder(InitPosition)
                //Init
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0, scoringSystem.launcherUpdateAction())
                .afterTime(0, scoringSystem.intakeAction(0, 1))

                //Move to Scoring Position
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, ServoGate.closeGateAction())

                //Intake Human
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(90),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(90), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds((double)intakePause/1000)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Intake GPP
                .setTangent(Math.toRadians(0))
                .afterTime(1, scoringSystem.intakeAction(0, 1))
                .splineToSplineHeading(GPPAlignPose, Math.toRadians(90))
                .lineToYSplineHeading(GPPGrabPos.y, Math.toRadians(90))
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to Scoring Position
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Intake Human
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(90),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(90), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds((double)intakePause/1000)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Intake Human
                .strafeToLinearHeading(HumanAlignPos, Math.toRadians(90),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(HumanGrabPos.y, Math.toRadians(90), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds((double)intakePause/1000)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(ScorePositionPos, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Park
                .strafeToLinearHeading(ParkPos, Math.toRadians(90));

        waitForStart();
        if (isStopRequested()) return;
        Action builtAuto = auto.build();

        while(opModeIsActive()) {
            blackboard.put("BotPoseRR", drivetrain.localizer.getPose());
            //System.out.printf("%s %s %s %s\n", blackboard, drivetrain, drivetrain.localizer, drivetrain.localizer.getPose());

            telemetry.addData("Intake Motor Velocity: ", scoringSystem.getIntakeVel());
            telemetry.addData("Launcher Motor Velocity ", scoringSystem.getLauncherVel());

            telemetry.addData("Launcher Motor Target Vel: ", scoringSystem.LaunchVel);
            telemetry.addData("BotPoseRR", drivetrain.localizer.getPose());

            dashboardTelemetry.addData("Intake Motor Velocity: ", scoringSystem.getIntakeVel());
            dashboardTelemetry.addData("Launcher Motor Velocity ", scoringSystem.getLauncherVel());

            dashboardTelemetry.addData("Launcher Motor Target Vel: ", scoringSystem.LaunchVel);

            dashboardTelemetry.update();
            builtAuto.run(new TelemetryPacket());
        }
    }
}