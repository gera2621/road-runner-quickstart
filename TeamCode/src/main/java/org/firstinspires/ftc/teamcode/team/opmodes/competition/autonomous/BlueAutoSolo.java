package org.firstinspires.ftc.teamcode.team.opmodes.competition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ServoGate;

@Autonomous(name = "1. Autonomous BLUE Goal Solo", group = "Autonomous OpMode")
public class BlueAutoSolo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int littlePause = 200;
        int scorePause = 1000;
        int gatePause = 2500;

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

        Pose2d InitPosition = new Pose2d(-49.4, -47.9, Math.toRadians(-125));

        Pose2d PreScorePositionPose = new Pose2d(-30, -24, Math.toRadians(-125));
        Vector2d PreScorePosition = new Vector2d( -30, -24);

        Pose2d ScorePositionPose = new Pose2d(-24, -24, Math.toRadians(-135));
        Vector2d ScorePosition = new Vector2d(-24, -24);

        Vector2d CollectAlignPos = new Vector2d(-30, -18);

        Vector2d PPGAlignPos = new Vector2d(-10,-40);
        Pose2d PPGAlignPose = new Pose2d(-10,-40, Math.toRadians(-90));

        Vector2d PPGGrabPos = new Vector2d(-10,-44);
        Pose2d PPGGrabPose = new Pose2d(-10,-44, Math.toRadians(-90));

        Vector2d PGPAlignPos = new Vector2d(17,-32);
        Pose2d PGPAlignPose = new Pose2d(17,-32, Math.toRadians(-90));

        Vector2d PGPGrabPos = new Vector2d(17,-49);
        Pose2d PGPGrabPose = new Pose2d(17,-49, Math.toRadians(-90));

        Vector2d PGPGatePos = new Vector2d(15.5, -52);
        Pose2d PGPGatePose = new Pose2d(15.5, -52, Math.toRadians(-90));

        Vector2d GateParkPos = new Vector2d(3, -55.5);
        Pose2d GateParkPose = new Pose2d(3, -55.5, Math.toRadians(0));

        Vector2d GateIntakePos = new Vector2d(13.5, -52.5);
        Pose2d GateIntakePose = new Pose2d(13.5, -52.5, Math.toRadians(-120));

        Vector2d GateLeavePos = new Vector2d(3,-25);
        Pose2d GateLeavePose = new Pose2d(3, -25, Math.toRadians(0));

        Vector2d GPPAlignPos = new Vector2d(36, -35);
        Pose2d GPPAlignPose = new Pose2d(36, -35, Math.toRadians(-90));

        Vector2d GPPGrabPos = new Vector2d(36, -48);
        Pose2d GPPGrabPose = new Pose2d(36, -48, Math.toRadians(-90));

        Vector2d ParkPos = new Vector2d(-0, -40);
        Pose2d ParkPose = new Pose2d(-0, -40, Math.toRadians(-90));

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        int scoreAngle = -130;

        TrajectoryActionBuilder auto = drivetrain.actionBuilder(InitPosition)
                //Init
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0, scoringSystem.launcherUpdateAction())
                .afterTime(0, scoringSystem.intakeAction(0, 1))

                //Move to Scoring Position
                .strafeToLinearHeading(ScorePosition, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, ServoGate.closeGateAction())

                //Intake PGP
                .setTangent(Math.toRadians(-315))

                .splineToSplineHeading(PGPAlignPose, Math.toRadians(-90))
                .strafeToLinearHeading(PGPGrabPos, Math.toRadians(-90))
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .lineToYSplineHeading(PGPAlignPos.y, Math.toRadians(-90))
                .splineToLinearHeading(ScorePositionPose, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //MoveToGate 1
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(GateIntakePose, Math.toRadians(-120))
                .waitSeconds((double)gatePause/1000)
                .afterTime(0.25, scoringSystem.intakeAction(0, 0))

                //Move to scoring position
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(ScorePositionPose, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Intake PPG
                .afterTime(0.25, scoringSystem.intakeAction(0, 1))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(PPGAlignPose, Math.toRadians(-90))
                .lineToYSplineHeading(PPGGrabPos.y, Math.toRadians(-90))
                .afterTime(0.25, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(ScorePosition, Math.toRadians(scoreAngle))

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
                .splineToSplineHeading(GPPAlignPose, Math.toRadians(-90))
                .lineToYSplineHeading(GPPGrabPos.y, Math.toRadians(-90))
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to Scoring Position
                .strafeToLinearHeading(ScorePosition, Math.toRadians(scoreAngle))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds((double)littlePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds((double)scorePause/1000)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Park
                .afterTime(0, scoringSystem.launcherOffAction())
                .strafeToLinearHeading(PPGGrabPos, Math.toRadians(-180));

        waitForStart();
        if (isStopRequested()) return;
        Action builtAuto = auto.build();
        //blackboard.put("AutoEndCoordidnates", 2);

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