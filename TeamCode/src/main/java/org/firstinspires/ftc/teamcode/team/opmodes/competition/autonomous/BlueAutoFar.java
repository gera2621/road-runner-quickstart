package org.firstinspires.ftc.teamcode.team.opmodes.competition.autonomous;

import static org.firstinspires.ftc.teamcode.team.internalLib.AutoMap.poseAngle;
import static org.firstinspires.ftc.teamcode.team.internalLib.AutoMap.trunc;

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
import org.firstinspires.ftc.teamcode.team.internalLib.AutoMap;
import org.firstinspires.ftc.teamcode.team.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ServoGate;

@Autonomous(name = "5. Autonomous BLUE Far", group = "Autonomous OpMode")
public class BlueAutoFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double littlePause = AutoMap.LittlePause;
        double scorePause = AutoMap.ScorePause;
        double intakePause = AutoMap.IntakePause;

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

        final Pose2d InitPosition = AutoMap.BlueFarInitPosition;

        final Pose2d ScorePosition = AutoMap.BlueFarScorePosition;

        final Pose2d HumanAlign = AutoMap.BlueHumanAlign;

        final Pose2d HumanGrab = AutoMap.BlueHumanGrab;

        final Pose2d GPPAlign = AutoMap.BlueGPPAlign;

        final Pose2d GPPGrab = AutoMap.BlueGPPGrab;

        final Pose2d Park = AutoMap.BlueParkFar;

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        scoringSystem.setLaunchVel(2100);

        int scoreAngle = -155;

        TrajectoryActionBuilder auto = drivetrain.actionBuilder(InitPosition)
                //Init
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0, scoringSystem.launcherUpdateAction())
                .afterTime(0, scoringSystem.intakeAction(0, 1))

                //Move to Scoring Position
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))
                .waitSeconds(littlePause)

                //Score
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, ServoGate.closeGateAction())

                //Intake GPP
                .setTangent(Math.toRadians(0))
                .afterTime(1, scoringSystem.intakeAction(0, 1))
                .splineToSplineHeading(GPPAlign, poseAngle(GPPAlign))
                .lineToYSplineHeading(trunc(GPPGrab).y, poseAngle(GPPGrab))
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to Scoring Position
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Intake Human
                .strafeToLinearHeading(trunc(HumanAlign), poseAngle(HumanAlign),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(trunc(HumanGrab).y, poseAngle(HumanAlign), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds(intakePause)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Intake Human
                .strafeToLinearHeading(trunc(HumanAlign), poseAngle(HumanAlign),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(trunc(HumanGrab).y, poseAngle(HumanAlign), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds(intakePause)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Intake Human
                .strafeToLinearHeading(trunc(HumanAlign), poseAngle(HumanAlign),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(trunc(HumanGrab).y, poseAngle(HumanAlign), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds(intakePause)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))

                //Park
                .strafeToLinearHeading(trunc(Park), poseAngle(Park));

        waitForStart();
        if (isStopRequested()) return;
        Action builtAuto = auto.build();

        while(opModeIsActive()) {
            blackboard.put("BotPoseRR", drivetrain.localizer.getPose());
            scoringSystem.launcherUpdate();
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