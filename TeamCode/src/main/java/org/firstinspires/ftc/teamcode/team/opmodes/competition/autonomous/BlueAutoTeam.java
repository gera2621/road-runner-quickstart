package org.firstinspires.ftc.teamcode.team.opmodes.competition.autonomous;

import static org.firstinspires.ftc.teamcode.team.internalLib.AutoMap.poseAngle;
import static org.firstinspires.ftc.teamcode.team.internalLib.AutoMap.trunc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.internalLib.AutoMap;
import org.firstinspires.ftc.teamcode.team.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ServoGate;

@Autonomous(name = "2. Autonomous BLUE Goal Team", group = "Autonomous OpMode")
public class BlueAutoTeam extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double littlePause = AutoMap.LittlePause;
        double scorePause = AutoMap.ScorePause;
        double gatePause = AutoMap.GatePause;

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

        final Pose2d Park = AutoMap.BluePark;

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        int scoreAngle = -130;

        TrajectoryActionBuilder auto = drivetrain.actionBuilder(InitPosition)
                //Init
                .afterTime(0, ServoGate.closeGateAction())
                .afterTime(0, scoringSystem.launcherUpdateAction())
                .afterTime(0, scoringSystem.intakeAction(0, 1))

                //Move to Scoring Position
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, ServoGate.closeGateAction())

                //Intake PGP
                .setTangent(Math.toRadians(-315))

                .splineToSplineHeading(PGPAlign, poseAngle(PGPAlign))
                .strafeToLinearHeading(trunc(PGPGrab), poseAngle(PGPGrab))
                .afterTime(0.5, scoringSystem.intakeAction(0, 0))

                //Move back, then hit gate
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(GatePark, poseAngle(GatePark))

                //Move to scoring Positon
                .setTangent(Math.toRadians(-270))
                .lineToYSplineHeading(trunc(PGPAlign).y, poseAngle(PGPAlign))  //angle may be better as 0?
                .splineToLinearHeading(ScorePosition, poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //MoveToGate 1
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(GateIntake, poseAngle(GateIntake))
                .waitSeconds(gatePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))

                //Move to scoring position
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(ScorePosition, poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //MoveToGate 2
                .afterTime(0.5, scoringSystem.intakeAction(0, 1))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(GateIntake, poseAngle(GateIntake))
                .waitSeconds(gatePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))

                //Move to scoring position
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(ScorePosition, poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Intake PPG
                .afterTime(0.25, scoringSystem.intakeAction(0, 1))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(PPGAlign, poseAngle(PPGAlign))
                .lineToYSplineHeading(trunc(PPGGrab).y, poseAngle(PPGGrab))
                .afterTime(0.25, scoringSystem.intakeAction(0, 0))

                //Move to scoring Positon
                .strafeToLinearHeading(trunc(ScorePosition), poseAngle(ScorePosition))

                //Score
                .afterTime(0,ServoGate.openGateAction())
                .waitSeconds(littlePause)
                .afterTime(0, scoringSystem.intakeAction(0, 1))
                .waitSeconds(scorePause)
                .afterTime(0, scoringSystem.intakeAction(0, 0))
                .afterTime(0, ServoGate.closeGateAction())

                //Park
                .afterTime(0, scoringSystem.launcherOffAction())
                .strafeTo(trunc(Park));

        waitForStart();
        if (isStopRequested()) return;
        Action builtAuto = auto.build();
        //blackboard.put("AutoEndCoordidnates", 2);

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