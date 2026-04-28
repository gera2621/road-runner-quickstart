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

@Autonomous(name = "12. Autonomous RED Far PARK", group = "Autonomous OpMode")
public class RedAutoFarPark extends LinearOpMode {
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

        final Pose2d InitPosition = AutoMap.RedFarInitPosition;

        final Pose2d ScorePosition = AutoMap.RedFarScorePosition;

        final Pose2d HumanAlign = AutoMap.RedHumanAlign;

        final Pose2d HumanGrab = AutoMap.RedHumanGrab;

        final Pose2d GPPAlign = AutoMap.RedGPPAlign;

        final Pose2d GPPGrab = AutoMap.RedGPPGrab;

        final Pose2d Park = AutoMap.RedParkFar;

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        scoringSystem.setLaunchVel(0);

        int scoreAngle = 155;

        TrajectoryActionBuilder auto = drivetrain.actionBuilder(InitPosition)
                //Init
                .afterTime(0, ServoGate.closeGateAction())

                //Intake Human
                .strafeToLinearHeading(trunc(HumanAlign), poseAngle(HumanAlign),drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .lineToYSplineHeading(trunc(HumanGrab).y, poseAngle(HumanGrab), drivetrain.defaultVelConstraint, drivetrain.slowAccelConstraint)
                .waitSeconds(intakePause)
                .afterTime(0.5, scoringSystem.intakeAction(0, 0));


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