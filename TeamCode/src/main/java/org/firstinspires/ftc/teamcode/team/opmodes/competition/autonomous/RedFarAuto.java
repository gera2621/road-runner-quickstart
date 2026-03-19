package org.firstinspires.ftc.teamcode.team.opmodes.competition.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
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
public class RedFarAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int tinyPause = 200;
        int littlePause = 250;
        int bigPause = 500;
        int scorePause = 1250;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        Pose2d InitPosition = new Pose2d(60, 14.5, 90);

        Vector2d ParkPos = new Vector2d(56, 14.5);
        Pose2d ParkPose = new Pose2d(56, 14.5, Math.toRadians(90));


        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        TrajectoryActionBuilder moveToPark = drivetrain.actionBuilder(InitPosition)
                .strafeToLinearHeading(ParkPos, Math.toRadians(90));



        waitForStart();
        if (isStopRequested()) return;

        ScoringSystem ScoringSystem = new ScoringSystem(
                (DcMotorEx) hardwareMap.dcMotor.get("launcher"),
                (DcMotorEx) hardwareMap.dcMotor.get("intake"),
                (DcMotorEx) hardwareMap.dcMotor.get("turret"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcher2")
        );
        ServoGate ServoGate = new ServoGate(
                hardwareMap.servo.get("gate")
        );

        ServoGate.closeGate();
        ScoringSystem.intake(0,1);

        Actions.runBlocking(new SequentialAction(moveToPark.build()));

        wait(500);

        ScoringSystem.intake(0,0);

        while(opModeIsActive()) {
            telemetry.addData("Intake Motor Velocity: ", ScoringSystem.getIntakeVel());
            telemetry.addData("Launcher Motor Velocity ", ScoringSystem.getLauncherVel());

            telemetry.addData("Launcher Motor Target Vel: ", ScoringSystem.LaunchVel);

            dashboardTelemetry.addData("Intake Motor Velocity: ", ScoringSystem.getIntakeVel());
            dashboardTelemetry.addData("Launcher Motor Velocity ", ScoringSystem.getLauncherVel());

            dashboardTelemetry.addData("Launcher Motor Target Vel: ", ScoringSystem.LaunchVel);

            dashboardTelemetry.update();
        }
    }
}