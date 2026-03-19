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

@Autonomous(name = "3. Autonomous BLUE Far", group = "Autonomous OpMode")
public class BlueAutoFar extends LinearOpMode {
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

        Pose2d InitPosition = new Pose2d(0, 0, 0);

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, InitPosition);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {

            blackboard.put("BotPoseRR", drivetrain.localizer.getPose());

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