package org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop;

import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionRed.RobotState.INTAKE;
import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionRed.RobotState.PRESCORE;
import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionRed.RobotState.SCORE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.team.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.team.internalLib.AuxiliaryLocalizationSystem;

import java.util.Locale;

@TeleOp(name = "IMU Reset", group = "Linear OpMode")
public class IMUReset extends LinearOpMode {
    private String infoIMU = "";
    public GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(55,70, DistanceUnit.MM);

        waitForStart();
        if (isStopRequested()) return;

        pinpoint.recalibrateIMU();

        sleep(500);
        //if below doesn't work and sets the bot to 0, 0 replace ResetPosAndIMU with recalibrateIMU()
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0,0, AngleUnit.DEGREES, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
        telemetry.update();

        telemetry.setMsTransmissionInterval(11);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

            Pose2D pos = pinpoint.getPosition();

            dashboardTelemetry.update();
        }
    }