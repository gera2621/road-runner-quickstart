package org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop;

import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionBlue.RobotState.INTAKE;
import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionBlue.RobotState.PRESCORE;
import static org.firstinspires.ftc.teamcode.team.opmodes.competition.teleop.TeleOpCompetitionBlue.RobotState.SCORE;

//adb connect 192.168.43.1:5555

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

@TeleOp(name = "1. TeleOp BLUE", group = "Linear OpMode")
public class TeleOpCompetitionBlue extends LinearOpMode {
    private String infoIMU = "";
    private Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(55,70, DistanceUnit.MM);

        pinpoint.recalibrateIMU();

        sleep(500);

        pinpoint.setPosition(AuxiliaryLocalizationSystem.ConvertRRPoseToDriverPose((Pose2d) blackboard.get("BotPoseRR")));

        Pose2D TargetPose = new Pose2D(DistanceUnit.MM,1800,1800,AngleUnit.DEGREES,0.0);
        Pose2D InitPose = new Pose2D(DistanceUnit.MM,-752.313,1360.717,AngleUnit.DEGREES,90);

        int SmallManualSpeedAdjustment = 5;
        int ManualSpeedAdjustment = 25;

        double oldTime = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();
        if (isStopRequested()) return;

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.
         */
        limelight.start();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        RobotState robotState = INTAKE;

        boolean precision = false;

        //ElapsedTime timer = new ElapsedTime();
        //timer.reset();

        MecanumDrive drivetrain = new MecanumDrive(
                hardwareMap.dcMotor.get("leftFront"),
                hardwareMap.dcMotor.get("leftBack"),
                hardwareMap.dcMotor.get("rightFront"),
                hardwareMap.dcMotor.get("rightBack"),
                hardwareMap.get(IMU.class, "imu")
        );

        ScoringSystem scoringsystem = new ScoringSystem(
                (DcMotorEx) hardwareMap.dcMotor.get("launcher"),
                (DcMotorEx) hardwareMap.dcMotor.get("intake"),
                (DcMotorEx) hardwareMap.dcMotor.get("turret"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcher2")
        );

        double last_tx_value = 0;
        boolean last_was_valid = false;
        double last_detection = getRuntime();
        double detection_start = getRuntime();
        ServoGate ServoGate = new ServoGate(
                hardwareMap.servo.get("gate")
        );

        ServoGate.closeGate();

        GamepadButton stateBack = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.LEFT_BUMPER);
        GamepadButton stateForward = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);
        GamepadButton UpAccel = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_UP);
        GamepadButton RightAccelSmol = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_RIGHT);
        GamepadButton DownDecel = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_DOWN);
        GamepadButton LeftDecelSmol = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_LEFT);
        GamepadButton ManualSpeedToggle = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.triangle);
        GamepadButton ManualTurretToggle = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.circle);
        GamepadButton PinpointReset = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.START);
        GamepadButton TargetReset = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.SHARE);
        GamepadButton SlowLaunchOnButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.X);

        boolean ManualSpeedOn = false;
        boolean ManualTurretOn = false;

        infoIMU += "IMU - " + pinpoint.getDeviceName();
        infoIMU += " | ID: " + pinpoint.getDeviceID();
        infoIMU += " | ver" + pinpoint.getDeviceVersion();
        infoIMU += " | yawS: " + pinpoint.getYawScalar();

        while (opModeIsActive()) {

            pinpoint.update();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
            double tx_value = result.getTx();
            double target = 0;

            //vision based targeting j


//            if (!result.isValid() && ((last_detection - detection_start) > 0)) {
//                //
//                target = ((getRuntime() - last_detection) <= 0.25) ? last_tx_value : 0;
//                last_was_valid = false;
//            } else if (result.isValid()) {
//                target = tx_value;
//
//                if (!last_was_valid) {
//                    detection_start = getRuntime();
//                }
//                last_detection = getRuntime();
//                last_was_valid = true;
//                last_tx_value = tx_value;
//            } else {
//                target = 0;
//            }
            target = AuxiliaryLocalizationSystem.getAngle(pinpoint.getPosition(), TargetPose);

            switch (robotState) {
                case INTAKE:
                    ServoGate.closeGate();
                    drivetrain.zeroPowerFloat();

                    scoringsystem.launcherIdle();

                    drivetrain.botOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

                    scoringsystem.setTurretTarget(0,2000);

                    if (stateForward.isPressed()) {
                        robotState = PRESCORE;
                    }
                    break;

                case PRESCORE:
                    ServoGate.closeGate();
                    drivetrain.zeroPowerFloat();

                    if(!ManualSpeedOn){
                        scoringsystem.setLaunchVel( (int)  ScoringSystem.TurretDistToFlywheelVelocity(AuxiliaryLocalizationSystem.getDistance(pinpoint.getPosition(), TargetPose)));

                        //up = decrease y
                        //down = increase y
                        //left = increase x
                        //right = decrease x
                        if (UpAccel.isPressed()) {
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM),TargetPose.getY(DistanceUnit.MM)+50,AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (RightAccelSmol.isPressed()){
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM)+50,TargetPose.getY(DistanceUnit.MM),AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (DownDecel.isPressed()) {
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM),TargetPose.getY(DistanceUnit.MM)-50,AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (LeftDecelSmol.isPressed()){
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM)-50,TargetPose.getY(DistanceUnit.MM),AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }

                    } else {
                        if (UpAccel.isPressed()) {
                            scoringsystem.launchVelAdjust(ManualSpeedAdjustment);
                        }
                        if (RightAccelSmol.isPressed()){
                            scoringsystem.launchVelAdjust(SmallManualSpeedAdjustment);
                        }
                        if (DownDecel.isPressed()) {
                            scoringsystem.launchVelAdjust(-ManualSpeedAdjustment);
                        }
                        if (LeftDecelSmol.isPressed()){
                            scoringsystem.launchVelAdjust(-SmallManualSpeedAdjustment);
                        }
                        scoringsystem.setLaunchVel(1365);
                    }

                    scoringsystem.launcherUpdate();

                    drivetrain.botOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

                    if(ManualTurretOn){
                        scoringsystem.setTurretTarget(0,2000);
                    }else{
                        scoringsystem.setTurretTarget(target,2000);
                    }

                    if (stateForward.isPressed()) {
                        robotState = SCORE;
                    }
                    if (stateBack.isPressed()) {
                        robotState = INTAKE;
                    }

                    break;

                case SCORE:
                    ServoGate.openGate();
                    drivetrain.zeroPowerBrake();


                    if(!ManualSpeedOn){
                        scoringsystem.setLaunchVel( (int)  ScoringSystem.TurretDistToFlywheelVelocity(AuxiliaryLocalizationSystem.getDistance(pinpoint.getPosition(), TargetPose)));

                        //up = increase y
                        //down = decrease y
                        //left = increase x
                        //right = decrease x
                        if (UpAccel.isPressed()) {
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM),TargetPose.getY(DistanceUnit.MM)+50,AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (RightAccelSmol.isPressed()){
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM)-50,TargetPose.getY(DistanceUnit.MM),AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (DownDecel.isPressed()) {
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM),TargetPose.getY(DistanceUnit.MM)-50,AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }
                        if (LeftDecelSmol.isPressed()){
                            TargetPose = new Pose2D(DistanceUnit.MM, TargetPose.getX(DistanceUnit.MM)+50,TargetPose.getY(DistanceUnit.MM),AngleUnit.DEGREES, TargetPose.getHeading(AngleUnit.DEGREES));
                        }

                    } else {
                        if (UpAccel.isPressed()) {
                            scoringsystem.launchVelAdjust(ManualSpeedAdjustment);
                        }
                        if (RightAccelSmol.isPressed()){
                            scoringsystem.launchVelAdjust(SmallManualSpeedAdjustment);
                        }
                        if (DownDecel.isPressed()) {
                            scoringsystem.launchVelAdjust(-ManualSpeedAdjustment);
                        }
                        if (LeftDecelSmol.isPressed()){
                            scoringsystem.launchVelAdjust(-SmallManualSpeedAdjustment);
                        }
                        scoringsystem.setLaunchVel(1365);
                    }

                    scoringsystem.launcherUpdate();

                    drivetrain.botOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

                    if(ManualTurretOn){
                        scoringsystem.setTurretTarget(0,2000);
                    }else{
                        scoringsystem.setTurretTarget(target,2000);
                    }

                    if (stateForward.isPressed()) {
                        robotState = INTAKE;
                    }

                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + robotState);
            }

            if(ManualSpeedToggle.isPressed()){
                ManualSpeedOn = !ManualSpeedOn;
            }
            if(ManualTurretToggle.isPressed()){
                ManualTurretOn = !ManualTurretOn;
            }
            scoringsystem.intake(gamepad1.left_trigger, gamepad1.right_trigger);

            if(TargetReset.isPressed()){
                TargetPose = new Pose2D(DistanceUnit.MM,1800,1800,AngleUnit.DEGREES,0.0);
            }

            if (PinpointReset.isPressed()) {
                pinpoint.recalibrateIMU();
                sleep(500);
                pinpoint.setPosition(new Pose2D(DistanceUnit.MM,-1494.382,-1349.533,AngleUnit.DEGREES,-90));
            }

            Pose2D pos = pinpoint.getPosition();

            telemetry.addData("BotInitPoseRR",(Pose2d) blackboard.get("BotPoseRR"));
            telemetry.addData("BotInitPoseConverted", AuxiliaryLocalizationSystem.ConvertRRPoseToDriverPose((Pose2d) blackboard.get("BotPoseRR")));

            telemetry.addData("LimelightResultState", result == null ? "null" : (result.isValid() ? "Valid" : "Invalid"));

            telemetry.addData("RobotState", robotState);
            telemetry.addData("RobotIsInPreciseMode", precision);

            telemetry.addData("Front Left Motor Power: ", drivetrain.getFrontLeftPower());
            telemetry.addData("Back Left Motor Power: ", drivetrain.getBackLeftPower());
            telemetry.addData("Front Right Motor Power: ", drivetrain.getFrontRightPower());
            telemetry.addData("Back Right Motor Power: ", drivetrain.getBackRightPower());

            telemetry.addData("Intake Motor Velocity: ", scoringsystem.getIntakeVel());
            telemetry.addData("Launcher Motor Velocity ", scoringsystem.getLauncherVel());

            telemetry.addData("Launcher Motor TargetPose Vel: ", scoringsystem.LaunchVel);

            //telemetry.addData("Launcher Motor Multiplier: ", ScoringSystem.LaunchMult);

            telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
            telemetry.addData("Left Trigger: ", gamepad1.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad1.right_trigger);

            telemetry.addData("Turret Positon: ", scoringsystem.getTurretPos());
            telemetry.addData("Turret TargetPose Position", scoringsystem.getTurretTargetPos());
            telemetry.addData("Robot Heading", Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS)));

            telemetry.addData("IMU Status", pinpoint.getDeviceStatus());
            telemetry.addData(" IMU Info", infoIMU);
            telemetry.addData("Camera Calculated Distance", AuxiliaryLocalizationSystem.getDistancefromAngle(result.getTy(), 750));
            telemetry.addData("OdoCalculatedDistance", AuxiliaryLocalizationSystem.getDistance(pinpoint.getPosition(), TargetPose));
            telemetry.update();

            dashboardTelemetry.addData("Front Left Motor Power: ", drivetrain.getFrontLeftPower());
            dashboardTelemetry.addData("Back Left Motor Power: ", drivetrain.getBackLeftPower());
            dashboardTelemetry.addData("Front Right Motor Power: ", drivetrain.getFrontRightPower());
            dashboardTelemetry.addData("Back Right Motor Power: ", drivetrain.getBackRightPower());

            dashboardTelemetry.addData("Intake Motor Velocity: ", scoringsystem.getIntakeVel());
            dashboardTelemetry.addData("Launcher Motor Velocity ", scoringsystem.getLauncherVel());

            dashboardTelemetry.addData("Launcher Motor TargetPose Vel: ", scoringsystem.LaunchVel);

            dashboardTelemetry.addData("Turret Position: ", scoringsystem.getTurretPos());

            dashboardTelemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
            dashboardTelemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            dashboardTelemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
            dashboardTelemetry.addData("Left Trigger: ", gamepad1.left_trigger);
            dashboardTelemetry.addData("Right Trigger: ", gamepad1.right_trigger);

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("Position", data);
            telemetry.addData("Position", data);

            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", pinpoint.getVelX(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            dashboardTelemetry.addData("Velocity", velocity);
            telemetry.addData("Velocity", velocity);

            telemetry.addData("Status", pinpoint.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            dashboardTelemetry.update();
        }
    }
    enum RobotState {
        INTAKE,
        PRESCORE,
        PRIME,
        SCORE
    }
}