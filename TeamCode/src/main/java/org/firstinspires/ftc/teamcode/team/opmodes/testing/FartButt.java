//package org.firstinspires.ftc.teamcode.team.opmodes.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
//import org.firstinspires.ftc.teamcode.team.subsystems.MecanumDrive;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpDrivetrain", group = "Linear OpMode")
//public class FartButt extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        DcMotor motor = hardwareMap.get(DcMotor.class, "shooter");
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Servo servo = hardwareMap.get(Servo.class, "hood");
//        servo.setPosition(0);
//
//        while (opModeIsActive())
//        {
//            motor.setPower(-0.6);
//        }
//    }
//}