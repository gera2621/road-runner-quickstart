package org.firstinspires.ftc.teamcode.team.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.team.libraries.SlewRateLimiter;

public class MecanumDrive {
    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightFront;
    public final DcMotorEx rightBack;
    public final IMU imu;

    public MecanumDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, IMU imu) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.imu = imu;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB
        // forward
        imu.initialize(parameters);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public SlewRateLimiter leftFrontSlew = new SlewRateLimiter(4);
    public SlewRateLimiter leftBackSlew = new SlewRateLimiter(4);
    public SlewRateLimiter rightFrontSlew = new SlewRateLimiter(4);
    public SlewRateLimiter rightBackSlew = new SlewRateLimiter(4);

    public void resetIMU() {
        imu.resetYaw();
    }
    // This button choice was made so that it is hard to hit on accident,
    // it can be freely changed based on preference.
    // The equivalent button is start on Xbox-style controllers.

    public void botOrientedDrive(double x, double y, double rx, double sp) {


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rx) + Math.abs(x) + Math.abs(y), 1);
        double frontLeftPower = ((-y + x + rx) / denominator) * (1 - (0.6 * sp));
        double backLeftPower = ((-y - x + rx) / denominator) * (1 - (0.6 * sp));
        double frontRightPower = ((-y - x - rx) / denominator) * (1 - (0.6 * sp));
        double backRightPower = ((-y + x - rx) / denominator) * (1 - (0.6 * sp));

        leftFront.setPower(MecanumPowerCurve(frontLeftPower));
        leftBack.setPower(MecanumPowerCurve(backLeftPower));
        rightFront.setPower(MecanumPowerCurve(frontRightPower));
        rightBack.setPower(MecanumPowerCurve(backRightPower));
    }

    public double MecanumPowerCurve (double input){
        return 0.538721*Math.pow(input,3) + 0.46936*input;
    }

    public void zeroPowerBrake() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void zeroPowerFloat() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getFrontLeftPower() {
        return leftFront.getPower();
    }
    public double getBackLeftPower() {
        return leftBack.getPower();
    }
    public double getFrontRightPower() {
        return rightFront.getPower();
    }
    public double getBackRightPower() {
        return rightBack.getPower();
    }
}
