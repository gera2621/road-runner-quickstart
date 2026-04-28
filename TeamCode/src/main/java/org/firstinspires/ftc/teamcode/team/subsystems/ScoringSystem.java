package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.teamcode.team.libraries.PIDController;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.config.Config;

import java.util.List;


public class ScoringSystem {
    private final DcMotorEx launcher;
    //private final DcMotorEx intake2;
    private final DcMotorEx intake;
    //private final VoltageSensor voltageSensor;
    private final DcMotorEx turret;

    private final DcMotorEx launcher2;

    @Config
    public static class intakePIDF {
        public static double P = 16;
        public static double I = 0;
        public static double D = 0;
        public static double F = 12;
    }
    @Config
    public static class launcherPIDF {
        public static double P = 14.5;
        public static double I = 0.00012; //0.00008
        public static double D = 0.00000;
        public static double F = 0;
    }
    @Config
    public static class turretPIDF {
        public static double P = 12;

        public static double I = 0.5;
        public static double D = 0.5;
        public static double F = 24;
    }
    public static launcherPIDF LauncherPIDF = new launcherPIDF();
    public static intakePIDF IntakePIDF = new intakePIDF();
    public static turretPIDF TurretPIDF = new turretPIDF();

    public static PIDController launcherPID = new PIDController(launcherPIDF.P, launcherPIDF.I, launcherPIDF.D);


    public double LaunchVel = 1240;

    public ScoringSystem(DcMotorEx launcher, DcMotorEx intake, DcMotorEx turret, DcMotorEx launcher2) {
        this.launcher = launcher;
        this.launcher2 = launcher2;
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherPID.reset();

        launcherPID.withIntegralRange(50);

        //this.voltageSensor = voltageSensor;

        this.intake = intake;

        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setVelocityPIDFCoefficients(intakePIDF.P, intakePIDF.I, intakePIDF.D, intakePIDF.F);

        this.turret = turret;
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setTargetPositionTolerance(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setPower(1);
        turret.setPositionPIDFCoefficients(10);
        turret.setVelocityPIDFCoefficients(turretPIDF.P,turretPIDF.I,turretPIDF.D,turretPIDF.F);
    }

    public void launcherOff(){
        launcher.setVelocity(0);
        launcher2.setPower(launcher.getPower());
    }

    public void launcherIdle(){
        launcher.setVelocity(750);
        launcher.setVelocityPIDFCoefficients(0,0,0,16);
        launcher2.setPower(launcher.getPower());
    }

    public Action launcherOffAction(){
        return new InstantAction(
                this::launcherOff
        );
    }
    public void launcherUpdate(){
        double power = launcherPID.update(LaunchVel,launcher.getVelocity());
        launcher.setVelocity(power);
        launcher2.setPower(power);
    }

    public Action launcherUpdateAction(){
        return new InstantAction(
                this::launcherUpdate
        );
    }
    public static double TurretDistToFlywheelVelocity (double distance) {
        return 0.95*(0.00000266667*Math.pow((distance), 2)+ 0.240435*(distance) + 900.28771); //1.0325
    }
    //Used Odometry Distance

    public void launchVelAdjust(int input){
        LaunchVel += input;
    }

    public void setLaunchVel(int velocity){
        LaunchVel = velocity;
    }
    public Action setLaunchVelAction(int velocity){
        return new InstantAction(
                () -> this.setLaunchVel(velocity)
        );
    }

    public void intake(double out, double in){
        intake.setVelocityPIDFCoefficients(intakePIDF.P, intakePIDF.I, intakePIDF.D, intakePIDF.F);
        //intake2.setVelocityPIDFCoefficients(intakePIDF.P, intakePIDF.I, intakePIDF.D, intakePIDF.F);
        intake.setVelocity((2800*out)-(3600*in));
    }

    public Action intakeAction(double out, double in) {
        return new InstantAction(
                () -> this.intake(out, in)
        );
    }

    public void setTurretTarget(double inputDegrees, double totalTicks) {
        double ticksPerDegree = totalTicks / 360.0;
        double targetTicks = inputDegrees * ticksPerDegree;
        double limit = 0.30 * totalTicks;
        double clampedPosition = Math.max(-limit, Math.min(targetTicks, limit));
        turret.setTargetPosition((int) clampedPosition);
    }


    public double getTurretPos() {
        return turret.getCurrentPosition();
    }
    public double getTurretTargetPos(){
        return turret.getTargetPosition();
    }
    public double getTurretPower(){
        return turret.getPower();
    }
    public double getIntakeVel() {
        return intake.getVelocity();
    }
    public double getLauncherVel() {
        return launcher.getVelocity();
    }
}
