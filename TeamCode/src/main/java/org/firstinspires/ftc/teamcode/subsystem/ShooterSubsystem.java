package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

import com.acmerobotics.dashboard.config.Config;


@Config
public class ShooterSubsystem extends UndefinedSubsystemBase {
    //=========== MOTORS ===========\\
    private final MotorEx shooterRight;
    private final MotorEx shooterLeft;

    //=========== PIDF CONTROL ===========\\
    private final PIDFController shooterPID;
    private double targetRPM = 0;
    private double currentRPM = 0;

    // PIDF constants (tune these)
    public static double kP = 0.0008;
    public static double kI = 0.0001;
    public static double kD = 0.0001;
    public static double kF = 0.0002;

    private final double TICKS_PER_REV = 28.0; //for 6k rpm

    //=========== TELEMETRY ===========\\
    private final ColorfulTelemetry cTelemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

        shooterRight = new MotorEx(hardwareMap, "shooterRight", Motor.GoBILDA.BARE);
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft", Motor.GoBILDA.BARE);

        shooterLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        shooterRight.setInverted(false);
        shooterLeft.setInverted(true);

        // Initialize PIDF
        shooterPID = new PIDFController(kP, kI, kD, kF);
        shooterPID.setTolerance(50);
    }

    //============== CONTROL METHODS ==============\\

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    // called repeatedly to spin up and regulate shooter speed

    public void spinUpChooseRPM(int rpm){
        shooterPID.setPIDF(kP, kI, kD, kF);
        //get current velocity
        currentRPM = ((shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0) * (60.0 / TICKS_PER_REV);

        //compute feedforward
        double ff = kF * rpm;
        double pidOutput = shooterPID.calculate(currentRPM, rpm);

        // 0<power<1
        double totalOutput = Range.clip(ff + pidOutput, 0, 1);

        //set power
        shooterLeft.set(totalOutput);
        shooterRight.set(totalOutput);
    }
    public void spinUp() {

        shooterPID.setPIDF(kP, kI, kD, kF);
        //get current velocity
        currentRPM = ((shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0) * (60.0 / TICKS_PER_REV);

        //compute feedforward
        double ff = kF * targetRPM;
        double pidOutput = shooterPID.calculate(currentRPM, targetRPM);

        // 0<power<1
        double totalOutput = Range.clip(ff + pidOutput, 0, 1);

        //set power
        shooterLeft.set(totalOutput);
        shooterRight.set(totalOutput);
    }

    public void stop() {

        shooterLeft.set(0);
        shooterRight.set(0);
    }

    public void spinSlowReverse(){
        shooterLeft.set(-.6);
        shooterRight.set(-.6);
    }

    public boolean atSpeed() {
        double avgVelocity = (shooterRight.getVelocity() + shooterLeft.getVelocity()) / 2.0;
        double targetVelocity = rpmToTicksPerSecond(targetRPM);
        return Math.abs(avgVelocity - targetVelocity) < 100;
    }

    //============== UTIL ==============\\

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double tpsToRPM(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    @Override
    public void periodic() {

    }

    public void printTelemetry(ColorfulTelemetry t) {
        t.reset();
        t.addLine("SHOOTER SUBSYSTEM");
        t.addData("Target RPM", targetRPM);
        t.addData("Current RPM", currentRPM);
        t.addData("Right RPM", tpsToRPM(shooterRight.getVelocity()));
        t.addData("Left RPM", tpsToRPM(shooterLeft.getVelocity()));
        t.addData("At Speed?", atSpeed());
        t.update();
    }
}
