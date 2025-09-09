package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {
    private final MotorEx topMotor;
    private final MotorEx bottomMotor;
    private final Telemetry telemetry;
    private final double targetRPM = 2500;  //TODO: Tune this

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        topMotor = new MotorEx(hardwareMap, "topShooter");
        bottomMotor = new MotorEx(hardwareMap, "bottomShooter");

        //set to vel control so its constant instead of just power
        topMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        bottomMotor.setRunMode(MotorEx.RunMode.VelocityControl);

        //reverse one
        bottomMotor.setInverted(true);
    }

    // Spin both wheels up to speed
    public void spinUp() {
        double velocity = rpmToTicksPerSecond(targetRPM);
        topMotor.setVelocity(velocity);
        bottomMotor.setVelocity(velocity);
    }

    // Stop both wheels
    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    //make sure they are close to right speed
    public boolean atSpeed() {
        return Math.abs(topMotor.getVelocity() - rpmToTicksPerSecond(targetRPM)) < 50 &&
                Math.abs(bottomMotor.getVelocity() - rpmToTicksPerSecond(targetRPM)) < 50;
    }

    @Override
    public void periodic() {
        telemetry.addData("SHOOTER SUBSYSTEM", "");
        telemetry.addData("Top Velocity (tps)", topMotor.getVelocity());
        telemetry.addData("Bottom Velocity (tps)", bottomMotor.getVelocity());
        telemetry.addData("Target (tps)", rpmToTicksPerSecond(targetRPM));
        telemetry.addData("At Speed?", atSpeed());
        telemetry.update();
    }

    // convert rpm to tps
    private static final double TICKS_PER_REV = 537.7; //TODO: Tune
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
