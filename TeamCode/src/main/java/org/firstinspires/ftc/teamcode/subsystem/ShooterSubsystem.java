package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx shooter;

    private final double targetRPM = 6000;  //TODO: Tune this

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public ShooterSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

        shooter = new MotorEx(hardwareMap, "shooter");


        //set to vel control so its constant instead of just power
        shooter.setRunMode(MotorEx.RunMode.VelocityControl);

        //reverse
        shooter.setInverted(false);
    }

    // Spin up to speed
    public void spinUp() {
        double velocity = rpmToTicksPerSecond(targetRPM);
        shooter.setVelocity(velocity);

    }

    public void spinUpReverse(){
        double velocity = rpmToTicksPerSecond(targetRPM);
        shooter.setVelocity(-velocity);
    }

    // Stop both wheels
    public void stop() {
        shooter.set(0);

    }

    //make sure they are close to right speed
    public boolean atSpeed() {
        return Math.abs(shooter.getVelocity() - rpmToTicksPerSecond(targetRPM)) < 50;
    }

    public void printTelemetry(ColorfulTelemetry t) {

        t.reset(); // reset any previous styles


        t.setColor(ColorfulTelemetry.Black).bold();
        t.addLine("SHOOTER SUBSYSTEM");  // header
        t.reset();
        t.addData("Velocity (tps)", shooter.getVelocity());

        t.addData("Target (tps)", rpmToTicksPerSecond(targetRPM));

        //set green if at speed
        if (atSpeed()) {
            t.setColor(ColorfulTelemetry.Green);
        } else {
            t.setColor(ColorfulTelemetry.Red);
        }
        t.addData("At Speed?", atSpeed());


        t.update();

    }

    @Override
    public void periodic() {




    }

    // convert rpm to tps
    private static final double TICKS_PER_REV = 537.7; //TODO: Tune
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
