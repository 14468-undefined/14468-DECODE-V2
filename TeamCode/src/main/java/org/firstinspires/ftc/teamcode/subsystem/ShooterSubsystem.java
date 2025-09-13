package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx shooterRight;
    private final MotorEx shooterLeft;

    private final double targetRPM = 6000;  //TODO: Tune this

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public ShooterSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

        shooterRight = new MotorEx(hardwareMap, "shooterRight");
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft");

        //set to vel control so its constant instead of just power
        shooterRight.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterLeft.setRunMode(MotorEx.RunMode.VelocityControl);

        //reverse
        shooterRight.setInverted(false);
        shooterLeft.setInverted(false);
    }

    // Spin up to speed
    public void spinUp() {
        double velocity = rpmToTicksPerSecond(targetRPM);
        shooterRight.setVelocity(velocity);
        shooterLeft.setVelocity(velocity);

    }

    public void spinUpReverse(){
        double velocity = rpmToTicksPerSecond(targetRPM);
        shooterRight.setVelocity(-velocity);
        shooterLeft.setVelocity(-velocity);
    }

    public void shoot(){
        //shoot with mechanism to push into flywheel
    }

    // Stop both wheels
    public void stop() {
        shooterRight.setVelocity(0);
        shooterLeft.setVelocity(0);

    }

    //make sure they are close to right speed
    public boolean atSpeed() {
        double velocity = rpmToTicksPerSecond(targetRPM);
        double avgVelocity = (shooterRight.getVelocity() + shooterLeft.getVelocity()) / 2.0;
        return Math.abs(avgVelocity - velocity) < 100;
    }

    public void printTelemetry(ColorfulTelemetry t) {

        t.reset(); // reset any previous styles


        t.setColor(ColorfulTelemetry.Black).bold();
        t.addLine("SHOOTER SUBSYSTEM");  // header
        t.reset();
        t.addData("RightVelocity (tps)", shooterRight.getVelocity());
        t.addData("leftVelocity (tps)", shooterLeft.getVelocity());

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
