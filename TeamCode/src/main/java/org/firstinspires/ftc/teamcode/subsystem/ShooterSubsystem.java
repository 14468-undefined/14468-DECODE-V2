package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx topMotor;
    private final MotorEx bottomMotor;

    private final double targetRPM = 2500;  //TODO: Tune this

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public ShooterSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

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


            cTelemetry.reset(); // reset any previous styles

            cTelemetry
                    .setColor(ColorfulTelemetry.Black).bold()
                    .addLine("SHOOTER SUBSYSTEM")  // header
                    .reset()
                    .addData("Top Velocity (tps)", topMotor.getVelocity())
                    .addData("Bottom Velocity (tps)", bottomMotor.getVelocity())
                    .addData("Target (tps)", rpmToTicksPerSecond(targetRPM));

            //set green if at speed
            if (atSpeed()) {
                cTelemetry.setColor(ColorfulTelemetry.Green);
            } else {
                cTelemetry.setColor(ColorfulTelemetry.Red);
            }
            cTelemetry.addData("At Speed?", atSpeed());


            cTelemetry.update();

    }

    // convert rpm to tps
    private static final double TICKS_PER_REV = 537.7; //TODO: Tune
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
