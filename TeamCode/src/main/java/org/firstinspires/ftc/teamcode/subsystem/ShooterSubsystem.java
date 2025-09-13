package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class ShooterSubsystem extends SubsystemBase {
    //===========MOTORS==========\\
    private final MotorEx shooterRight;
    private final MotorEx shooterLeft;


    //============Servos===========\\
    private final Servo hood;
    private final Servo feeder;

    private final double targetRPM = 6000;  //TODO: Tune this

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public ShooterSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        // ================== MOTORS ================== \\
        this.cTelemetry = telemetry;

        shooterRight = new MotorEx(hardwareMap, "shooterRight", Motor.GoBILDA.BARE);
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft", Motor.GoBILDA.BARE);

        //set to vel control so its constant instead of just power
        shooterRight.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterLeft.setRunMode(MotorEx.RunMode.VelocityControl);

        //reverse
        shooterRight.setInverted(false);
        shooterLeft.setInverted(false);



        // ================== SERVOS ================== \\
        hood = hardwareMap.get(Servo.class, "hood");
        feeder = hardwareMap.get(Servo.class, "feeder");

        hood.scaleRange(0, 1);
        feeder.scaleRange(0, 1);

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


        // Display both motors
        t.addData("Target RPM",targetRPM);
        t.addData("Right RPM", tpstoRPM(shooterRight.getVelocity()));
        t.addData("Left RPM", tpstoRPM(shooterLeft.getVelocity()));



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
    private static final double TICKS_PER_REV = 28; //TODO: Tune
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
    private double tpstoRPM(double tps){
        return (tps * 60) / TICKS_PER_REV;
    }
}
