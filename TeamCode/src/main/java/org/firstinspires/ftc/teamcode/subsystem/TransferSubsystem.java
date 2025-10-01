package org.firstinspires.ftc.teamcode.subsystem;




import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TransferSubsystem extends SubsystemBase {




    private final CRServo transfer;
    double transferPower = 0;

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public TransferSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        // ================== MOTORS ================== \\
        this.cTelemetry = telemetry;

        transfer = hardwareMap.get(CRServo.class, "transfer");



    }


    public void spin(){
        transfer.setPower(Constants.transferConstants.SPIN_POWER);
    }
    public void spinReverse(){
        transfer.setPower(Constants.transferConstants.SPIN_REVERSE_POWER);
    }

    public void stop(){
        transfer.setPower(0);
    }
    public void printTelemetry(ColorfulTelemetry t) {

        t.reset(); // reset any previous styles


        t.setColor(ColorfulTelemetry.Black).bold();
        t.addLine("INTAKE SUBSYSTEM");






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
