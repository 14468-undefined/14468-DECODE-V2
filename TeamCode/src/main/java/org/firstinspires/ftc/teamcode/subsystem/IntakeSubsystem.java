package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final MotorEx intakeMotor;

    private final double power = 0.5;//TODO: change

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    public IntakeSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

        intakeMotor = new MotorEx(hardwareMap, "intake");


        //power instead of vel control
        intakeMotor.setRunMode(MotorEx.RunMode.RawPower);
        intakeMotor.setInverted(false);//run forwards

    }


    public void intake() {
        intakeMotor.set(power);
    }

    //send balls out through intake
    public void intakeReverse() {
        intakeMotor.set(-power);
    }


    public void stop() {
        intakeMotor.set(0);
    }

    public void printTelemetry(ColorfulTelemetry t) {
        t.reset(); // reset any previous styles


        t.setColor(ColorfulTelemetry.Blue).bold().addLine("INTAKE SUBSYSTEM").reset();

        String status;
        if (intakeMotor.get() > 0) {
            status = "Intaking";
            t.setColor(ColorfulTelemetry.Green);
        } else if (intakeMotor.get() < 0) {
            status = "Reversing";
            t.setColor(ColorfulTelemetry.Orange);
        } else {
            status = "Stopped";
            t.setColor(ColorfulTelemetry.Red);
        }
        t.addData("Status", status);


        t.update();


        //return power
        t.addData("Motor Power", intakeMotor.get());
        t.update();

    }

    @Override
    public void periodic() {




    }

}