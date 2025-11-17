package org.firstinspires.ftc.teamcode.subsystem;




import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LEDSubsystem extends SubsystemBase {

    /*
    this controls the goBilda PWM indicator lights - there's a few methods for different colors
     */



    private final Servo LED1;


    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public LEDSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        // ================== MOTORS ================== \\
        this.cTelemetry = telemetry;



        LED1 = hardwareMap.get(Servo.class, "hood");


        double currentPose = LED1.getPosition();


    }

    public void setColor(LEDColor color){

        LED1.setPosition(color.position);
    }

    public void setPoseTest(double pose){
        LED1.setPosition(pose);
    }

    public enum LEDColor {
        OFF(0.0),

        RED(0.277),

        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),//light green
        GREEN(0.5),
        AZURE(0.555),//blue green
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0);


        public final double position;

        LEDColor(double position) {
            this.position = position;
        }
    }

    public double getPose(){
        return LED1.getPosition();
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


}
