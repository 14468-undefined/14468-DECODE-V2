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

    private volatile boolean oscillating = false; // flag to control breathing
    private Thread oscillateThread;

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

        RED(0.28),

        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),//light green
        GREEN(0.5),
        UNDEFINEDBLUE(.57),
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

    // --- Start rainbowing LED ---
    public void startOscillating() {
        if (oscillating) return; // already running

        oscillating = true;
        oscillateThread = new Thread(() -> {
            double current = getPose();          // start from current position
            boolean increasing = true;           // direction
            double speed = 0.0048;               // ~6 sec full cycle

            while (oscillating) {
                if (increasing) {
                    current += speed;
                    if (current >= .72) {
                        current = .72;
                        increasing = false;
                    }
                } else {
                    current -= speed;
                    if (current <= 0.28) {
                        current = 0.28;
                        increasing = true;
                    }
                }

                setPoseTest(current);

                try { Thread.sleep(20); } catch (InterruptedException e) {} // ~50 Hz update
            }
        });
        oscillateThread.start();
    }

    // --- Stop rainbowing LED ---
    public void stopOscillating() {
        oscillating = false;
        if (oscillateThread != null) {
            oscillateThread.interrupt();
            oscillateThread = null;
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
