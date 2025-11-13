package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.UndefinedSubsystemBase;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class HuskyLensSubsystem extends UndefinedSubsystemBase {

    private final HuskyLens huskyLens;
    private boolean active = true;


    private Servo rotation;


    public enum Mode { ARTIFACT, APRILTAG }
    private Mode mode = Mode.ARTIFACT;


    private double turnPower = 0;
    private double servoPosition = 0.85; // start high


    private static final int SCREEN_CENTER_X = 160;
    private static final int TARGET_CENTER_Y = 140;
    private static final double kP_TURN_ARTIFACT = 0.004;
    private static final double kP_TURN_APRILTAG = 0.01;

    private static final double SERVO_MIN = 0.15;
    private static final double SERVO_MAX = 0.85;

    public HuskyLensSubsystem(HardwareMap hardwareMap, ColorfulTelemetry t) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        rotation = hardwareMap.get(Servo.class, "hood");
        rotation.scaleRange(Constants.WebcamConstants.ROTATION_MIN, Constants.WebcamConstants.ROTATION_MAX);
        rotation.setPosition(Constants.WebcamConstants.ROTATION_GOAL);

    }

    /** Check for a tag, deactivate after found */
    public HuskyLens.Block detectTag(int targetID) {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        HuskyLens.Block target = null;

        while (target == null) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks != null && blocks.length > 0) {
                for (HuskyLens.Block b : blocks) {
                    if (b.id == targetID) {
                        target = b;
                        break;
                    }
                }
            }

            //share CPu
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        return target;
    }


    public void enable() {
        active = true;
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION); // re-enable processing
    }

    public void disable() {
        active = false;

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.NONE);
        huskyLens.close();

    }

    //change mode
    public void setModeArtifact() { mode = Mode.ARTIFACT; }
    public void setModeAprilTag() { mode = Mode.APRILTAG; }
    public Mode getMode() { return mode; }

    //accessors
    public double getTurnPower() { return turnPower; }
    public double getServoPosition() { return servoPosition; }


    public double getTagHeadingError() {
        if (!active) return 0;

        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null || blocks.length == 0) return 0;


        HuskyLens.Block b = blocks[0];

        // HuskyLens resolution is 320x240
        double frameCenterX = 160.0;
        return (b.x - frameCenterX) / frameCenterX;
    }

    public void printTelemetry(ColorfulTelemetry t) {
        t.reset(); // reset any previous styles

        //telem
        t.addLine("Mode: " + mode);
        t.addData("Turn Power", turnPower);
        t.addData("Servo Position", servoPosition);

        t.update();
    }

    @Override
    public void periodic() {
        if (!active) {
            turnPower = 0;
            return;
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null || blocks.length == 0) {
            turnPower = 0;
            return;
        }

        HuskyLens.Block block = blocks[0]; // take primary target

        int centerX = block.x;
        int centerY = block.y;




        switch (mode) {
            case ARTIFACT:
                // Horizontal rotate
                double xError = centerX - SCREEN_CENTER_X;
                turnPower = clamp(xError * kP_TURN_ARTIFACT);

                // Vertical-based servo drop
                double normalized = (double) centerY / 240.0;
                servoPosition = SERVO_MAX - normalized * (SERVO_MAX - SERVO_MIN);
                servoPosition = clampServo(servoPosition);
                break;

            case APRILTAG:
                double angle = getTagHeadingError();
                turnPower = clamp(angle * kP_TURN_APRILTAG);

                // Keep servo high for vision
                servoPosition = SERVO_MAX;
                break;
        }
    }

    // clamps so it doesnt go crazy
    private double clamp(double v) { return Math.max(-0.3, Math.min(0.3, v)); }
    private double clampServo(double v) { return Math.max(SERVO_MIN, Math.min(SERVO_MAX, v)); }
}
