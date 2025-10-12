package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.UndefinedSubsystemBase;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class HuskyLensSubsystem extends UndefinedSubsystemBase {

    private final HuskyLens huskyLens;
    private boolean active = true;

    public HuskyLensSubsystem(HardwareMap hardwareMap, ColorfulTelemetry t) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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


    public void printTelemetry(ColorfulTelemetry t) {
        t.reset(); // reset any previous styles

        t.update();
    }

    @Override
    public void periodic() {

    }
}
