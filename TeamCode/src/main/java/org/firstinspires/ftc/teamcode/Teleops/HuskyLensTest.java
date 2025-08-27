package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;

import java.util.List;

@TeleOp(name="HuskyLens Test", group="Test")
public class HuskyLensTest extends LinearOpMode {


    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;




    ;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get HuskyLens from hardware map
        robot = new BaseRobot(hardwareMap);
        robot.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        robot.huskyLens.initialize();  // only if your library requires it
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.addLine("HuskyLens ready — waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            // Ask the sensor for the list of recognized blocks



// Get all detected blocks
            HuskyLens.Block[] blocks = robot.huskyLens.blocks();



            if (blocks != null && blocks.length > 0) {
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block b = blocks[i];

                    // Color ID (always 1 for COLOR_RECOGNITION)
                    int colorID = b.id;

                    // Center location
                    int centerX = b.x;
                    int centerY = b.y;

                    // Size
                    int width = b.width;
                    int height = b.height;

                    // Send to telemetry
                    telemetry.addData("Block " + i,
                            "ColorID=%d, Center=(%d,%d), Size=(%d,%d)",
                            colorID, centerX, centerY, width, height);
                }
            } else {
                telemetry.addData("Blocks", "No blocks detected yet");
            }

            telemetry.update();



        }
    }
}