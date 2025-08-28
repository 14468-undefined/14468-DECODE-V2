package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;

@TeleOp(name="HuskyLens Test", group="Test")
public class HuskyLensTest extends LinearOpMode {

    BaseRobot robot;




    boolean dismissTelemInstructions = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BaseRobot(hardwareMap);
        robot.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        robot.huskyLens.initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (!dismissTelemInstructions) {
                telemetry.addLine("press 'a' for april tag detection");
                telemetry.addLine("press 'b' for color detection");
                telemetry.addLine("press 'x' to dismiss");
            }

            if (gamepad1.x) {
                dismissTelemInstructions = true;
            }




            if (gamepad1.a) {
               robot.aprilTagDetection(telemetry);

            }
            else if (gamepad1.b) {
                robot.colorProportionalController(telemetry);
            }

            else {
                    robot.drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x));
                    telemetry.addData("Status", "No Color Detected");
                }
            }

            telemetry.update();
        }
    }
