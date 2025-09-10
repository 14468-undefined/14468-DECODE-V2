package org.firstinspires.ftc.teamcode.teleop.helper;

// FTC SDK
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Hardware
import com.qualcomm.robotcore.hardware.HardwareMap;

// FTCLib command-based
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

// Your subsystems
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.WebcamAprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

// Utilities
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

// Java
import java.util.function.DoubleSupplier;

@TeleOp
public class AprilTagTestTeleop extends LinearOpMode {

    BaseRobot robot;
    CommandBase driveToTagCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        // Schedule the command
        driveToTagCommand = robot.webcamVision.getDriveToTagCommand(robot.drive);
        driveToTagCommand.schedule();

        while (opModeIsActive()) {
            // normal manual control if no tag
            if (!robot.webcamVision.hasTarget()) {
                double forward = -gamepad1.left_stick_y;
                double strafe  =  gamepad1.left_stick_x;
                double turn    = -gamepad1.right_stick_x;

                robot.drive.driveFieldcentric(strafe, forward, turn, 1.0);
            }

            robot.cTelemetry.reset();
            robot.printTelemetry(robot.cTelemetry);
            robot.cTelemetry.update();
        }

        driveToTagCommand.cancel();
    }
}
