package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class PrepDriveShootCommand extends SequentialCommandGroup {

    public PrepDriveShootCommand(BaseRobot robot) {
        DriveSubsystem drive = robot.drive;

        //start flywheel up to speed
        SpinUpShooterCommand spinUpShooter = new SpinUpShooterCommand(robot.shooter);

        // start vision
        CommandBase startVision = robot.webcamVision.runOnce(robot.webcamVision::startVision);

        //drive to tag with PID
        CommandBase driveToTagWhenSeen = new CommandBase() {
            {
                addRequirements(robot.drive, robot.webcamVision);
            }

            @Override
            public void execute() {
                if (robot.webcamVision.hasTarget()) {
                    robot.webcamVision.getDriveToTagCommand(drive).execute();
                }
                // else: do nothing, manual driving can continue
            }

            @Override
            public boolean isFinished() {
                // Finish once we have a tag and it’s close enough
                return robot.webcamVision.isAtTarget();
            }

            @Override
            public void end(boolean interrupted) {
                drive.rest();
            }
        };

        // Run shooter and drive in parallel once tag is detected
        ParallelCommandGroup driveAndSpin = new ParallelCommandGroup(spinUpShooter, driveToTagWhenSeen);

        //stop vision
        CommandBase stopVision = robot.webcamVision.runOnce(robot.webcamVision::stopVision);

        //shoot
        //CommandBase shoot = robot.shooter.runOnce(robot.shooter::shoot);

        // --- Sequence everything ---
        addCommands(
                startVision,
                driveAndSpin,
                stopVision
                //shoot
        );
    }
}
