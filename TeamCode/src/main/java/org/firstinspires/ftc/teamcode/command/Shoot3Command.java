package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class Shoot3Command extends SequentialCommandGroup {

    public Shoot3Command(BaseRobot robot) {
        DriveSubsystem drive = robot.drive;

        // Define commands
        CommandBase shoot = new RunCommand(robot.shooter::shoot, robot.shooter);
        SpinUpShooterCommand spinUpShooter = new SpinUpShooterCommand(robot.shooter);
        CommandBase stopShooter = new InstantCommand(robot.shooter::stop, robot.shooter);

        // Only require subsystems you actually use
        addRequirements(robot.shooter);

        // Add commands in sequence
        addCommands(
                spinUpShooter,
                shoot,
                spinUpShooter,
                shoot,
                spinUpShooter,
                shoot,
                stopShooter
        );
    }
}
