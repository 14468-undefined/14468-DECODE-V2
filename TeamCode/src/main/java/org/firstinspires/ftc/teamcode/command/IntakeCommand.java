package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(BaseRobot robot) {


        addRequirements(robot.intake, robot.transfer);
        addCommands(
                // Run intake
                new RunCommand(() -> robot.intake.intake(), robot.intake),

                // Run transfer in reverse
                new RunCommand(() -> robot.transfer.spinReverse(), robot.transfer)
        );
    }
}
