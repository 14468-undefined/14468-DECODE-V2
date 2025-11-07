package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class SpinUpShooterCommand extends SequentialCommandGroup {

    public SpinUpShooterCommand(ShooterSubsystem shooter) {

        addCommands(
                // Start spinning
                new InstantCommand(shooter::spin)

                //wait til at speed
                //new WaitUntilCommand(shooter::atSpeed)
        );


        addRequirements(shooter);
    }
}
