package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(BaseRobot robot, int numShots, int pose) {
        DriveSubsystem drive = robot.drive;

        if (pose == 1){
            robot.shooter.setTargetRPM(Constants.shooterConstants.CLOSE_SHOT_RPM);
        }
        if (pose == 2){
            robot.shooter.setTargetRPM(Constants.shooterConstants.MID_SHOT_RPM);
        }
        if (pose == 3){
            robot.shooter.setTargetRPM(Constants.shooterConstants.FAR_ZONE_SHOT_RPM);
        }
        // Define commands
        CommandBase spinUpShooter = new RunCommand(robot.shooter::spinUp, robot.shooter);
        CommandBase stopShooter = new InstantCommand(robot.shooter::stop, robot.shooter);
        CommandBase waitTilAtSpeed = new WaitUntilCommand(() -> robot.shooter.atSpeed());
        Command shoot1 = new RunCommand(() -> robot.transfer.spin(), robot.transfer).withTimeout(Constants.transferConstants.TRANSFER_TIME);
        CommandBase firstWait = new WaitCommand(1000);
        Command shoot2 = new RunCommand(() -> robot.transfer.spin(), robot.transfer).withTimeout(Constants.transferConstants.TRANSFER_TIME);
        Command secondWait = new WaitCommand(1000);
        Command shoot3 = new RunCommand(() -> robot.transfer.spin(), robot.transfer).withTimeout(Constants.transferConstants.TRANSFER_TIME);



        addRequirements(robot.shooter, robot.transfer);

        //if 3 balls
        if(numShots == 3) {

            addCommands(
                    // Wait until shooter reaches speed
                    waitTilAtSpeed,

                    // Parallel group: shooter spins constantly while doing the transfer pulses
                    new ParallelCommandGroup(spinUpShooter,
                            // Sequential transfer pulses
                            new SequentialCommandGroup(
                                    shoot1,
                                    firstWait,
                                    shoot2,
                                    secondWait,
                                    shoot3,
                                    stopShooter
                            )
                    )

            );
        }
        else if(numShots == 2){

            addCommands(
                    // Wait until shooter reaches speed
                    waitTilAtSpeed,

                    // Parallel group: shooter spins constantly while doing the transfer pulses
                    new ParallelCommandGroup(spinUpShooter,
                            // Sequential transfer pulses
                            new SequentialCommandGroup(
                                    shoot1,
                                    firstWait,
                                    shoot2,
                                    stopShooter
                            )
                    )

            );
        }
        else{
            addCommands(
                    // Wait until shooter reaches speed
                    waitTilAtSpeed,

                    // Parallel group: shooter spins constantly while doing the transfer pulses
                    new ParallelCommandGroup(spinUpShooter,
                            // Sequential transfer pulses
                            new SequentialCommandGroup(
                                    shoot1,
                                    stopShooter
                            )
                    )

            );
        }

    }

    }

