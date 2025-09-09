package org.firstinspires.ftc.teamcode.teleop.helper;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "A_CommandBasedTest")
public class CommandBasedTest extends CommandOpMode {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    @Override
    public void initialize() {
        FtcDashboard dash = FtcDashboard.getInstance();
        ColorfulTelemetry cTelemetry = new ColorfulTelemetry(telemetry, dash);

        shooter = new ShooterSubsystem(hardwareMap, cTelemetry);
        intake = new IntakeSubsystem(hardwareMap, cTelemetry);

        // Button mappings (gamepad1)
        // Y = intake, A = outtake, B = stop
        //gamepad1.y.whileHeld(new InstantCommand(intake::intake, intake));
        //gamepad1.a.whileHeld(new InstantCommand(intake::intakeReverse, intake));
        //gamepad1.b.whenPressed(new InstantCommand(intake::stop, intake));

       // new Trigger(() -> g2.getButton(GamepadKeys.Button.Y)).whileActiveOnce(new Intake(robot.shooter));

        //add force stop button
        schedule(); // start the command scheduler
    }
}