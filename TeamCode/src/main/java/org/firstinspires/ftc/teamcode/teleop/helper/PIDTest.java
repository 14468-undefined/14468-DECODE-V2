package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;


@TeleOp(name = "PIDTest" , group = "TeleOp")
public class PIDTest extends CommandOpMode {
    ShooterSubsystem shooter;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void initialize() {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("init done");
        telemetry.update();


    }

    @Override
    public void run() {
        super.run();

        shooter.applyPIDF();
        shooter.spin();


        packet.put("target_shooter_rpm", shooter.getTargetRPM());
        packet.put("current_shooter_rpm", shooter.getShooterVelocity());
        dash.sendTelemetryPacket(packet);


    }
}