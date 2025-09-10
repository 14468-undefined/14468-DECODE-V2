package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class BaseRobot extends UndefinedSubsystemBase {

    public AutoUtil autoGenerator;
    public ShooterSubsystem shooter;
    public DriveSubsystem drive;
    public IntakeSubsystem intake;
    public ColorfulTelemetry cTelemetry;


    public BaseRobot(HardwareMap hwMap, Pose2d startPos){
        drive = new DriveSubsystem(hwMap, startPos);
        shooter = new ShooterSubsystem(hwMap, cTelemetry);
        intake = new IntakeSubsystem(hwMap, cTelemetry);
        autoGenerator = new AutoUtil(drive);
    }


    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        drive.printTelemetry(t);
        intake.printTelemetry(t);
        shooter.printTelemetry(t);
    }

    @Override
    public void periodic() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();

    }

    class ExampleAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

           //do things here



            //AutoUtil.delay(1.5);


            return false;
        }
    }


}