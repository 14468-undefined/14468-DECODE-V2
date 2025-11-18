package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "LEDTest" , group = "TeleOp")
public class LEDTest extends SampleCommandTeleop {

    //BaseRobot robot;


    int shooterRPM = 6000;

    double curPose = 0;

    ElapsedTime time = new ElapsedTime();

    double LEDColor = .5;

    boolean goingUp = true;    // whether we're going 0→1 or 1→0

    @Override
    public void onInit() {


    }

    @Override
    public void onStart() {

        g1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            LEDColor += .01;
        });
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            LEDColor -= .01;
        });


        g1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() ->{
            robot.LED.setPoseTest(LEDColor);
        });








    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop

        //pen.addLine("shooter RPM set:", shooterRPM);

        curPose = robot.LED.getPose();

        pen.addData("LED Color: ", robot.LED.getPose());
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
