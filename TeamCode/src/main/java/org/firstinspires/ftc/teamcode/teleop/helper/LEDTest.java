package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "LEDTest" , group = "TeleOp")
public class LEDTest extends SampleCommandTeleop {

    //BaseRobot robot;


    int shooterRPM = 6000;

    double LEDPose = .5;

    @Override
    public void onInit() {








    }

    @Override
    public void onStart() {


        g1.getGamepadButton(GamepadKeys.Button.A).whileHeld(() -> {
            robot.LED.setPoseTest(LEDPose);
        });


        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            LEDPose -= .02;
        });
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            LEDPose += .02;
        });


        LEDPose+=.01;






    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop

        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
