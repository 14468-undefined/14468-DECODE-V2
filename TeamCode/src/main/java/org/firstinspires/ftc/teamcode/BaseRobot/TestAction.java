package org.firstinspires.ftc.teamcode.BaseRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;

public class TestAction implements Action {

    BaseRobot robot;
    ElapsedTime time = new ElapsedTime();



    public TestAction(BaseRobot b){
        this.robot=b;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {


        //put movements here
        //robot.setMotorPos(0);
        //other stuff


        while(time.milliseconds() < 100){
            //.1 sec delay

        }








        return false;
    }



}