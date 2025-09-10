package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class LoopTimeSubsystem extends UndefinedSubsystemBase{

    ElapsedTime t = new ElapsedTime();
    int pastLoops = 0;
    int curLoops = 0;
    public LoopTimeSubsystem(){

    }

    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine("CUR LOOP TIME: " + pastLoops + " loops per second");
    }

    @Override
    public void periodic() {
        if(t.seconds() > 1.0){
            pastLoops = curLoops;
            curLoops = 0;
            t.reset();
        }
        curLoops++;

    }
}