package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class CommandToActionWrapper implements Action {

    CommandBase cmd;
    boolean hasInitialized = false;


    public CommandToActionWrapper(CommandBase command){

        this.cmd=command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!hasInitialized){
            cmd.initialize();
            hasInitialized = true;
        }


        cmd.execute();

        if(cmd.isFinished()){
            cmd.end(false);
            return false;
        }
        else return true;

    }


}