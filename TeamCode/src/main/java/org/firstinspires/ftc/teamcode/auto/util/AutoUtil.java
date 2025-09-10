package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class AutoUtil {
    //Net side start (close to baskets)
    public static final int NET = 0;

    //Observation side start(farm from baskets)
    public static final int OBS = 1;

    public static final Vector2d highBasketDeposit = new Vector2d(-60, -48);

    //Starting Pose2d from the net side
    public static final Pose2d NETSTART = new Pose2d(new Vector2d(-36, -63), Math.toRadians(90));
    public static final Pose2d OBSSTART = new Pose2d(12, -63, Math.toRadians(90));

    //Base robot's drive subsystem
    private DriveSubsystem drive;

    public AutoUtil(DriveSubsystem drive) {
        this.drive=drive;
    }

    public Action getStartToNetAction(int start) {
        if(start==NET) {
            return drive.actionBuilder(NETSTART)
                    .strafeToLinearHeading(highBasketDeposit, Math.toRadians(45))
                    .build();
        }
        else {
            return drive.actionBuilder(OBSSTART)
                    .strafeToLinearHeading(new Vector2d(-75,-46), Math.toRadians(90))
                    .build();
        }
    }

    public Action getBackupToDepositAction(Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeToLinearHeading(highBasketDeposit, Math.toRadians(45))
                .build();
    }


    public Action getNetToBoxThree(Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-55, -36), Math.toRadians(90))
                .build();
    }



    public static void delay(double t) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < t) {
        }
    }
    public static Action getDelayAction(double t) {
        return (q) -> {delay(t); return false;};
    }

}