package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="Autov1")
public class AutoV1 extends SampleAuto {
    BaseRobot robot;

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(12, -63, Math.toRadians(270)));

        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {
        //Spec 1
        /*Actions.runBlocking((t) ->{robot.specimenClaw.close();return false;});
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(0, (t) -> {
                    robot.pivot.runToPosition(Constants.PivotConstants.highSpecimenDelivery, 1);
                    return false;})
                .strafeToConstantHeading(new Vector2d(0, -58))


                .build());
        robot.drive.updatePoseEstimate();
        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.highSpecimenDelivery, 1);
            robot.wrist.setFacingStraightParallelToSlider();
            return false;});

        Actions.runBlocking(AutoUtil.getDelayAction(.75));
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToConstantHeading(new Vector2d(-1.5, -36))
                .build());

        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.highClipDeposit, 1);return false;});
        Actions.runBlocking(AutoUtil.getDelayAction(.15));
        Actions.runBlocking((t) -> {robot.specimenClaw.open();return false;});





        //Spec 2 & Push
//        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.wallSpecimenPickup, .5);return false;});
//        Actions.runBlocking((t) -> {robot.pivot.runToPosition(Constants.PivotConstants.wallPickup, .5);return false;});

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToConstantHeading(new Vector2d(45, -48))
                .afterTime(.2, (t) -> {
                    robot.extendo.runToPosition(Constants.ExtendoConstants.wallSpecimenPickup, .5);
                    robot.pivot.runToPosition(Constants.PivotConstants.wallPickup, .5);
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(45, -16))
                .strafeToConstantHeading(new Vector2d(47, -16))
                .strafeToConstantHeading(new Vector2d(47, -70))
                .strafeToConstantHeading(new Vector2d(47, -65))
                .strafeToLinearHeading(new Vector2d(40, -70), Math.toRadians(90.00001))
                .strafeToConstantHeading(new Vector2d(40, -87))
                .build());
        robot.drive.updatePoseEstimate();

        Actions.runBlocking((t) -> {robot.specimenClaw.close();return false;});
        Actions.runBlocking(AutoUtil.getDelayAction(.15));

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(.5, (t) -> {
                    robot.extendo.runToPosition(Constants.ExtendoConstants.highSpecimenDelivery, 1);
                    robot.pivot.runToPosition(Constants.PivotConstants.highSpecimenDelivery, 1);
                    return false;})

                .strafeToConstantHeading(new Vector2d(40, -75))
                .strafeToSplineHeading(new Vector2d(3, -46), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(3, -38))
                .build());
        robot.drive.updatePoseEstimate();
        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.highClipDeposit, 1); return false;});
        Actions.runBlocking(AutoUtil.getDelayAction(.15));
        //Actions.runBlocking((t) -> {robot.specimenClaw.open();return false;});


        //Spec 3
//        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.wallSpecimenPickup, .5);return false;});
//        Actions.runBlocking((t) -> {robot.pivot.runToPosition(Constants.PivotConstants.wallPickup, .5);return false;});

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(0, (t) -> {
                    robot.specimenClaw.open();
                    robot.extendo.runToPosition(Constants.ExtendoConstants.wallSpecimenPickup, .5);
                    robot.pivot.runToPosition(Constants.PivotConstants.wallPickup, .5);
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(30, -63))
                .strafeToSplineHeading(new Vector2d(40, -75), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(40, -88.5))
                .build());
        robot.drive.updatePoseEstimate();

        Actions.runBlocking((t) -> {robot.specimenClaw.close();return false;});
        Actions.runBlocking(AutoUtil.getDelayAction(.15));

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(.5, (t) -> {
                    robot.extendo.runToPosition(Constants.ExtendoConstants.highSpecimenDelivery, 1);
                    robot.pivot.runToPosition(Constants.PivotConstants.highSpecimenDelivery, 1);
                    return false;})

                .strafeToConstantHeading(new Vector2d(40, -75))
                .strafeToSplineHeading(new Vector2d(7, -47), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(8, -38))
                .build());
        robot.drive.updatePoseEstimate();

        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.highClipDeposit, 1); return false;});
        Actions.runBlocking(AutoUtil.getDelayAction(.15));
        Actions.runBlocking((t) -> {robot.specimenClaw.open();return false;});

        Actions.runBlocking((t) -> {robot.pivot.runToPosition(Constants.PivotConstants.rest, .5); return false;});
        Actions.runBlocking((t) -> {robot.extendo.runToPosition(Constants.ExtendoConstants.rest, 1); return false;});
        robot.wrist.setAllTheWayBack();

        Actions.runBlocking(AutoUtil.getDelayAction(2));
        //

         */
    }

    @Override
    public void onStop() {

    }
}