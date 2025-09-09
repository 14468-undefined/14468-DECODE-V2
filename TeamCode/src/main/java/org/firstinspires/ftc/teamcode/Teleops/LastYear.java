package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

@TeleOp
public class LastYear extends LinearOpMode {//leagues last year

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    volatile BaseRobot robot;
    private volatile boolean endTeleop = false;


    boolean prevA = false;
    boolean prevY = false;
    boolean prevB = false;
    boolean prevDpadRight = false;
    boolean prevDpadUp = false;
    boolean prevDpadDown = false;

    boolean prevBack = false;
    boolean prevStart = false;
    boolean prevTouchOuttake = false;
    boolean prevTouchIntake = false;


    @Override
    public void runOpMode() throws InterruptedException {


        Thread driveThread = new Thread(this::driveLoop);



        waitForStart();
        //USE SPEED VARIABLE FOR DRIVE
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);
        driveThread.start();

        while(!isStopRequested() && opModeIsActive()) {

            robot.TeleopUpdate();





/*
            telemetry.addLine();
            telemetry.addLine("Servos: ");
            telemetry.addLine();

            telemetry.addLine("Intake:");
            telemetry.addData("gimbal servo pos" , robot.getGimbalPos());
            telemetry.addData("intake grasper pos" , robot.getIntakeGrasperPos());
            telemetry.addData("v4b pos" , robot.getV4bPos());
            telemetry.addLine();

            telemetry.addLine("Outtake:");
            telemetry.addData("Axle servo pos" , robot.getOuttakeAxlePos());
            telemetry.addData("outtake wrist pos:" , robot.getOuttakeWristPos());
            telemetry.addData("outtake grasper pos" , robot.getOuttakeGrasperPos());

            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("controls: ");
            telemetry.addLine();

            telemetry.addLine("Gamepad1:");
            telemetry.addLine("right/left stick: drive");
            telemetry.addLine("dpad_up/right & left/down: speeds high/med/low");

            telemetry.addLine();

            telemetry.addLine("Gamepad2:");
            telemetry.addLine("sticks: intake slides");
            telemetry.addLine("triggers: outtake slides");
            telemetry.addLine("dpad_up: Specimen scoring");
            telemetry.addLine("dpad_left: HB scoring");
            telemetry.addLine("dpad_down: Grab from wall");
            telemetry.addLine("dpad_right: v4b hover over ground");
            telemetry.addLine("bumpers: outtake claw");
            telemetry.addLine("b/x: intake claw open/closed");
            telemetry.addLine("y: intake action");
            telemetry.addLine("a: v4b intake pos");
            */


            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
        endTeleop=true;
        try{
            driveThread.join();
        }
        catch (InterruptedException e){

        }






        }
    public void driveLoop(){
        while(!endTeleop){

            double driveSpeed = 1;


            //GAMEPAD1------------------------------------------------------------------------

            if (gamepad1.dpad_up){
                driveSpeed = 1;//full speed
            }
            if (gamepad1.dpad_left || gamepad1.dpad_right){
                driveSpeed = .5;//half speed
            }
            if (gamepad1.dpad_down){
                driveSpeed = 0.25;//quarter speed
            }
            if(gamepad1.a){
                driveSpeed = .125;//1/8 speed
            }
            //drive
            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed), -gamepad1.right_stick_x * driveSpeed));

        }
    }
}