package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot.TestAction;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class BasicTest extends LinearOpMode {

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



            //Put any non-drive related commands here







            telemetry.addLine("robot position (starting at x: 0, y: 0, heading: 0)");
            //telemetry.addData("x:", drive.pose.position.x);
            //telemetry.addData("y:", drive.pose.position.y);
            //telemetry.addData("heading (deg):", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("Motors: ");
            telemetry.addLine();

            telemetry.addLine("Example motor group: ");
            //telemetry.addData("right something motor: ", robot.getRightSomethingMotor);


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