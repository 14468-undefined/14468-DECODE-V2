package org.firstinspires.ftc.teamcode.BaseRobot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;


import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;


import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;


public class BaseRobot {


    //color sensor


    //end color sensor


    //motor powers
    //private final double MotorExamplePower = 0.9;


    //end motor powers

    // Arm Position fields
    //private int MotorExamplePos = 0;


    //private double ServoExamplePos = 0;


    //servo constants

    //private final double ServoMax = .7638;


    //end servo constants

    //motor constants

    //private final int MotorExampleMax = 3100;


    //end motor constants

    //private int realIntakeSlidesPos = 0; //TODO: figure out what this was


    public MecanumDrive drive;

    //DcMotor ExampleMotor;


    //Servo ExampleServo;

    DigitalChannel touchSensor1;


    public RevColorSensorV3 colorSensor;


    public BaseRobot(HardwareMap hwMap) {
        this(hwMap, new Pose2d(0, 0, 0));
    }

    public BaseRobot(HardwareMap hwMap, Pose2d pose) {

        drive = new MecanumDrive(hwMap, pose);

        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.enableLed(true);

        this.touchSensor1 = hwMap.digitalChannel.get("touchSensor1");


        //ExampleMotor = hwMap.dcMotor.get("ExampleMotor");
        //ExampleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to run with encoders and grab current Position
        //ExampleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ExampleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //ExampleMotorPos = leftIntakeSlider.getCurrentPosition();


        //ExampleServo = hwMap.servo.get("ExampleServo");
        //ExampleServo.setPosition(Constants.Constants.open);
        //ExampleServoPos = ExampleServo.getPosition();


    }

    public void resetExampleEncoders() {
        //ExampleMotorPos = 0;
        //ExampleMotor.setPower(0);

        //ExampleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ExampleMotor.setTargetPosition(0);

    }



    /*public boolean touchSensorCheck() {
        return !TouchSensor1.getState();
    }

     */


    public String detectColor() {
        // Get the RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (isRedSample(red, green, blue)) {
            return "RedSample";
        } else if (isBlueSample(red, green, blue)) {
            return "BlueSample";
        } else if (isYellowSample(red, green, blue)) {
            return "YellowSample";
        } else {
            return "No Color"; // Default when no color is detected
        }
    }

    private boolean isRedSample(int red, int green, int blue) {
        return red > 100 && green < 80 && blue < 80;
    }

    private boolean isBlueSample(int red, int green, int blue) {
        return blue > 100 && red < 80 && green < 80;//check if all of these are true, if not return false
    }

    private boolean isYellowSample(int red, int green, int blue) {
        return red > 100 && green > 100 && blue < 80;
    }


    //what would cause a camera video t


    public void update() {
        //motors

        //update all motors

        //servos

        //update all servos
    }

    public void TeleopUpdate() {
        //motors


        //servos
        //robot.updateServoPos();
    }


    public void updateServoPos() {
        /*if (servoPos > 1) {
            servoPos = 1;
        }
        if (servoPos < 0) {
            servoPos = 0;
        }
        servo.setPosition(servoPos);

         */
    }

    public void changeServoPos(double deltaPos) {

        // servoPos += deltaPos;
    }

    public void setServoPos(double newPos) {

        //ServoPos = newPos;
    }


    public void updateSlidesMotorPos() {
        //this is only if you are using 2 viperslides in parallel because they can fight each other
        /*if (leftSlider.getCurrentPosition() < 50 && rightSlidePos < 50) {
            leftSlider.setPower(0);
            rightSlider.setPower(0);

        }
        if (leftSlider.getCurrentPosition() == leftSlidePos && leftSlider.getCurrentPosition() != 0) {
            leftSlider.setPower(.2);//power when running
            rightSlider.setPower(.2);
        }


        rightSlider.setTargetPosition(rightIntakeSlidePos);
        leftSlider.setTargetPosition(leftIntakeSlidePos);

        rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (leftSlider.getCurrentPosition() != leftSlidePos) {
            rightSlider.setPower(Constants.SlideConstants.power);
            leftSlider.setPower(Constants.SlideConstants.power);
        }


        if (leftSlidePos > Constants.SlideConstants.MAX) {
            leftSlidePos = Constants.SlideConstants.MAX;//limit
            rightSlidePos = Constants.SlideConstants.MAX;//limit
        }
        if (rightSlidePos < 0) {
            rightSlidePos = 0;//limit
            leftSlidePos = 0;
        }

         */


    }

    public void changeSlidesPos(int deltaPos) {
        //rightSlidePos += deltaPos;
        //leftSlidePos += deltaPos;

    }


    public void setSlidesPos(int newPos) {
        //rightSlidePos = newPos;
        //leftSlidePos = newPos;

    }

    public void setSlidesPower(double power) {
        /*rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlidePos = leftSlider.getCurrentPosition();
        leftSlidePos = leftSlider.getCurrentPosition();
        rightSlider.setPower(power);
        leftSlider.setPower(power);

         */
    }


    public void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {

        }

    }


    //PUT ALL ACCESSORS HERE!!
    /*
    public double getServoPos(){
        return servoPos;
    }
     */

}
