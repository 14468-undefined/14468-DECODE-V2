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



    public boolean touchSensorCheck() {
        //return !TouchSensor1.getState();
    }


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
        updateAxlePos();
        updateTrayPos();
        updateGimbalPos();
        updateIntakeGrasperPos();
        updateOuttakeGrasperPos();
        updateWristPos();
        updateV4bPos();
    }

    public void prepForIntake() {
        setV4bPos(Constants.v4bConstants.hover);
    }

    public void intakeGround() {
        setV4bPos(Constants.v4bConstants.ground);

    }

    public void intakeUpStage() {
        //move up one stage
        if (v4bPos == Constants.v4bConstants.ground) {
            setV4bPos(Constants.v4bConstants.hover);
        } else if (v4bPos == Constants.v4bConstants.hover) {
            setV4bPos(Constants.v4bConstants.up);
        } else {
            setV4bPos(Constants.v4bConstants.up);
        }
    }

    public void intakeDownStage() {
        //go down one stage
        if (v4bPos == Constants.v4bConstants.up) {
            setV4bPos(Constants.v4bConstants.hover);
        } else if (v4bPos == Constants.v4bConstants.hover) {
            setV4bPos(Constants.v4bConstants.ground);
        } else {
            setV4bPos(Constants.v4bConstants.ground);
        }
    }

    public void axleToggle() {
        if (outtakeAxlePos == Constants.outtakeAxleConstants.specScoring || outtakeAxlePos == Constants.outtakeAxleConstants.HBScoring) {
            setAxlePos(Constants.outtakeAxleConstants.passThrough);
        } else {
            setAxlePos(Constants.outtakeAxleConstants.specScoring);
        }
    }

    public void intakeGrasperToggle() {
        //if open, close | if closed, open
        if (intakeGrasperPos == Constants.intakeClawConstants.closed) {
            setIntakeGrasperPos(Constants.intakeClawConstants.open);
        } else {
            setIntakeGrasperPos(Constants.intakeClawConstants.closed);
        }
    }

    public void outtakeGrasperToggle() {
        if (outtakeGrasperPos == Constants.outtakeClawConstants.closed) {
            setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        } else {
            setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        }
    }


    public void servoTestingUpdate() {
        intakeGimbal.setPosition(gimbalPos);
        outtakeAxle.setPosition(outtakeAxlePos);
        tray.setPosition(trayPos);
        intakeGrasper.setPosition(intakeGrasperPos);
        outtakeGrasper.setPosition(outtakeGrasperPos);
        v4b.setPosition(v4bPos);
    }

    public void updateGimbalPos() {
        if (gimbalPos > 1) {
            gimbalPos = 1;
        }
        if (gimbalPos < 0) {
            gimbalPos = 0;
        }
        intakeGimbal.setPosition(gimbalPos);
    }

    public void changeGimbalPos(double deltaPos) {
        gimbalPos += deltaPos;
    }

    public void setGimbalPos(double newPos) {
        gimbalPos = newPos;
    }

    public void updateWristPos() {
        outtakeWrist.setPosition(outtakeWristPos);
    }

    public void setOuttakeWristPos(double newPos) {
        outtakeWristPos = newPos;
    }

    public void changeWristPos(double deltaPos) {
        outtakeWristPos += deltaPos;
    }

    public void updateAxlePos() {
        //if(outtakeSlidesPos < OUTTAKE_SLIDES_TRANSFER && outtakeAxlePos > AXLE_IN_ROBOT){
        //  outtakeAxlePos = AXLE_IN_ROBOT;
        //}
        outtakeAxle.setPosition(outtakeAxlePos);
    }

    public void changeAxlePos(double deltaPos) {
        outtakeAxlePos += deltaPos;
    }

    public void setAxlePos(double newPos) {
        outtakeAxlePos = newPos;
    }

    public void updateTrayPos() {
        tray.setPosition(trayPos);
    }

    public void changeTrayPos(double deltaPos) {
        trayPos += deltaPos;
    }

    public void setTrayPos(double newPos) {

        trayPos = newPos;
    }

    public void updateIntakeGrasperPos() {

        if (v4bPos == Constants.v4bConstants.up) {
            setIntakeGrasperPos(Constants.intakeClawConstants.closed);
        }
        //if(intakeGrasperPos != INTAKE_GRASPER_CLOSED && intakeGrasperPos != INTAKE_GRASPER_OPEN){
        //  intakeGrasperPos = INTAKE_GRASPER_CLOSED;
        //}
        intakeGrasper.setPosition(intakeGrasperPos);
    }

    public void changeIntakeGrasperPos(double deltaPos) {
        intakeGrasperPos += deltaPos;
    }

    public void setIntakeGrasperPos(double newPos) {
        intakeGrasperPos = newPos;
    }

    public void updateOuttakeGrasperPos() {
        outtakeGrasper.setPosition(outtakeGrasperPos);
    }

    public void changeOuttakeGrasperPos(double deltaPos) {
        outtakeGrasperPos += deltaPos;
    }

    public void setOuttakeGrasperPos(double newPos) {
        outtakeGrasperPos = newPos;
    }

    public void updateV4bPos() {

        //less than up = up
        if (v4bPos < Constants.v4bConstants.up && rightIntakeSlidePos < Constants.intakeSlideConstants.minFromGround) {

            setIntakeSlidesPos(Constants.intakeSlideConstants.minFromGround);
            setV4bPos(Constants.v4bConstants.hover);
        }
        v4b.setPosition(v4bPos);
    }

    public void changeV4bPos(double deltaPos) {
        v4bPos += deltaPos;
    }

    public void setV4bPos(double newPos) {
        v4bPos = newPos;
    }


    public void updateIntakeSlidesPos() {

        if (leftIntakeSlider.getCurrentPosition() < 50 && rightIntakeSlidePos < 50) {
            leftIntakeSlider.setPower(0);
            rightIntakeSlider.setPower(0);

        }
        if (leftIntakeSlider.getCurrentPosition() == leftIntakeSlidePos && leftIntakeSlider.getCurrentPosition() != 0) {
            leftIntakeSlider.setPower(.2);
            rightIntakeSlider.setPower(.2);
        }


        rightIntakeSlider.setTargetPosition(rightIntakeSlidePos);
        leftIntakeSlider.setTargetPosition(leftIntakeSlidePos);

        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (leftIntakeSlider.getCurrentPosition() != leftIntakeSlidePos) {
            rightIntakeSlider.setPower(Constants.intakeSlideConstants.power);
            leftIntakeSlider.setPower(Constants.intakeSlideConstants.power);
        }


        if (leftIntakeSlidePos > Constants.intakeSlideConstants.MAX) {
            leftIntakeSlidePos = Constants.intakeSlideConstants.MAX;//limit
            rightIntakeSlidePos = Constants.intakeSlideConstants.MAX;//limit
        }
        if (rightIntakeSlidePos < 0) {
            rightIntakeSlidePos = 0;//limit
            leftIntakeSlidePos = 0;
        }


    }

    public void changeIntakeSlidesPos(int deltaPos) {
        rightIntakeSlidePos += deltaPos;
        leftIntakeSlidePos += deltaPos;

    }


    public void setIntakeSlidesPos(int newPos) {
        rightIntakeSlidePos = newPos;
        leftIntakeSlidePos = newPos;

    }

    public void setIntakePower(double power) {
        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeSlidePos = leftIntakeSlider.getCurrentPosition();
        leftIntakeSlidePos = leftIntakeSlider.getCurrentPosition();
        rightIntakeSlider.setPower(power);
        leftIntakeSlider.setPower(power);
    }

    //outtake -------------------------------------------------------------------------
    public void updateOuttakeSlidesPos() {

        if (leftOuttakeSlider.getCurrentPosition() < 50 && leftOuttakeSlidePos < 50) {
            leftOuttakeSlider.setPower(0);
            rightOuttakeSlider.setPower(0);

        }

        leftOuttakeSlider.setTargetPosition(leftOuttakeSlidePos);
        rightOuttakeSlider.setTargetPosition(rightOuttakeSlidePos);

        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightOuttakeSlider.setPower(Constants.outtakeSlideConstants.power);
        leftOuttakeSlider.setPower(Constants.outtakeSlideConstants.power);

        if (outtakeReset == true) {
            if (rightOuttakeSlidePos > Constants.outtakeSlideConstants.MAX) {
                rightOuttakeSlidePos = Constants.outtakeSlideConstants.MAX;
                leftOuttakeSlidePos = Constants.outtakeSlideConstants.MAX;

            }
            if (rightOuttakeSlidePos < 0) {
                rightOuttakeSlidePos = 0;
                leftOuttakeSlidePos = 0;
            }
        }
        // if(leftOuttakeSliderPos != rightOuttakeSliderPos){
        //   leftOuttakeSliderPos = rightOuttakeSliderPos;
        //}


    }

    public void changeOuttakeSlidesPos(int deltaPos) {
        rightOuttakeSlidePos += deltaPos;
        leftOuttakeSlidePos += deltaPos;
    }

    public void setOuttakeSlidesPos(int newPos) {
        //if(outtakeReset){
        rightOuttakeSlidePos = newPos;
        leftOuttakeSlidePos = newPos;
        //}

    }

    public void setOuttakePower(double power) {
        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOuttakeSlidePos = rightOuttakeSlider.getCurrentPosition();
        leftOuttakeSlidePos = leftOuttakeSlider.getCurrentPosition();
        rightOuttakeSlider.setPower(power);
        leftOuttakeSlider.setPower(power);
    }


    public void delay(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {

        }

    }


    public int getRealIntakeSlidesPos() {
        return realIntakeSlidesPos;
    }

    //servos
    public double getGimbalPos() {
        return gimbalPos;
    }

    public double getIntakeGrasperPos() {
        return intakeGrasperPos;
    }

    public double getOuttakeGrasperPos() {
        return outtakeGrasperPos;
    }

    public double getOuttakeWristPos() {
        return outtakeWristPos;
    }

    public double getOuttakeAxlePos() {
        return outtakeAxlePos;
    }

    public double getV4bPos() {
        return v4bPos;
    }

    public double getTrayPos() {
        return trayPos;
    }

    public int getRightOuttakeSlidePos() {
        return rightOuttakeSlidePos;
    }

    public int getLeftOuttakeSlidePos() {
        return leftOuttakeSlidePos;
    }

    public int getRightIntakeSlidePos() {
        return rightIntakeSlidePos;
    }

    public int getLeftIntakeSlidePos() {
        return leftIntakeSlidePos;
    }
}
//end servos

