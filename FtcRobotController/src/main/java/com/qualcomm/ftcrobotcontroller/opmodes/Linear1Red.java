package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by FTCRubies on 11/1/2015.
 * This is one of the first linear programs we created
 */
public class Linear1Red extends LinearOpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;


//    final static int ENCODER_CPR = 1440;
//    final static double GEAR_RATIO = 2;
//    final static int WHEEL_DIAMETER = 4;
//    final static int DISTANCE = 120;
//
//    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
//    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
//    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        sleep(11000);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        sleep(7000);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.7);

        sleep(1500);

        leftMotor.setPower(0.4);
        rightMotor.setPower(0.4);

        sleep(0500);

        leftMotor.setPower(0);
        rightMotor.setPower(0);


//        waitForStart();
//
//        leftMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//
//
//        while(leftMotor.getCurrentPosition() != 0 || rightMotor.getCurrentPosition() != 0) {
//            waitForNextHardwareCycle();
//        }
//
//
//        leftMotor.setTargetPosition((int) COUNTS);
//        rightMotor.setTargetPosition((int) COUNTS);
//
//        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
//
//        telemetry.addData("Starting Motors", 0);
//
//        leftMotor.setPower(0.5);
//        rightMotor.setPower(0.49);
//
//        while(leftMotor.getCurrentPosition() == 0 || rightMotor.getCurrentPosition() == 0) {
//            waitForNextHardwareCycle();
//        }
//
//        while(leftMotor.isBusy() || rightMotor.isBusy()){
//            waitForNextHardwareCycle();
//        }
//
//
//        leftMotor.setPower(0.2);
//        rightMotor.setPower(0.5);
//
//        sleep(1000);
//
//        telemetry.addData("Finished", 0);
//







//
//        //set the left and right motors from the config. file
//        leftMotor = hardwareMap.dcMotor.get("left_drive");
//        rightMotor = hardwareMap.dcMotor.get("right_drive");
//        //reverse the motor on the right side
//        //this makes the robot goes forward with both wheels
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        //Wait until the start button has been pressed on the Drive Station phone
//        waitForStart();
//
//        //Set the motors to the robot driving forward at 0.5 power
//        leftMotor.setPower(0.5);
//        rightMotor.setPower(0.5);
//
//        //let this continue for two seconds
//        sleep(2000);
//
//        //turn robot to the right
//        leftMotor.setPower(0.5);
//        rightMotor.setPower(-0.5);
//        //let this happen for 1.1 seconds
//        sleep(1100);
//
//        //stop the robot
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);


    }
}
