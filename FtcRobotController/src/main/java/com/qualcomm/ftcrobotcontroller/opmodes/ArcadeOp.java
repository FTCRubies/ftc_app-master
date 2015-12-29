package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTCRubies on 11/1/2015.
 *This is the OpMode that we created and chose to use in the driver-controlled period
 */
public class ArcadeOp extends OpMode {

    final double LEFT_OPEN_POSITION = 0.2;
    final double LEFT_CLOSED_POSITION = 0.8;
    final double RIGHT_OPEN_POSITION = 0.9;
    final double RIGHT_CLOSED_POSITION = 0.2;


    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftArm;
    Servo leftGripper;
    Servo rightGripper;

    @Override
    public void init() {
       //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftArm = hardwareMap.dcMotor.get("left_arm");

        //reverse the left motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");


    }

    @Override
    public void loop() {
        //get values from gamepads
        //pushing forward returns negative value
        //so we reverse the y-value
        float xValue = gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;
        //speedValue is used in the right joystick on the drive train
        //gamepad. We use it to speed up and slow down the power
        //with more precision
        float speedValue = -0.45f *(gamepad1.right_stick_y +1)+1;


        //calculate the power amount for each motor
        float leftPower = (yValue + xValue) * speedValue;
        float rightPower = (yValue - xValue) *speedValue;

        //clip power values so that it is only between -1 and 1
        leftPower = Range.clip(leftPower, -1,1);
        rightPower = Range.clip(rightPower,-1,1);

        //set power of motors with the values on the gamepad
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);


        //this controls the up and down movement of the arm
        //it uses the y and b buttons9oo
        if(gamepad2.y) {
            leftArm.setPower(0.2);
        } else if (gamepad2.b) {
            leftArm.setPower(-0.2);
        }else {
            leftArm.setPower(0);

        //this opens and closes the gripper with two buttons
        // the x button opens the grippers and the a closes them
        if (gamepad1.x) {
            leftGripper.setPosition(LEFT_OPEN_POSITION);
            rightGripper.setPosition(RIGHT_OPEN_POSITION);


        }
        if (gamepad1.a) {
            leftGripper.setPosition(LEFT_CLOSED_POSITION);
            rightGripper.setPosition(RIGHT_CLOSED_POSITION);
        }
        }





    }
}
