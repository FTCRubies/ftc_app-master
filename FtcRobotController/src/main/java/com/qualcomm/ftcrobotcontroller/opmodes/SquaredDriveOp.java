package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTCRubies on 11/1/2015.
 */
public class SquaredDriveOp extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        //reverse the left motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightSquaredVal = rightStickVal * rightStickVal;
        double leftSquaredVal = leftStickVal * leftStickVal;

        if(rightStickVal < 0) {
            rightMotor.setPower(-rightSquaredVal);
        } else {
            rightMotor.setPower(rightSquaredVal);
        }
        if(leftStickVal < 0) {
            leftMotor.setPower(-leftSquaredVal);
        } else {
            leftMotor.setPower(leftSquaredVal);

        }
    }
}
