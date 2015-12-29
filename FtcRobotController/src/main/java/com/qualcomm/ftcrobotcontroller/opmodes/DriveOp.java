package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTCRubies on 10/31/2015.
 */
public class DriveOp extends OpMode {

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
        //get values from gamepads
        //use negative power because forward returns -1

        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        //set power of motors with gamepad values
        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);
    }
}
