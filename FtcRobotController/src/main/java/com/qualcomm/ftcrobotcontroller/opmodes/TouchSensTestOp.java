package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by FTCRubies on 11/7/2015.
 */
public class TouchSensTestOp extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchSensor;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor =  hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        //reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //get a reference from the hardware map to the touch sensor
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");

    }

    @Override
    public void loop() {
        if (touchSensor.isPressed()) {
            //Stop the motors when the touch sensor is pressed
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else {
            //Continue to drive if the touch sensor is not pressed
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
        }
        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));

    }
}
