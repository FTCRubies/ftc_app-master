package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by FTCRubies on 11/8/2015.
 */
public class LineFollowOp extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    OpticalDistanceSensor opticalDistanceSensor;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_EOPD");

    }

    @Override
    public void loop() {
        //Write the reflectance detected to a variable
        double reflectance = opticalDistanceSensor.getLightDetected();

        //if the sensor is on the line
        //only the right motor rotates to move it off of the line
        if (reflectance >= 0.25) {
            rightMotor.setPower(-0.2);
            leftMotor.setPower(0);
        }
        //If the sensor is off of the line,
        //the left motor rotates to move it back towards the line
        else {
            leftMotor.setPower(-0.2);
            rightMotor.setPower(0);
            telemetry.addData("Reflectance", reflectance);
        }


    }
}
