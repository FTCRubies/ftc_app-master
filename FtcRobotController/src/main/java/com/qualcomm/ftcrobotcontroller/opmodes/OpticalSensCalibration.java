package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by FTCRubies on 11/8/2015.
 */
public class OpticalSensCalibration extends OpMode {
    OpticalDistanceSensor opticalDistanceSensor;

    @Override
    public void init() {
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_EOPD");

    }

    @Override
    public void loop() {
        double reflectance = opticalDistanceSensor.getLightDetected();
        telemetry.addData("Reflectance", reflectance);

    }
}