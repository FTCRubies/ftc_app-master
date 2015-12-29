package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by FTCRubies on 11/8/2015.
 */
public class OpticalSensTestOp extends OpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    OpticalDistanceSensor opticalDistanceSensor;
    ElapsedTime timer;

    double lightValue = (0.1);
    double darkValue = (0.2);
    double threshold = (lightValue+darkValue) / 2;

    enum State {Drive, Backup, Turn}
    State state;

    double BACKUP_TIME = 0.8;
    double TURN_TIME = 0.7;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        //reverse the right motor so the robot drives straight
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //get references to the optical distance sensor  from the hardware map
        //turn the red LED on
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_EOPD");

        //set the initial state
        state = State.Drive;

        //set up the timer
        timer = new ElapsedTime();

    }

    @Override
    public void loop() {
        //get the amount of the reflected light as a value from 0 to 1
        double reflectance = opticalDistanceSensor.getLightDetected();

        //Run the appropriate case based on the current state
        switch(state) {
            case Drive:
                //See if the light sensor detects the line, and switch states
                if(reflectance > threshold) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    state = State.Backup;
                    //reset the timer for backing up
                    timer.reset();
               } else {
                    //If not, set the motors to drive forward slowly
                    leftMotor.setPower(0.15);
                    rightMotor.setPower(0.15);
                }
                break;
            case Backup:
                //Set the motors to drive backwards
                leftMotor.setPower(-0.25);
                rightMotor.setPower(-0.25);

                //Check if the time to back up is complete, and switch states to turning
                if (timer.time() >= BACKUP_TIME) {
                    state = State.Turn;
                    timer.reset();

                }
                break;
            case Turn:
                //set the robot right
                leftMotor.setPower(0.25);
                rightMotor.setPower(-0.25);

                //Check if the time to turn is complete, and switch the states back to driving
                if (timer.time() >= TURN_TIME) {
                    state = State.Drive;
                }
                break;

        }

        telemetry.addData("Current State", state.name());
        telemetry.addData("Reflectance", reflectance);

    }
}
