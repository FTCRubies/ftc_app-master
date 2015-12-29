package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by FTCRubies on 11/1/2015.
 * This is one of the first linear programs we created
 */
public class TestLinearBlue extends LinearOpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        sleep (11000);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        sleep(6250);

        leftMotor.setPower(0.7);
        rightMotor.setPower(0.0);

        sleep(1500);

        leftMotor.setPower(0.4);
        rightMotor.setPower(0.4);

        sleep(0500);

        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }
}

