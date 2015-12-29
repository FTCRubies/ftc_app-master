
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by FTCRubies on 11/13/2015.
 */
public class AutonomousLinearRed extends AutonomousLinearBlue {
    AutonomousLinearRed() {
        this.mCurrentColor = AutonomousColors.RED;
    }
}



public class AutonomousLinearBlue extends LinearOpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;

public enum AutonomousColors {
    RED,
    BLUE
};

    public AutonomousColors mCurrentColor;

    public AutonomousLinearBlue() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        sleep(11000);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        sleep(2700);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.0);

        sleep(2400);

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);

        sleep(2000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }
}
