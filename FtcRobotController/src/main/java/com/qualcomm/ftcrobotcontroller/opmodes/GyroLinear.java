

/**
 * Created by FTCRubies on 11/28/2015.
 */





    package com.qualcomm.ftcrobotcontroller.opmodes;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.hardware.ModernRoboticsI2cGyro;
    import com.qualcomm.robotcore.hardware.DcMotor;

public class GyroLinear extends LinearOpMode {



       DcMotor rightMotor;
        DcMotor leftMotor;




        @Override
        public void runOpMode() throws InterruptedException {


            ModernRoboticsI2cGyro sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

            hardwareMap.logDevices();
            leftMotor = hardwareMap.dcMotor.get("leftMotor");

            leftMotor = hardwareMap.dcMotor.get("left_drive");
            rightMotor = hardwareMap.dcMotor.get("right_drive");

// calibrate the gyro.
            sensorGyro.calibrate();
            waitForStart();

// make sure the gyro is calibrated.
            while (sensorGyro.isCalibrating()) {
                Thread.sleep(50);
            }

            while (opModeIsActive()) {
                telemetry.addData("Heading ", sensorGyro.getHeading());
                telemetry.addData("Integrated ", sensorGyro.getIntegratedZValue());

                Thread.sleep(100);

                waitForStart();
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
                sleep(5000);



                while (leftMotor.isBusy() || rightMotor.isBusy()) {

                    }
                }

                            }
        }



