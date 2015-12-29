package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

//------------------------------------------------------------------------------
// A2818_StateMachine.java
//------------------------------------------------------------------------------
// Extends the OpMode class to provide a Example Autonomous code
//------------------------------------------------------------------------------
/* This opMode does the following steps:
 * 0) Wait till the gyro is calibrated.
 * 1)Drives forward using gyro to correct
 * 2) Stops and waits for end of Auto
 *
 * The code is executed as a state machine.  Each "State" performs a specific task which takes time to execute.
 * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on to next state
 */

public class AutonomousBase extends OpMode {
    // A list of system States.
    private enum State
    {
        STATE_INITIAL,
        STATE_GYRO,
        STATE_BACKUP,
        STATE_TURN,
        STATE_FIND_ANGLE,
        STATE_FIND_LINE,
        STATE_PRESS_BUTTON,
        STATE_DUMP_CLIMBERS1,
        STATE_DUMP_CLIMBERS2,
        STATE_STOP
    }
    public enum Alliance_Color {
        RED,
        BLUE
    }
    final double BASE_SPEED = 1;
    final double CORRECTION = 13;
    final double SERVO_OUT = 1;
    final double SERVO_IN = 0;
    final int BASE_TURN = 45;
    final double COUNTS_PER_INCH = 720;

    final PathSeg[] mBackupPath = {
            new PathSeg(4.0, 4.0, 0.7),
    };


    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    public DcMotor      mLeftMotor;
    public DcMotor      mRightMotor;
    public ModernRoboticsI2cGyro   gyroSensor;
    public OpticalDistanceSensor    opticalSensor;
    public UltrasonicSensor     ultrasonicSensor;
    public TouchSensor      touchSensor;
    public ModernRoboticsI2cColorSensor     lineSensor;
    public ModernRoboticsI2cColorSensor     beaconSensor;
    public Servo    sideServo;
    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    private State       mCurrentState;    // Current State Machine State.
    private int         heading;
    public Alliance_Color     mCurrentAlliance;
    private int         mLeftEncoderTarget;
    private int         mRightEncoderTarget;



    //--------------------------------------------------------------------------
    // Demo Hardware
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init()
    {
        // Initialize class members.
        mLeftMotor  = hardwareMap.dcMotor.get("left_drive");
        mRightMotor = hardwareMap.dcMotor.get("right_drive");
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);

        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        opticalSensor = hardwareMap.opticalDistanceSensor.get("sensor_EOPD");
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonic");
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        //change name to what is on the phone later
        lineSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        gyroSensor.calibrate();
        lineSensor.enableLed(true);
        lineSensor.enableLed(false);
        sideServo.setPosition(SERVO_IN);


    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop()
    {

    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start()
    {
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0);        // Set target speed to zero
        mRuntime.reset();           // Zero game clock
        newState(State.STATE_INITIAL);
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop()
    {
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());

        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //
        switch (mCurrentState)
        {
            case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
                if (gyroSensor.isCalibrating())
                {
                    break;
                }
                else{
                    // Display Diagnostic data for this state.
                    heading = gyroSensor.getIntegratedZValue()+(mCurrentAlliance == Alliance_Color.RED ? BASE_TURN : - BASE_TURN);
                    telemetry.addData("Integrated ", gyroSensor.getIntegratedZValue());
                    setDriveSpeed(BASE_SPEED,BASE_SPEED);
                    newState(State.STATE_GYRO);
                }
                break;

            case STATE_GYRO: // Follow path until last segment is completed
                if (touchSensor.isPressed())
                {
                    newState(State.STATE_BACKUP);
                }
                else {

                    double difference = gyroSensor.getIntegratedZValue()-heading;
                    setDriveSpeed(BASE_SPEED+CORRECTION *difference/100.0, BASE_SPEED-CORRECTION *difference/100.0);
                    telemetry.addData("Integrated ", gyroSensor.getIntegratedZValue());
                    telemetry.addData("Right speed ", mRightMotor.getPower());
                    telemetry.addData("Left speed ", mLeftMotor.getPower());

                }
                    break;

            case STATE_BACKUP: // Follow path until last segment is completed
                if (mStateTime.time()<1.0)
                {
                    setDriveSpeed(-1, -1);
                }
                else {
                    if (mCurrentAlliance == Alliance_Color.RED) {
                        setDriveSpeed(-0.1, 0.1);
                    } else {
                        setDriveSpeed(0.1, -0.1);
                    }
                    newState(State.STATE_FIND_ANGLE);
                    telemetry.addData("State Time ", mStateTime.time());
                }

                break;

            case STATE_FIND_ANGLE: // Follow path until last segment is completed
                if (gyroSensor.getIntegratedZValue() >= BASE_TURN && mCurrentAlliance == Alliance_Color.RED

                        ||gyroSensor.getIntegratedZValue() >= -BASE_TURN && mCurrentAlliance == Alliance_Color.BLUE)
                {
                    setDriveSpeed(0.5,0.5);
                    newState(State.STATE_FIND_LINE);
                } else {
                    telemetry.addData("Angle ", gyroSensor.getIntegratedZValue());

                }
                break;
            case STATE_FIND_LINE: // Follow path until last segment is completed
                if (lineSensor.//white)
                {
                    setDriveSpeed(0.5,0.5);
                    newState(State.STATE_PRESS_BUTTON);
                } else {
                double difference = gyroSensor.getIntegratedZValue()-(heading+(mCurrentAlliance == Alliance_Color.RED ? BASE_TURN : -BASE_TURN ));
                setDriveSpeed(BASE_SPEED+CORRECTION *difference/100.0, BASE_SPEED-CORRECTION *difference/100.0);
                telemetry.addData("Integrated ", gyroSensor.getIntegratedZValue());
                telemetry.addData("Right speed ", mRightMotor.getPower());
                telemetry.addData("Left speed ", mLeftMotor.getPower());;

                }
                break;
            case STATE_PRESS_BUTTON:         // Stay in this state until encoders are both Zero.
                if (lineSensor.//white;)
                {
                    if (mCurrentAlliance == Alliance_Color.BLUE){
                        if(beaconSensor.blue()> beaconSensor.red()){
                            sideServo.setPosition(SERVO_OUT);
                            newState(State.STATE_DUMP_CLIMBERS1);
                        }
                        else {
                            newState(State.STATE_DUMP_CLIMBERS2);

                        }

                    }
                    else {
                        if (beaconSensor.red()>beaconSensor.blue()){
                            sideServo.setPosition(SERVO_OUT);
                            newState(State.STATE_DUMP_CLIMBERS1);
                        }
                        else {

                        }
                    }

                    }





                    /*double difference2 = DISTANCE-ultrasonicSensor.getUltrasonicLevel();
                    setDriveSpeed(BASE_SPEED + CORRECTION * difference2 / 100.0, BASE_SPEED - CORRECTION * difference2 / 100.0);
                    telemetry.addData("Distance ", ultrasonicSensor.getUltrasonicLevel());
                    telemetry.addData("Right speed ",mRightMotor.getPower());
                    telemetry.addData("Left speed ",mLeftMotor.getPower());
                    telemetry.addData("Color value ",opticalSensor.getLightDetected());
                    */
                }
                else{
                    // Display Diagnostic data for this state.
                    newState(State.);
                }



            case STATE_STOP:
                useConstantPower();
                setDriveSpeed(0,0);

                break;
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop()
    {
        // Ensure that the motors are turned off.
        useConstantPower();
        setDrivePower(0, 0);
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }


    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        mLeftMotor.setPower(Range.clip(leftPower, -1, 1));
        mRightMotor.setPower(Range.clip(rightPower, -1, 1));
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (mLeftMotor.getChannelMode() != mode)
            mLeftMotor.setChannelMode(mode);

        if (mRightMotor.getChannelMode() != mode)
            mRightMotor.setChannelMode(mode);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition()
    {
        return mLeftMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return mRightMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------

}

