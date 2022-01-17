/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.MaxbotixUltrasonicI2c;
//import org.firstinspires.ftc.teamcode.sensors.ModernRoboticsRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import org.firstinspires.ftc.teamcode.test.teleOP.Limit_Switch_Test;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative_MINIBOT and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotSetup {

    /* Define Motor references ------------------------------------------------------------------*/
    private DcMotor  LFDrive     = null;        // Front left wheel
    private DcMotor  RFDrive     = null;        // Front right wheel
    private DcMotor  LRDrive     = null;        // Back left wheel
    private DcMotor  RRDrive     = null;        // Back right wheel


    /* Define Servo references ------------------------------------------------------------------*/
    private CRServo rotateArm       = null;
    private Servo   shoulderArm     = null;
    private Servo   elbowArm        = null;
    private Servo   armClaw         = null;

    private MaxbotixUltrasonicI2c proximityFront = null;
    private MaxbotixUltrasonicI2c proximityBack = null;
    private MaxbotixUltrasonicI2c proximityLeft = null;
    private MaxbotixUltrasonicI2c proximityRight = null;

    private DistanceSensor front_distance = null;
    private ModernRoboticsI2cRangeSensor back_distance = null;

    /**
     * Creates an instance of the Hardware Nut class
     */
    public RobotSetup(){
    }

    /* Motor getters ----------------------------------------------------------------------------*/
    public DcMotor getLeftFrontDrive() { return LFDrive; }

    public DcMotor getRightFrontDrive() {
        return RFDrive;
    }

    public DcMotor getLeftRearDrive() {
        return LRDrive;
    }

    public DcMotor getRightRearDrive() {
        return RRDrive;
    }




    /* Servo getters ----------------------------------------------------------------------------*/
    public CRServo getRotateArm() {
        return rotateArm;
    }

    public Servo getShoulderArm() {
        return shoulderArm;
    }

    public Servo getElbowArm() {
        return elbowArm;
    }

    public Servo getArmClaw() {
        return armClaw;
    }


    public MaxbotixUltrasonicI2c getProximityFront() {return proximityFront;}
    public MaxbotixUltrasonicI2c getproximityBack() {return proximityBack;}
    public MaxbotixUltrasonicI2c getProximityLeft() {return proximityLeft;}
    public MaxbotixUltrasonicI2c getProximityRight() {return proximityRight;}

    public DistanceSensor getFront_distance() {return front_distance;}
    public ModernRoboticsI2cRangeSensor getBack_distance() {return back_distance;}

    public double[] getArmPositions() {
        return new double[]{ shoulderArm.getPosition(), elbowArm.getPosition(), armClaw.getPosition()};
    }

    /* setMotorMode()
    Set up the motors to run in a particular mode, such as:
        RUN_WITHOUT_ENCODER - just run but do not use encoders at all
        RUN_USING_ENCODERS - run using encoders you can check to see haow far motors have run
        RUN_TO_POSITION - use encoders so you can run to a particular position based on encoder reading
     */
    public void setMotorMode(DcMotor.RunMode mode) {
        LFDrive.setMode(mode);
        RFDrive.setMode(mode);
        LRDrive.setMode(mode);
        RRDrive.setMode(mode);
    }


    /* setMotorToBrakeOnStop()
        Set up the motors to BRAKE when they stop, such as:
            true:  motors turn on brake when they stop (DcMotor.ZeroPowerBehavior.BRAKE)
            false: motors free spin when they are finished running
         */
    public void setMotorToBrakeOnStop(boolean mode) {
        if (mode == true) {
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT );
        }
    }

    /* setMotorForwardDirection()
       Set the direction of each motor based on the direction the robot should go
           true: robot moves forward
           false: robot moves backwards
       Because the motors are set up on opposite sides, one motor's forward is the robot's reverse
       and visa-versa
     */
    public void setMotorForwardDirection(boolean mode) {
        if (mode == true) {  // move robot FORWARD
            LFDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            RFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            RRDrive.setDirection(DcMotorSimple.Direction.REVERSE );
        } else {   // Move robot BACKWARD
            LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            RFDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            LRDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            RRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }


    public void startAll(double power) {
        LFDrive.setPower(power);
        RFDrive.setPower(power);
        LRDrive.setPower(power);
        RRDrive.setPower(power);
    }

    public void stopAll() {
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LRDrive.setPower(0);
        RRDrive.setPower(0);
    }

    /* Functional methods -----------------------------------------------------------------------*/

    /**
     * Initialize all standard hardware interfaces
     *
     * @param hwMap - reference to the hardware map on the user interface program
     */
    public void init(final HardwareMap hwMap) {

        /* INITIALIZE MOTORS --------------------------------------------------------------------*/

        /* Wheel Motors */

        //Front right motor -- set on Motor #0
        RFDrive = hwMap.get(DcMotor.class, "rf_drivemotor");

        //Front left motor -- set on Motor #1
        LFDrive  = hwMap.get(DcMotor.class, "lf_drivemotor");

        //Back right motor -- set on Motor #2
        RRDrive   = hwMap.get(DcMotor.class, "rr_drivemotor");

        //Back left motor -- set on Motor #3
        LRDrive    = hwMap.get(DcMotor.class, "lr_drivemotor");


        LFDrive.setDirection(DcMotor.Direction.REVERSE);  // Set to REVERSE if using AndyMark motors
        RFDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        LRDrive.setDirection(DcMotor.Direction.REVERSE);
        RRDrive.setDirection(DcMotor.Direction.FORWARD);



               // Set all motors to zero power
        /* SET INITIAL POWER --------------------------------------------------------------------*/
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LRDrive.setPower(0);
        RRDrive.setPower(0);


        /* SET MOTOR MODE -----------------------------------------------------------------------*/
        LFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //* INITIALIZE SERVOS --------------------------------------------------------------------*//*

        //Left hook for the build platform
        rotateArm = hwMap.get(CRServo.class, "rotate_arm");

        // Shoulder of the Arm on the platform
        shoulderArm = hwMap.get(Servo.class, "shoulder_arm");

        //Right hook for the build platform
        elbowArm = hwMap.get(Servo.class, "elbow_arm");

        //Left servo for the autonomous arm
        //armClaw = hwMap.get(Servo.class, "arm_claw");


        //proximityFront = hwMap.get(MaxbotixUltrasonicI2c.class, "proximityF");
        //proximityBack = hwMap.get(MaxbotixUltrasonicI2c.class, "proximityB");
        proximityLeft = hwMap.get(MaxbotixUltrasonicI2c.class, "left_ultrasonic");
        //proximityRight = hwMap.get(MaxbotixUltrasonicI2c.class, "proximityR");


        front_distance = hwMap.get(DistanceSensor.class, "front_distance");
        back_distance = hwMap.get(ModernRoboticsI2cRangeSensor.class, "back_distance");

    }
}