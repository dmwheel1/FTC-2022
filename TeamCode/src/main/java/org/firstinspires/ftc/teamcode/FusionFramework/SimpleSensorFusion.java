package org.firstinspires.ftc.teamcode.FusionFramework;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RobotSetup;
import org.firstinspires.ftc.teamcode.sensors.MaxbotixUltrasonicI2c;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

public class SimpleSensorFusion {

    private MaxbotixUltrasonicI2c leftultrasonic;
    private MaxbotixUltrasonicI2c rightultrasonic;
    private DistanceSensor frontdistance;
    private ModernRoboticsI2cRangeSensor backdistance;

    public SimpleSensorFusion(RobotSetup robot){
        leftultrasonic = robot.getProximityLeft();
        rightultrasonic = robot.getProximityRight();
        frontdistance = robot.getFront_distance();
        backdistance = robot.getBack_distance();
    }

    public double getFrontDistance(){
        return frontdistance.getDistance(DistanceUnit.CM);
    }

    public double getBackDistance(){
        return backdistance.getDistance(DistanceUnit.CM);
    }

    public double getRightDistance(){
        rightultrasonic.measureRange();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            //ignore normal operation
        }
        return rightultrasonic.getLastRange();
    }

    public double getLeftDistance(){
        leftultrasonic.measureRange();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            //ignore normal operation
        }
        return leftultrasonic.getLastRange();
    }
}
