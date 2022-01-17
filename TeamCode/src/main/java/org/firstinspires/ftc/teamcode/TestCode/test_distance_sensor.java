package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RobotSetup;

@TeleOp(name="Distance Sensor Test", group="Dave")
public class test_distance_sensor extends OpMode {

    private final RobotSetup robot = new RobotSetup();   // Get the Hardware Setup for this robot

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); // the hardwareMap comes from OpMode class and represents the data from the Android COnfiguration of the robot
    }

    @Override
    public void loop() {
        try {
            super.wait(5000);
        } catch (Exception e) {
            // ignore any thow
        }

        robot.getProximityLeft().measureRange();

        try {
            super.wait(120);
        } catch (Exception e) {
            // ignore any thow
        }

        double front = robot.getFront_distance().getDistance(DistanceUnit.CM);
        double back = robot.getBack_distance().cmUltrasonic();
        short left = robot.getProximityLeft().getLastRange();

        telemetry.addData("distance", "front (%.2f), back (%.2f), left (%d)", front, back, left);
        telemetry.update();
    }
}
