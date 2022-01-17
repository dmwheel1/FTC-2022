package org.firstinspires.ftc.teamcode.FusionFramework;

import org.firstinspires.ftc.teamcode.common.RobotSetup;

public class DriveTrainIntf {

    private RobotSetup robot;

    // Constructor - call when initializing your OpMode or TeleOp after initializing the RobotSetup
    // must pass in the initialized RobotSetup object
    public DriveTrainIntf(RobotSetup rs) {
        robot = rs;
    }

    // Drive - drive straight in either forward or backward direction
    // forward - positive power ... backward is negative power value
    public void Drive(double power) {

        if (power < 0.0 ) {
            robot.setMotorForwardDirection(false);
            // change power to positive value for the startAll() call
            power = power * -1;
        } else {
            robot.setMotorForwardDirection(true);
        }
        robot.startAll( power );
    }

    public void Drive(double Lpower, double Rpower) {

        if ((Lpower < 0 && Rpower > 0 ) || ( Rpower < 0 && Lpower > 0 )) {
            // Do a turn instead of drive forward/backward
            Turn( Lpower, Rpower);
        } else {
            robot.getLeftFrontDrive().setPower(Lpower);
            robot.getRightFrontDrive().setPower(Rpower);
            robot.getLeftRearDrive().setPower(Lpower);
            robot.getRightRearDrive().setPower(Rpower);
        }
    }

    public void Drive(double LFp, double RFp, double LRp, double RRp) {

    }

    public void Turn( double Lpower, double Rpower) {
        if ( Rpower < 0 && Lpower > 0 ) {
            // Do something for left turn
            robot.getLeftFrontDrive().setPower(0.5*Lpower);
            robot.getLeftRearDrive().setPower(0.5*Lpower);
            robot.getRightRearDrive().setPower(1.2*Rpower);
            robot.getRightFrontDrive().setPower(1.2*Rpower);

        } else {
            // Else do something for a right turn
            robot.getLeftFrontDrive().setPower(1.2*Lpower);
            robot.getLeftRearDrive().setPower(1.2*Lpower);
            robot.getRightRearDrive().setPower(0.5*Rpower);
            robot.getRightFrontDrive().setPower(0.5*Rpower);
        }
    }

    public void Turn(int degrees, double power) {

    }

    public void Stop() {
        robot.stopAll();
    }

    public void DriveTo(int X, int Y, double speed, int timeout) {

    }

    public void DriveTo(int X, int Y, double speed, int faceAngle, int timeout) {

    }

}
