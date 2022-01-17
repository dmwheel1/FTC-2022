package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FusionFramework.SimpleSensorFusion;
import org.firstinspires.ftc.teamcode.common.RobotSetup;
import org.firstinspires.ftc.teamcode.FusionFramework.DriveTrainIntf;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import org.firstinspires.ftc.teamcode.FusionFramework.CommandRecorder;

/**
 * Created 10/18/2017
 *
 * This Opmode is used to control the robot during the teleOp period of the competition
 *
 * @author Anna Field
 */
@TeleOp(name="Dave TeleOp Test", group="Dave")

public class Dave_Teleop_Test extends OpMode {

    private final RobotSetup robot = new RobotSetup();   // Get the Hardware Setup for this robot
    private DriveTrainIntf drivetrain;
    private SimpleSensorFusion sensorfusion;

    private double LastFrontMeasure;
    private double LastBackMeasure;

    private CommandRecorder cr;

    private ElapsedTime tm_arm = null;
    private ElapsedTime tm_elbow = null;

    Telemetry.Item msgItem ;
    Telemetry.Item frontSensor ;
    Telemetry.Item backSensor ;
    /**
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); // the hardwareMap comes from OpMode class and represents the data from the Android COnfiguration of the robot

        sensorfusion = new SimpleSensorFusion(robot);

        drivetrain = new DriveTrainIntf(robot);


        //Reset encoders
        turnOnStopAndReset();

        //Turn on RUN_USING_ENCODER
        turnOnRunUsingEncoder();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Sydney");

    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        LastFrontMeasure = sensorfusion.getFrontDistance();
        LastBackMeasure = sensorfusion.getBackDistance();

        tm_arm = new ElapsedTime(MILLISECONDS);
        tm_arm.reset();
        tm_elbow = new ElapsedTime(MILLISECONDS);
        tm_elbow.reset();

        telemetry.setAutoClear(false);
        msgItem  = telemetry.addData("Socket Msg: ", "No message yet");
        frontSensor = telemetry.addData("Distance Front: ", 0.0);
        backSensor = telemetry.addData("Distance Front: ", 0.0);

    }

    @Override
    public void stop() {
        // try to garbage collect the COmmandRecorder so the file gets closed
        cr.finalize();
        super.stop();
    }

    private void turnOnRunUsingEncoder() {
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnOnStopAndReset() {
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
    private void drive(final double drive, final double strafe, final double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        robot.getLeftFrontDrive().setPower(.9 * frontLeftPower);
        robot.getRightFrontDrive().setPower(.9 * frontRightPower);
        robot.getLeftRearDrive().setPower(.9 * backLeftPower);
        robot.getRightRearDrive().setPower(.9 * backRightPower);
    }
    */

    @Override
    public void loop() {


        /* DRIVE ROBOT --------------------------------------------------------------------------*/
        /*
        double drive = -gamepad1.left_stick_y;   // Power for forward and back motion; Negative because the gamepad is weird
        double strafe = 0; // NO STRAFE ON THIS ROBOT -- gamepad1.left_stick_x;  // Power for left and right motion
        double rotate = gamepad1.right_stick_x;  // Power for rotating the robot

        drive(drive, strafe, rotate);
        */

        double Lpower = gamepad1.left_stick_y;
        double Rpower = gamepad1.right_stick_y;
        telemetry.addData("Speed: ", "L:%.2f  R:%.2f", Lpower, Rpower);


        double front = sensorfusion.getFrontDistance();
        frontSensor.setValue(front);
        double back  = sensorfusion.getBackDistance();
        backSensor.setValue(back);


        // Before doing the next command, make sure to finish the last command we wrote to the file


        // TODO: Read the back wheels encoder values
        // backRightEncoder = ???
        // backLeftEncoder = ???

        // TODO: write the encoder values to the file (we have to change the CommandRecord to do this)

        // Complete the last command and write to the file
        cr.finishLastCommand(); // will only write if there was a first command


        // double back = sensorfusion.
        if (front < LastFrontMeasure && front < 6 && (Lpower > 0 || Rpower > 0)) {
            telemetry.addLine("front distance sensor stopped forward movement");
        } else if ( back < LastBackMeasure && back < 6 && (Lpower < 0 || Rpower <   0)) {
            telemetry.addLine("back distance sensor stopped backward movement");
        } else {
            // record this command
            cr.writeNewCommand("drivetrain.Drive( " + Lpower + ", " + Rpower + ", ");

            // TODO: Read the back wheels encoder values and save to a class member variable
            // m_backRightEncoder = ???
            // m_backLeftEncoder = ???

            drivetrain.Drive(Lpower, Rpower);
            LastFrontMeasure = front;
            LastBackMeasure = back;
        }

        //control the ARM rotation
        if (gamepad1.right_trigger > .1) { // move clockwise
            robot.getRotateArm().setPower(-gamepad1.right_trigger * 1.0);
            telemetry.addData("Rotate Arm Left", 2);

        } else if (gamepad1.left_trigger  > .1) {  // move counter-clockwise
            robot.getRotateArm().setPower(gamepad1.left_trigger * 1.0);
            telemetry.addData("Rotate Arm Right", 2);

        } else {
            robot.getRotateArm().setPower(0);
        }

        // Check for arm movement
        if (tm_arm.time() > 80) { // only check the button if it hasn't been pressed recently

            if (gamepad1.y) {
                double elbow = robot.getShoulderArm().getPosition();
                if (elbow < 1.0) {
                    robot.getShoulderArm().setPosition(elbow + 0.05);
                    telemetry.addData("Shoulder Up: ", elbow);
                }
                tm_arm.reset(); // reset timer so we know we just processes a button press and shouldn't process one for a few milliseconds

            } else if (gamepad1.x) {
                double elbow = robot.getShoulderArm().getPosition();
                if (elbow > 0.0) {
                    robot.getShoulderArm().setPosition(elbow - 0.05);
                    telemetry.addData("Shoulder Down: ", elbow);
                }
                tm_arm.reset();

            }
        }

        // Check for elbow movement
        if (tm_elbow.time() > 80) { // only check the button if it hasn't been pressed recently

            if (gamepad1.a) {
                double elbow = robot.getElbowArm().getPosition();
                if (elbow < 1.0) {
                    robot.getElbowArm().setPosition(elbow + 0.05);
                    telemetry.addData("Elbow Up: ", elbow);
                }
                tm_elbow.reset(); // reset timer so we know we just processes a button press and shouldn't process one for a few milliseconds


            } else if (gamepad1.b) {
                double elbow = robot.getElbowArm().getPosition();
                if (elbow > 0.0) {
                    robot.getElbowArm().setPosition(elbow - 0.05);
                    telemetry.addData("Elbow Down: ", elbow);
                }
                tm_elbow.reset();

            }
        }
        telemetry.update();
    }
}
