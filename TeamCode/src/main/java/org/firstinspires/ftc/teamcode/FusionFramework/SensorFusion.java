package org.firstinspires.ftc.teamcode.FusionFramework;

import org.firstinspires.ftc.teamcode.common.ArrayQueue;

public class SensorFusion {

    // The following list of values indicate the types of sensors
    // that the SensorFusion Class is capable of using
    // these sensor types must be configured properly
    // using the configureSensors() method
    public enum SensorTypes {
        SENSOR_GYRO ,   // only one Gyro is allowed
        SENSOR_LASER_DISTANCE , // multiple allowed, but each requires a position value
        SENSOR_ULTRASONIC , // multiple allowed, but each requires a position value
        SENSOR_DRIVE_ENCODER , // multiple allowed, but each requires a wheel position value
        SENSOR_ODOMETRY , // multiple allowed, but each requires a wheel position value
    };

    public enum SensorPositionValue {
        SENSOR_POSITION_FRONT,
        SENSOR_POSITION_BACK,
        SENSOR_POSITION_LEFT,
        SENSOR_POSITION_RIGHT,
        SENSOR_POSITION_LEFT_FRONT_WHEEL,
        SENSOR_POSITION_LEFT_BACK_WHEEL,
        SENSOR_POSITION_RIGHT_FRONT_WHEEL,
        SENSOR_POSITION_RIGHT_BACK_WHEEL,
        SENSOR_POSITION_INVALID_VALUE
    }

    class QueueEntry {
        SensorTypes             typ;
        SensorObjectInterface   obj;
        SensorPositionValue     position;
    }

    // The Following are Private Members of the SesnorFusion class
    // used to maintain the set of sensors configured for determining distances and position
    private ArrayQueue<QueueEntry>  leftSensors;
    private ArrayQueue<QueueEntry>  rightSensors;
    private ArrayQueue<QueueEntry>  frontSensors;
    private ArrayQueue<QueueEntry>  backSensors;

    // Constructor for the SensorFusion class
    // Used to initialize the queue to holds sensors
    public SensorFusion()
    {
        leftSensors = new ArrayQueue<>();
        rightSensors = new ArrayQueue<>();
        frontSensors = new ArrayQueue<>();
        backSensors = new ArrayQueue<>();
    }

    public boolean configureSensors( SensorTypes stype, SensorPositionValue position, SensorObjectInterface intf) {
        boolean returnValue = false; // default return indicates failure
        QueueEntry qe;
        // Determine which queue to add this sensor into
        switch ( position ) {
            case SENSOR_POSITION_LEFT:
            case SENSOR_POSITION_LEFT_FRONT_WHEEL:
                // Add this sensor into the LEFT queue
                qe = new QueueEntry(); // create a new entry object to hold this sensor in the queue
                qe.typ = stype; qe.obj = intf; qe.position = position; // initialize the qe object
                leftSensors.add( qe ); // add into the queue
                // check if this is only for the left side
                if ( position == SensorPositionValue.SENSOR_POSITION_LEFT) {
                    // done!
                    returnValue = true; // indicate that we added a sensor to a queue
                    break;
                }
                // for wheels, also add to the front queue, so fall thru to SENSOR_POSITION_FRONT
            case SENSOR_POSITION_FRONT:
            case SENSOR_POSITION_RIGHT_FRONT_WHEEL:
                // Add this sensor into the FRONT queue
                qe = new QueueEntry(); // create a new entry object to hold this sensor in the queue
                qe.typ = stype; qe.obj = intf; qe.position = position; // initialize the qe object
                frontSensors.add( qe ); // add into the queue
                // check if this is only for the front
                if ( position == SensorPositionValue.SENSOR_POSITION_FRONT ||
                     position == SensorPositionValue.SENSOR_POSITION_LEFT_FRONT_WHEEL) {
                    // done!
                    returnValue = true; // indicate that we added a sensor to a queue
                    break;
                }
                // for wheel sensor also add to the right side
            case SENSOR_POSITION_RIGHT:
            case SENSOR_POSITION_RIGHT_BACK_WHEEL:
                // Add this sensor into the LEFT queue
                qe = new QueueEntry(); // create a new entry object to hold this sensor in the queue
                qe.typ = stype; qe.obj = intf; qe.position = position; // initialize the qe object
                rightSensors.add( qe ); // add into the queue
                // check if this is only for the right side
                if ( position == SensorPositionValue.SENSOR_POSITION_RIGHT ||
                     position == SensorPositionValue.SENSOR_POSITION_RIGHT_FRONT_WHEEL) {
                    // done!
                    returnValue = true; // indicate that we added a sensor to a queue
                    break;
                }
                // for wheels, also add to the front queue, so fall thru to SENSOR_POSITION_FRONT
            case SENSOR_POSITION_BACK:
            case SENSOR_POSITION_LEFT_BACK_WHEEL:
                // Add this sensor into the FRONT queue
                qe = new QueueEntry(); // create a new entry object to hold this sensor in the queue
                qe.typ = stype; qe.obj = intf; qe.position = position; // initialize the qe object
                backSensors.add( qe ); // add into the queue
                if ( position == SensorPositionValue.SENSOR_POSITION_LEFT_BACK_WHEEL) {
                    // Add this sensor into the LEFT queue
                    qe = new QueueEntry(); // create a new entry object to hold this sensor in the queue
                    qe.typ = stype; qe.obj = intf; qe.position = position; // initialize the qe object
                    leftSensors.add( qe ); // add into the queue
                }
                returnValue = true; // indicate that we added a sensor to a queue
                break;
            case SENSOR_POSITION_INVALID_VALUE:
                throw new IllegalArgumentException("Sensor position value is invalid");
        }
        // if returnValue is set to false, then we didn't add this sensor to a queue
        // so the caller can verify that the sensor was adding by checking the return value
        return returnValue;
    }

    public int getLeftDistance() {
        int returnValue = -1; // negative number indicates failure to perform action

        // Add implementation here

        return returnValue;
    }

    public int getRightDistance() {
        int returnValue = -1; // negative number indicates failure to perform action

        // Add implementation here

        return returnValue;
    }

    public int getForwardDistance() {
        int returnValue = -1; // negative number indicates failure to perform action

        // Add implementation here

        return returnValue;
    }

    public int getBackDistance() {
        int returnValue = -1; // negative number indicates failure to perform action

        // Add implementation here

        return returnValue;
    }

}
