package RoboRaiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ServoDistRobot {

    private BNO055IMU imu;
    private Servo inTake;
 //   public enum inTakeDirections {off, in, out};
    private DistanceSensor sensorRange;


    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    //Robot Constants



    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public ServoDistRobot() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;

        inTake = hwMap.get(Servo.class, "inTakeServo");
  //      inTake.setPosition(0.0);

        //     sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");

        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }


    /**
     * Sets the servo spin direction
     * - 0: off, stop spinning
     * - 1: on, spin clockwise or spin to intake a cone
     * - 2: off, spin counter-clockwise or spin to outtake a cone
     *
     * @param direction - the direction to spin (see above)
     */
    public void activateInTake(int direction)
    {

        switch (direction) {
            case 0:                      // Turn intake off
                inTake.setPosition(0.5);
                break;
            case 1:                       // Turn intake on and spin clockwise
                inTake.setPosition(1.0);
                break;
            case 2:                      // Turn intake on and spin counter-clockwise
                inTake.setPosition(0.0);
                break;
            default:
                break;
        }

    }

    public double getIntakePosition()
    {
        return inTake.getPosition();
    }

    /**
     * Gets the distance from the REV 2m distance sensor
     * @return - the distance in inches
     */
    public double getDistance()
    {
        return sensorRange.getDistance(DistanceUnit.MM);
    }


    //**********************************************************************************************
    //
    // IMU METHODS
    //
    //**********************************************************************************************


    /**
     * Gets the current heading from the IMU
     * @return current heading in degrees
     */
    public float getHeading() {

        float heading;

//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
 //       heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle
        heading = -imu.getAngularOrientation().firstAngle;

        return heading;
    }


    /**
     * Re-initializes the IMU
     */
    public void resetIMU() {

        imu.initialize(parameters);
    }

    /**
     * Calculates and returns an integrate Z-Axis (aka heading).  This handles when the heading crosses
     * 180 or -180
     * @return integrated Z-Axis
     */
    public double getIntegratedZAxis() {

        // This sets up the how we want the IMU to report data
        iza_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Obtain the heading (Z-Axis)
        iza_newHeading = iza_angles.firstAngle;

        // Calculate the change in the heading from the previous heading
        iza_deltaHeading = iza_newHeading - iza_lastHeading;

        // Bosch IMU wraps at 180, so compensate for this
        if (iza_deltaHeading <= -180.0) {

            iza_deltaHeading += 360.0;
        }
        else if (iza_deltaHeading >= 180.0) {

            iza_deltaHeading -= 360;
        }

        // Calculate the integratedZAxis
        integratedZAxis += iza_deltaHeading;

        // Save the current heading for the next call to this method
        iza_lastHeading = iza_newHeading;

        return integratedZAxis;
    }

    //**********************************************************************************************
    //
    // END IMU METHODS
    //
    //**********************************************************************************************





}


