package RoboRaiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MotorBotWithDCMOTOR {

    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor aMotor = null;

    public BNO055IMU imu;

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

    public static double autoheading;

    //Robot Constants



    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public MotorBotWithDCMOTOR() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;
        // Define and initialize motors
        aMotor = hwMap.get(DcMotor.class, "aMotor");

        // Defines the directions the motors will spin
        aMotor.setDirection(DcMotor.Direction.FORWARD);


        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //have the motors on the drivetrain break here.
        // Set all motors to zero power
        aMotor.setPower(0.0);

        // Stop and reset encoders
        resetEncoders();

        // Set all motors to run without encoders.
        runWithoutEncoders();

        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }

    //**********************************************************************************************
    //
    // MOTOR METHODS
    //
    //**********************************************************************************************

    /**
     * This method will set the power for the drive motors
     *
     * @param power  power setting for the left front motor

     */
    public void setDriveMotorPower(double power) { aMotor.setPower(power); }    /**
     * This method will set the power for the drive motors
     *
     * @param power  power setting for the left front motor

     */
  //  public void setDriveMotorVelocity(double ticksPerSecond) { aMotor.setVelocity(ticksPerSecond); }

    /**
     * calculates the number of encoder counts to travel a given distance for the turret
     * @param distance
     * @return
     */
    public double turretCalculateCounts(double distance) {

        double COUNTS;

        double DIAMETER = 0.875; // diameter of turret
        double GEAR_RATIO = (1.0 / 1.0); // gear ratio

        double PULSES = 1120.0; // encoder counts in one revolution - neverest 40 orbital

        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives rotations
        COUNTS = PULSES * ROTATIONS; //gives counts

        return COUNTS;
    }
    /**
     * Sets the target encoder value for the drive train motors
     * @param encoderPosition
     */
    public void setTargetPosition(int encoderPosition){ aMotor.setTargetPosition(encoderPosition); }

    /**
     * This method will set the mode of all of the drive train motors to run using encoder
     */
    public void runWithEncoders() { aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }

    /**
     * This method will set the mode all of the drive train motors to RUN_TO_POSITION
     */
    public void runWithEncodersSTP() { aMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
    /**
     * This method will set the mode of all of the drive train motors to run without encoder
     */
    public void runWithoutEncoders() { aMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }

    /**
     * This will set the mode of the drive train motors to STOP_AND_RESET_ENCODER, which will zero
     * the encoder count but also set the motors into a RUN_WITHOUT_ENCODER mode
     */
    public void resetEncoders() { aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

    /**
     * These methods will get individual encoder position from any of the drive train motors
     * @return the encoder position
     */
    public double getEncoderCounts() { return aMotor.getCurrentPosition(); }


    //**********************************************************************************************
    //
    // IMU METHODS
    //
    //**********************************************************************************************


    /**
     * Gets the current heading from the IMU
     * @return current heading in degrees
     */
    public float getHeadingZ() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

        return heading;
    }
    /**
     * Gets the current heading from the IMU
     * @return current heading in degrees
     */
    public float getHeadingY() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

        return heading;
    }
    /**
     * Gets the current heading from the IMU
     * @return current heading in degrees
     */
    public float getHeadingX() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

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


