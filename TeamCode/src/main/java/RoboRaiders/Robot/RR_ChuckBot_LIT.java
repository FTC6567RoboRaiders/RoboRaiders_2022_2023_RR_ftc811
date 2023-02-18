package RoboRaiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import RoboRaiders.Utilities.Logger.Logger;

public class RR_ChuckBot_LIT {



    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotorEx turretMotor = null;
    public DcMotorEx liftMotor = null;
    private Servo inTake;
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

    /* Public Constants */
    public double turret_home = 0.0;
    public double turret_right = 94.5; // 1/4 of a turn
    public double turret_left = -94.5; // 1/4 of a turn
    public double turret_back = -185.0; // 1/2 of a turn
    public double turretFinalPosition;

    public double lift_ground = 150.0;
    public double lift_high = 7550.0;
    public double lift_middle = 5950.0;
    public double lift_low = 3050.0;

    public static double robotHeading;
    public boolean firstTimeCalled = true;

    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public RR_ChuckBot_LIT() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        liftMotor = hwMap.get(DcMotorEx.class, "liftMotor");

        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setPower(0.0);
        liftMotor.setPower(0.0);

        // Stop and reset encoders
        turretResetEncoders();
        liftResetEncoders();

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // define and initialize the servo
        inTake = hwMap.get(Servo.class, "inTakeServo");
        setinTakeServoPosition(0.5); // set to home position

        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }

    //**********************************************************************************************
    //
    // TURRET METHODS
    //
    //**********************************************************************************************

    /**
     * calculates the number of encoder counts to travel a given distance for the turret
     * @param distance
     * @return
     */
    public double turretCalculateCounts(double distance) {

        double COUNTS;

        double DIAMETER = 3.5; // diameter of turret
        double GEAR_RATIO = (3.0 / 4.0); // gear ratio

        double PULSES = 288.0; // encoder counts in one revolution - core hex motor

        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives rotations
        COUNTS = PULSES * ROTATIONS; //gives counts

        return COUNTS;

    }

    /**
     * Sets the target encoder value for the drive train motors
     * @param encoderPosition
     */
    public void setTurretMotorTargetPosition(double encoderPosition) {

        turretMotor.setTargetPosition((int)encoderPosition);

    }

    /**
     * This method will set the mode of all of the drive train motors to run using encoder
     */
    public void turretRunWithEncoders() {

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * This method will set the mode all of the drive train motors to RUN_TO_POSITION
     */
    public void turretRunWithEncodersSTP() {

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * This method will set the mode of all of the drive train motors to run without encoder
     */
    public void turretRunWithoutEncoders() {

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * This will set the mode of the drive train motors to STOP_AND_RESET_ENCODER, which will zero
     * the encoder count but also set the motors into a RUN_WITHOUT_ENCODER mode
     */
    public void turretResetEncoders() {

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turretSetMotorTargetPosition(int encoderPosition) {

        turretMotor.setTargetPosition(encoderPosition);

    }
    public void setTurretMotorPower(double turretMotorPower) {

        turretMotor.setPower(turretMotorPower);

    }
    public void setTurretMotorVelocity(double ticsPerSecond) {

        turretMotor.setVelocity(ticsPerSecond);

    }

    /**
     * These methods will get individual encoder position from any of the drive train motors
     * @return the encoder position
     */
    public double getTurretEncoderCounts() {

        return turretMotor.getCurrentPosition();

    }
    //**********************************************************************************************
    //
    // LIFT METHODS
    //
    //**********************************************************************************************
    public void liftResetEncoders() {

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setLiftPower(double liftPower) { liftMotor.setPower(liftPower); }
    public void setLiftMotorTargetPosition(double liftPosition) { liftMotor.setTargetPosition((int)liftPosition); }
    public void setLiftMotorVelocity(double liftVelocity) { liftMotor.setVelocity(liftVelocity); }
    public double getLiftEncoderCounts() { return liftMotor.getCurrentPosition(); }
    public void liftRunWithEncodersSTP() { liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
    public void liftRunWithEncoders() { liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
    public void liftRunWithoutEncoders() { liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }
    public void setLiftMotorPower(double liftPower) { liftMotor.setPower(liftPower); }


    //**********************************************************************************************
    //
    // END DRIVE TRAIN METHODS
    //
    //**********************************************************************************************

    //**********************************************************************************************
    //
    // Servo METHODS
    //
    //**********************************************************************************************

    /**
     * this sets the servo position for our intake
     *
     * @param servoPosition - servo position
     *                      - 0.5 center position
     *                      - 0.0 intake position
     *                      - 1.0 scoring position
     *
     */
    public void setinTakeServoPosition(double servoPosition){
        inTake.setPosition(servoPosition);
    }


    //**********************************************************************************************
    //
    // End Servo METHODS
    //
    //**********************************************************************************************

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


