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


    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */


    /* Public Constants */
    public double turret_home = 0.0;
    public double turret_right = 94.5; // 1/4 of a turn
    public double turret_left = -94.5; // 1/4 of a turn
    public double turret_back = -192.0; // 1/2 of a turn was -185, -145, -135, -192, -220
    public double turretFinalPosition;

    public double lift_ground = 150.0;
    public double lift_high = 7550.0;
    public double lift_middle = 5950.0;
    public double lift_middle_deposit = 5392;
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

        turretMotor.setPower(0.0);
        liftMotor.setPower(0.0);

        // Stop and reset encoders
        turretResetEncoders();
        liftResetEncoders();

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define and initialize the servo
        inTake = hwMap.get(Servo.class, "inTakeServo");
        setinTakeServoPosition(0.0); // set to intake cone position


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

    public void setTurretPositionHome() {setTurretMotorTargetPosition(turret_home);}
    public void setTurretPositionBack() {setTurretMotorTargetPosition(turret_back);}
    public void setTurretPositionRight() {setTurretMotorTargetPosition(turret_right);}
    public void setTurretPositionLeft() {setTurretMotorTargetPosition(turret_left);}



    //**********************************************************************************************
    //
    // END TURRET METHODS
    //
    //**********************************************************************************************

    //**********************************************************************************************
    //
    // LIFT METHODS
    //
    //**********************************************************************************************

    public void   liftResetEncoders() { liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

    public void   setLiftPower(double liftPower) { liftMotor.setPower(liftPower); }
    public void   setLiftMotorTargetPosition(double liftPosition) { liftMotor.setTargetPosition((int)liftPosition); }
    public void   setLiftMotorVelocity(double liftVelocity) { liftMotor.setVelocity(liftVelocity); }
    public void   setLiftRunWithEncodersSTP() { liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
    public void   setLiftRunWithEncoders() { liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }
    public void   setLiftRunWithoutEncoders() { liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }

    public void   setLiftPositionGround() { setLiftMotorTargetPosition(-lift_ground); }
    public void   setLiftPositionLow() { setLiftMotorTargetPosition(-lift_low); }
    public void   setLiftPositionMid() { setLiftMotorTargetPosition(-lift_middle); }
    public void   setLiftPositionMidDeposit() {setLiftMotorTargetPosition(-lift_middle_deposit);}
    public void   setLiftPositionHigh() { setLiftMotorTargetPosition(-lift_high); }


    public double getLiftEncoderCounts() { return liftMotor.getCurrentPosition(); }


    //**********************************************************************************************
    //
    // END LIFT METHODS
    //
    //**********************************************************************************************

    //**********************************************************************************************
    //
    // SERVO METHODS
    //
    //**********************************************************************************************

    /**
     * this sets the servo position for our intake
     *
     * @param servoPosition - servo position
     *                      - 0.5 center position
     *                      - 0.0 in take cone
     *                      - 1.0 deposit cone
     *
     */
    public void setinTakeServoPosition(double servoPosition){
        inTake.setPosition(servoPosition);
    }

    public double getInTakePosition() { return inTake.getPosition(); }


    //**********************************************************************************************
    //
    // END SERVO METHODS
    //
    //**********************************************************************************************



}


