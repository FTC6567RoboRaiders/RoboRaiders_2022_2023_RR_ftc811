package RoboRaiders.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robot.ServoDistRobot;

// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name="Steve's Distance Servo Teleop", group="Test Teleops")

//
// Purpose: Demonstrate how to stop a servo from spinning when a distance sensor detects
//          an obstruction that is 1/4 of an inch or less.
//
//          GAMEPAD1.A - spin servo clockwise
//          GAMEPAD1.B - spin servo counter-clockwise
//
// Note:    Spinning servo clockwise will be "tied" to the distance sensor, spinning the
//          servo counter-clockwise will not be "tied" to the distance sensor.

public class ServoDistance extends OpMode {

    // Create an instance of the TestRobot and store it into StevesRobot
    public ServoDistRobot stevesRobot = new ServoDistRobot();

    @Override
    public void init() {

        // Initialize stevesRobot and tell user that the robot is initialized
        stevesRobot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }

    @Override
    public void loop() {


        telemetry.addData("gamepad1.a: ",gamepad1.a);
        telemetry.addData("gamepad1.b: ",gamepad1.b);
        telemetry.addData("gamepad1.y: ",gamepad1.y);
      //  telemetry.addData("distance: ",stevesRobot.getDistance());
        telemetry.addData("inTake position: ", stevesRobot.getIntakePosition());

        if (gamepad1.a)
        {
       //     if (stevesRobot.getDistance() < 20.0)
       //     {
                telemetry.addData("Setting direction: 1", true);
                stevesRobot.activateInTake(1);
        //    }
            //     else
//            {
//                telemetry.addData("Setting direction: 1", true);
//                stevesRobot.activateInTake(1);
        //    }
        }
        else if (gamepad1.b)
        {
            telemetry.addData("Setting direction: 2", true);
            stevesRobot.activateInTake(2);
        }
        else if (gamepad1.y)
        {
            telemetry.addData("Setting direction: 0", true);
            stevesRobot.activateInTake(0);
        }
//        else {
//            telemetry.addData("Setting direction: 0", true);
//            stevesRobot.activateInTake(0);
//        }

    }
    /**
     * smoothPower will attempt to smooth or scale joystick input when driving the
     * robot in teleop mode.  By smoothing the joystick input more controlled movement
     * of the robot will occur, especially at lower speeds.
     * <br><br>
     * To scale the input, 16 values are used that increase in magnitude, the algorithm
     * will determine where the input value roughly falls in the array by multiplying it
     * by 16, then will use the corresponding array entry from the scaleArray variable to
     * return a scaled value.
     * <br><br>
     * <b>Example 1:</b> dVal (the input value or value passed to this method) is set to 0.76
     * <br>
     * Stepping through the algorithm
     * <ol>
     * <li> 0.76*16 = 12.16, but because we cast the calculations as an integer (int)
     * we lose the .16 so the value just is 12, variable index now contains 12.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index contains a positive 12</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially 0.76 so not negative</li>
     * <li> If dVal (value passed to this method) was initially positive, then
     * set the variable dScale to the scaleArray(index), since index is 12, then
     * scaleArray(12) = 0.60.  <b>Remember, in java the first array index is 0,
     * this is why scaleArray(12) is not 0.50</b></li>
     * <li> Return the dScale value (0.60)</li>
     * </ol>
     * <p>
     * <br><br>
     * <b>Example 2</b> dVal (the input value or value passed to this method) is set to -0.43
     * <br>
     * Stepping through the algorithm
     * <ol>
     * <li> -0.43*16 = -6.88, but because we cast the calculations as an integer (int)
     * we lose the .88 so the value just is -6, variable index now contains -6.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index is negative, so make the negative a negative (essentially
     * multiplying the variable index by -1, the variable index now contains 6</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially -0.43, so make sure to return a negative value of scaleArray(6).
     * scaleArray(6) is equal to 0.18 and the negative of that is -0.18 <b>Remember,
     * in java the first array index is 0, this is why scaleArray(6) is not 0.15</b></li>
     * <li> Return the dScale value (-0.18)</li>
     * </ol>
     *
     * @param dVal the value to be scaled -between -1.0 and 1.0
     * @return the scaled value
     * <B>Author(s)</B> Unknown - copied from internet
     */
    double smoothPower(double dVal) {
        // in the floats.
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        // index should be positive.
        if (index < 0) {
            index = -index;
        }
        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }
        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        // return scaled value.
        return dScale;
    }
}
