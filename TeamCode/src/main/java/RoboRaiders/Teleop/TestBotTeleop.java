package RoboRaiders.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Properties.RoboRaidersProperties;
import RoboRaiders.Robot.TestRobot;
import RoboRaiders.Utilities.Logger.Logger;
import RoboRaiders.Utilities.RRStopWatch.RRStopWatch;

/**
 * These Things Should Work
 * ========================
 * Will the robot raise the intake before rotating if its in a notch?
 * Will the robot not raise and just rotate the intake if the it is up at a deposit level?
 * Can the robot rotate to all 4 sides?
 * Can the robot deposit at all 4 levels?
 * Can the robot intake and out-take the cone?
 * Will the robot return home at the press of a button?
 * Can the robot be manually controlled by the joysticks?
 * If the robot is resetting home and it lands on one of the four corners, can it be moved manually without the robot getting to the desired encoders? (Can use the manual joysticks while its moving to help it?)
 */


// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name="Steve's TestBot Teleop", group="Test Teleops")

public class TestBotTeleop extends OpMode {


//It is Wade, just curious if I can push from home.
    enum tState {
        turret_start,
        turret_liftMovingToTargetRotatePosition,
        turret_turning,
        turret_deposit,
        turret_returning,
        turret_returningHome
    }
    enum lState{
        lift_start,
        lift_extending,
        lift_deposit,
        lift_retract,


    }

    double turret_home = 0.0;
    double turret_right = 94.5; // 1/4 of a turn
    double turret_left = -94.5; // 1/4 of a turn
    double turret_back = -185.0; // 1/2 of a turn
    double turretFinalPosition;

    double lift_ground = 150.0;
    double lift_high = 7550.0;
    double lift_middle = 5950.0;
    double lift_low = 3050.0;
    double liftFinalPosition;
    double liftRotatePosition;

    // Create an instance of the TestRobot and store it into StevesRobot
    public TestRobot stevesRobot = new TestRobot();
    public Logger myLogger =  new Logger("TestBotTeleop");
    public Logger dtLogger = new Logger("DT");   // Drive train logger

    tState turretState = tState.turret_start;
    lState liftState = lState.lift_start;
    public RRStopWatch myStopWatch = new RRStopWatch();
    boolean yButton = false;
    boolean buttonA_B_X_Pushed = false;
    boolean buttonDpadGround = false;

    @Override
    public void init() {

        // Initialize stevesRobot and tell user that the robot is initialized
        stevesRobot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }


    @Override
    public void loop() {

        /**
         * To Do: Move the drive code into separate method, could also move turret, lift and
         * grabber code into separate methods.
         */
        buttonA_B_X_Pushed = false;
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;


//
//        doTurret();
//        doLift();
        doDrive();



//
//        myLogger.Debug("botheading ", botHeading);
//        myLogger.Debug("x / y / rx ", x, y, rx);
//        myLogger.Debug("rotX / rotY ", rotX, rotY);
//        myLogger.Debug("frontLeftPower / backLeftPower / frontRightPower / backRightPower ",
//                frontLeftPower,
//                backLeftPower,
//                frontRightPower,
//                backRightPower);
//        myLogger.Debug("Y Button", yButton);

        /**
         * To Do: Add some kind of button push to toggle or change the maximum speed of robot
         */


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

    /**
     * Do Turret stuff...
     */
//    public void doTurret(){
//        switch(turretState) {
//
//            // Turret is in the start state
//            case turret_start:
//                myLogger.Debug("TURRET: STARTHERE,STARTHERE,STARTHERE,STARTHERE");
//                myLogger.Debug("turretState: "+turretState);
//                myLogger.Debug("liftState: "+liftState);
//                myLogger.Debug("gamepad2.b "+gamepad2.b);
//                myLogger.Debug("gamepad2.x "+gamepad2.x);
//                myLogger.Debug("gamepad2.a "+gamepad2.a);
//                myLogger.Debug("gamepad2.y "+gamepad2.y);
////                myLogger.Debug("Lift Encoder Counts Compared to Final Position: " + Math.abs(stevesRobot.getLiftEncoderCounts() - liftFinalPosition));
//
//
//                /** Some rambling to get my thoughts together...what is written here was under extreme duress and tiredness
//                 *
//                 * When b, x, a gamepad2 button is pushed have the lift extend 5-6 inches (how many encoder counts??)
//                 * so that the intake mechanism is clear of the chassis.  Once clear of the chassis then we can start
//                 * rotating to the appropriate position.
//                 *
//                 * Note:  I think we need another state here to let the lift come up to the desired level and
//                 * when there start to rotate.  We will only do the liftFinalPosition check once and as it stands
//                 * liftFinalPosition is never set.  So we need a new variable too
//                 *
//                 * So this assumes that the lift is not in any state other than start, i think if the lift is start state this works
//                 * however, if the lift is not in a start state, i think we don't want to "extend" the lift since it is already
//                 * being extended....does this make sense????
//                 *
//                 * what if the lift is extended but below the height that we need to go, do we extend the lift and then rotate? or
//                 * ignore what the "gunner" pushed
//                 *
//                 */
//
//                /**
//                 * So we are gonna turn the turret either now or later, so set the target position and remember it
//                 */
////                stevesRobot.turretRunWithoutEncoders();
////                if (gamepad2.b) {                                                                   // gamepad2.b button pushed
////                    stevesRobot.setTurretMotorTargetPosition(turret_right);                         // Set the target position for the turret
////                    turretFinalPosition = turret_right;                                             // Remember the target position for the turret
////                    buttonA_B_X_Pushed = true;                                                      // Remember that b, x or the a button was pushed
////                }
////                else if (gamepad2.x) {
////                    stevesRobot.setTurretMotorTargetPosition(turret_left);                          // Set the target position for the turret
////                    turretFinalPosition = turret_left;                                              // Remember the target position for the turret
////                    buttonA_B_X_Pushed = true;                                                      // Remember that b, x or the a button was pushed
////                }
////                else if (gamepad2.a) {
////                    stevesRobot.setTurretMotorTargetPosition(turret_back);                          // Set the target position for the turret
////                    turretFinalPosition = turret_back;                                              // Remember the target position for the turret
////                    buttonA_B_X_Pushed = true;                                                      // Remember that b, x or the a button was pushed
////                }
//
//                /**
//                 * So if the lift state is lift_start, that means the lift is in home position.  So start to extending it.
//                 */
////                if (buttonA_B_X_Pushed) {
////                    myLogger.Debug("IT HAS ENTERED BUTTON ABX IF STATEMENT");
////                    if (liftState == lState.lift_start) {                                              // is the lift in start state?
////                        myLogger.Debug("IT HAS ENTERED LIFT STATE BEING IN START");
////                        liftRotatePosition = 3000.0;                                                    // Remember the position to extend the lift to
////                        stevesRobot.setLiftMotorTargetPosition(-3000.0);                                 // Set the position to extend the lift to
////
////                        stevesRobot.liftRunWithEncodersSTP();
////                        stevesRobot.setLiftMotorVelocity(1500.0);                                       // Apply a velocity to the lift motor
////                        turretState = tState.turret_liftMovingToTargetRotatePosition;                  // New state, indicate that the lift is moving to target rotate position
////                    }
//
//                    /**
//                     * Lift state is not lift_start, we can rotate the turret as long as we are above the height of the chassis -AND- we are not retracting the lift
//                     */
////                    else {                                                                             // the lift is not in start state
////                        if (Math.abs(stevesRobot.getLiftEncoderCounts() + 2500) < 5.0 && // the lift is above where we can start to rotate -AND-
////                                liftState != lState.lift_retract) {                                   // the lift is not retracting (coming down)
////                            stevesRobot.turretRunWithEncodersSTP();                                     // Yes, so ensure the turret motor is running with set target position
////                            stevesRobot.setTurretMotorVelocity(600.0);                                 // Apply velocity to turret motor, note that the target position was set above
////                            turretState = tState.turret_turning;                                       // jump to turret_turning state since we don't need to wait for the lift
////                        }
////                    }
//                }                                                                                   // if a, b, or x button pushed
//
//                break;
//
//            // the lift is extending to the target rotate position
//            case turret_liftMovingToTargetRotatePosition:
//                myLogger.Debug("HAS ENTERED TARGET ROTATE POSITION CASE");
//                myLogger.Debug("Lift Encoder:", stevesRobot.getLiftEncoderCounts());
//                if(Math.abs(stevesRobot.getLiftEncoderCounts() + liftRotatePosition) < 5.0) {// Has the lift extended far enough to begin rotating the turret?
//                    myLogger.Debug("Lift Encoder:", stevesRobot.getLiftEncoderCounts());
//                    myLogger.Debug("HAS REACHED WITHIN 5 ENCODERS OF FINAL POSITION");
//                    myLogger.Debug("Turret Encoders After Lift has Gone Up: ", stevesRobot.getTurretEncoderCounts());
//                    stevesRobot.turretRunWithEncodersSTP();                                         // Yes, so ensure the turret motor is running with set target position
//                    stevesRobot.setTurretMotorVelocity(600.0);                                      // Apply velocity to turret motor, note that the target position was set above
//                                                                                                    // in the turret_start state when the appropriate button was pushed
//                    stevesRobot.setLiftMotorVelocity(0.0);                                          // Stop the lift motor
//                    turretState = tState.turret_turning;                                            // Indicate that the turret is now turning to the requested position
//                }
//                break;
//
//            // the turret is turning to the desired position
//            case turret_turning:
////                myLogger.Debug("turretState: "+turretState);
////                myLogger.Debug("TEC: " + stevesRobot.getTurretEncoderCounts());
//
//                if(Math.abs(stevesRobot.getTurretEncoderCounts() - turretFinalPosition) < 5.0) {    // Has the turret turned to the position requested?
//                    stevesRobot.setTurretMotorVelocity(0.0);                                        // Yes, stop the turret motor
//                    turretState = tState.turret_deposit;                                            // Indicate that the turret is now in the deposit position.  Note: the lift must
//
//                }
//
//                break;
//
//
//            // the turret is in the deposit position
//
//            case turret_deposit:
//                myLogger.Debug("turret deposit");
//                myLogger.Debug(" lift state: ", String.valueOf(liftState));
//
//                if(gamepad2.right_stick_button && liftState == lState.lift_deposit) {               // Has the right stick button on gamepad 2 been pushed AND the lift is in the
//                                                                                                    // deposit state?
//                    myStopWatch.startTime();                                                        // Yes, start the stop watch (we will give the deposit mechanism x number of
//                                                                                                    // seconds to deposit.
//                    stevesRobot.setTurretMotorPower(0.0);                                           // Stop the turret, the turret may have been adjusted by the gunner (see below)
//                    stevesRobot.setinTakeServoPosition(1.0);                                        // Have the intake mechanism deposit the cone
//                    turretState = tState.turret_returning;                                          // Indicate that the turret will be returning to home
//// "Just type 'do good'" -Joshua Lipke
//                }
////                else{
////                    stevesRobot.turretRunWithoutEncoders();
////                    stevesRobot.setTurretMotorPower(0.40 * gamepad2.right_stick_x);                   // Allow the gunner to "adjust" the turret
////
////                }
//
//
//                break;
//
//            // turret is starting to return home
//            case turret_returning:
////                myLogger.Debug("turretState: "+ turretState);
////                myLogger.Debug("Y: " + gamepad2.y);
//                telemetry.addData("elapsed time: ", myStopWatch.getElaspedTime());
//                telemetry.update();
//                if(myStopWatch.getElaspedTime() >= 1.5) {                                           // Has deposit been completed??
//                    if(buttonDpadGround){
//                        stevesRobot.setLiftMotorTargetPosition(-2500);
//                        stevesRobot.setLiftMotorVelocity(-500);
//                        if(Math.abs(stevesRobot.getLiftEncoderCounts() - 2500) < 5.0){
//                            buttonDpadGround = false;
//                        }
//                    }
//                    stevesRobot.setTurretMotorTargetPosition(turret_home);                          // Yes, indicate to have the turret return to home
//                    stevesRobot.setTurretMotorVelocity(500.0);                                      // Apply a velocity
//                    turretState = tState.turret_returningHome;                                      // Indicate the turret is returning home
//                }
//
//                break;
//
//            // turret is returning to home
//            case turret_returningHome:
////                myLogger.Debug("turretState: "+turretState);
////                myLogger.Debug("TEC: " + stevesRobot.getSortedEncoderCount());
//
//                if(Math.abs(stevesRobot.getTurretEncoderCounts() - turret_home) < 10.0) {            // Has the turret returned home?
//                    myLogger.Debug("WE HAVE FINISHED RETURNING HOME ON TURRET");
//                    stevesRobot.setTurretMotorVelocity(0.0);                                        // Yes, stop the turret motor
//                    turretState = tState.turret_start;                                              // Indicate the turret is now back at start
//
//                }
//                break;
//
//            // when turret is in an undefined state - should never happen but you never know
//            default:
////                myLogger.Debug("turretState: "+turretState);
//                turretState = tState.turret_start;                                                  // Indicate that the turret is in a start state
//                break;
//
//        }
//
//        // Chicken switch, aka get me the *&^%$# out of here!
//
//    }

    /**
     * Does lift work...
     */
//    public void doLift(){
//        switch(liftState){
//            case lift_start:
//                myLogger.Debug("LIFT: STARTHERE,STARTHERE,STARTHERE,STARTHERE");
//                myLogger.Debug("turretState: "+turretState);
//                myLogger.Debug("liftState: "+liftState);
//                myLogger.Debug("gamepad2.dpad_up "+gamepad2.dpad_up);
//                myLogger.Debug("gamepad2.dpad_right "+gamepad2.dpad_right);
//                myLogger.Debug("gamepad2.dpad_left "+gamepad2.dpad_left);
//                myLogger.Debug("gamepad2.dpad_down "+gamepad2.dpad_down);
//
//                if (gamepad2.dpad_down) {
//                    stevesRobot.setLiftMotorTargetPosition(-lift_low);
//                    liftFinalPosition = lift_low;
//                    liftState = lState.lift_extending;
//                    stevesRobot.liftRunWithEncodersSTP();
//                    stevesRobot.setLiftMotorVelocity(1500.0);
//                }
//
//                else if (gamepad2.dpad_left) {
//                    stevesRobot.setLiftMotorTargetPosition(-lift_ground);
//                    liftFinalPosition = lift_ground;
//                    buttonDpadGround = true;
//                    liftState = lState.lift_extending;
//                    stevesRobot.liftRunWithEncodersSTP();
//                    stevesRobot.setLiftMotorVelocity(1500.0);
//
//                }
////"just type 'do good'" -josh again
//                else if (gamepad2.dpad_right) {
//
//                    stevesRobot.setLiftMotorTargetPosition(-lift_middle);
//                    liftFinalPosition = lift_middle;
//                    liftState = lState.lift_extending;
//                    stevesRobot.liftRunWithEncodersSTP();
//                    stevesRobot.setLiftMotorVelocity(1500.0);
//
//                    }
//
//                else if (gamepad2.dpad_up) {
//                    stevesRobot.setLiftMotorTargetPosition(-lift_high);
//                    liftFinalPosition = lift_high;
//                    stevesRobot.liftRunWithEncodersSTP();
//                    stevesRobot.setLiftMotorVelocity(1500.0);
//                    liftState = lState.lift_extending;
//                }
//                else if(turretState != tState.turret_liftMovingToTargetRotatePosition){
//                    telemetry.addData("ENCODERS OF LIFT:", stevesRobot.getLiftEncoderCounts());
//                    stevesRobot.liftRunWithoutEncoders();
//                    stevesRobot.setLiftMotorPower(-0.75 * (-gamepad2.right_stick_y));
//
//
//                }
//
//
//
//                break;
//
//
//            case lift_extending:
//                if(Math.abs(stevesRobot.getLiftEncoderCounts() + liftFinalPosition) < 5.0) {
//                    stevesRobot.setLiftPower(0.0);
//                    liftState = lState.lift_deposit;
//                }
//
//                break;
//            case lift_deposit:
//                if(turretState == tState.turret_returningHome){
//                    stevesRobot.setLiftMotorTargetPosition(-lift_ground);
//                    stevesRobot.setLiftMotorVelocity(1500.0);
//                    liftState = lState.lift_retract;
//
//                }
////                else{
////                    stevesRobot.turretRunWithoutEncoders();
////                    stevesRobot.setTurretMotorPower(0.40 * gamepad2.right_stick_x);                   // Allow the gunner to "adjust" the turret
////
////                }
//                break;
//            case lift_retract:
//                if(Math.abs(stevesRobot.getLiftEncoderCounts() + lift_ground) < 5.0) {
//                    myLogger.Debug("WE HAVE FINISHED RETURNING HOME ON LIFT");
//                    stevesRobot.setLiftMotorVelocity(0.0);
//                    stevesRobot.liftRunWithoutEncoders();
//                    liftState = lState.lift_start;
//                }
//                break;
//            default:
//                liftState = lState.lift_start;
//        }
//    }
    public void doDrive(){
        //double autoHeading = RoboRaidersProperties.getHeading();
        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = stevesRobot.getHeading();
        myLogger.Debug("HEADING HEADING HEADING: ", botHeading);
        double y = gamepad1.left_stick_y; // Remember, this is reversed!`
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double lTrigger = gamepad1.left_trigger;
        double rTrigger = gamepad1.right_trigger;

        telemetry.addLine("Variables");
        telemetry.addData("botHeading", String.valueOf(botHeading));
        telemetry.addData("y", String.valueOf(y));
        telemetry.addData("x", String.valueOf(x));
        telemetry.addData("rx", String.valueOf(rx));
        telemetry.addData("rotX", String.valueOf(rotX));
        telemetry.addData("rotY", String.valueOf(rotY));
        telemetry.addData("denominator", String.valueOf(denominator));
        telemetry.addData("frontLeftPower", String.valueOf(frontLeftPower));
        telemetry.addData("backLeftPower", String.valueOf(backLeftPower));
        telemetry.addData("frontRightPower", String.valueOf(frontRightPower));
        telemetry.addData("backRightPower", String.valueOf(backRightPower));
        telemetry.addData("auto heading: ", RoboRaidersProperties.getHeading());

        if(lTrigger > 0.0){
            frontLeftPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            frontRightPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            backLeftPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            backRightPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
        }
        else if(rTrigger > 0.0){
            frontLeftPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            frontRightPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            backLeftPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            backRightPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
        }

//        if(gamepad2.right_bumper) {
//            stevesRobot.setinTakeServoPosition(0.0);
//        }
//        else if(gamepad2.left_bumper) {
//            stevesRobot.setinTakeServoPosition(1.0);
//        }
//        else if(turretState != tState.turret_turning &&  turretState != tState.turret_returningHome) {// No
//            stevesRobot.turretRunWithoutEncoders();
//            stevesRobot.setTurretMotorPower(0.40 * gamepad2.right_stick_x);                   // Allow the gunner to "adjust" the turret
//
//        }
//        if(gamepad2.y && turretState != tState.turret_start) {                                      // The y button on gamepad2 has been pushed AND the turret is not in the start state
//
//            stevesRobot.setTurretMotorVelocity(0.0);                                                // Stop the turret
//            stevesRobot.setLiftMotorVelocity(0.0);
//            liftState = lState.lift_start;
//            turretState = tState.turret_start;                                                      // Indicate that the turret is in a start state
//
//        }
        stevesRobot.setDriveMotorPower(
                frontLeftPower*0.65,
                frontRightPower*0.65,
                backLeftPower*0.65,
                backRightPower*0.65);
        //               dtLogger);
    }
}
