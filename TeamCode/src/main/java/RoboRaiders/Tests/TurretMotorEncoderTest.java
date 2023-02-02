/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.Robot.TestRobot;

@Autonomous
//@disable
public class TurretMotorEncoderTest extends LinearOpMode
{
    double power = 0.20;
    double heading;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override

    public void runOpMode() throws InterruptedException {

        TestRobot dogMan = new TestRobot();
        dogMan.initialize(hardwareMap);

        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();

        // Wait for start to be pushed
        waitForStart();
        dogMan.runWithEncoders();
        //setDriveMotorPower(-power, power, -power, power);
        dogMan.setTurretMotorPower(power);

        while (opModeIsActive()) {

            telemetry.addLine().addData("Turret Encoder Count: ", dogMan.getTurretEncoderCounts());

            telemetry.addLine().addData("Power applied: ", power);
            telemetry.update();
        }
        dogMan.setTurretMotorPower(0.0);
        telemetry.addData("setting power to zero", true);



    }



}
