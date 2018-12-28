/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Neha Deshpande, FTC 12116 on 7/1/2018.
 */

@TeleOp(name="Servo Position Finder", group="Neha")
// @Disabled
public class RedbotServo extends LinearOpMode {

    private static double precision = 0.1;
    private static long delay = 200;

    /* Declare OpMode members. */
    HardwareIhba robot = new HardwareIhba();

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {


                if (gamepad1.right_bumper) {    // Pressing this makes it less precise
                    precision += 0.05;
                    sleep(1000);
                }
                if (gamepad1.left_bumper){      // Pressing this makes it more precise
                    precision -= 0.05;
                    sleep(1000);
                }
                print();


                // Pressing Y and dpad_up or dpad_down changes teamMarker's position
                while (opModeIsActive() && gamepad1.y) {
                    print();
                    if (gamepad1.dpad_up) {
                        robot.dump.setPower(precision);
                        sleep(delay);
                    }
                    if (gamepad1.dpad_down) {
                        robot.dump.setPower(precision);
                        sleep(delay);
                    }
                }

                // Pressing A and dpad_up or dpad_down changes sampling's position
//                while (opModeIsActive() && gamepad1.a) {
//                    print();
//                    if (gamepad1.dpad_up) {
//                        robot.teamMarker.setPosition(robot.teamMarker.getPosition() + precision);
//                        sleep(delay);
//                    }
//                    if (gamepad1.dpad_down) {
//                        robot.teamMarker.setPosition(robot.teamMarker.getPosition() - precision);
//                        sleep(delay);
//                    }
//                }


                // getPosition() returns the last value where it was COMMANDED to go to, so can't move servo by hand.
                print();


            }
    }

    private void print()
    {
        telemetry.addData("precision", precision);
        telemetry.addData("delay", delay);
        //telemetry.addData("y - marker", robot.marker.getPower());
      //  telemetry.addData("y - teamMarker", robot.teamMarker.getPosition());
        // Add more telemetry here if more servos are needed.

        telemetry.update();
    }
}
