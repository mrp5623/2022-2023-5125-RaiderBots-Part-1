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

package org.firstinspires.ftc.RaiderBotsteamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="RaiderBots Auto", group="Auto")

public class RaiderBotsAuto extends LinearOpMode {

    RaiderBotsHwMap robot   = new RaiderBotsHwMap();

    static final double COUNTS_PER_MOTOR_REV = 1440 ;    // eg: Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(DRIVE_SPEED, 10,10);
        encoderDrive(TURN_SPEED,-10,-10);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = robot.FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.FrontLeft.setTargetPosition(newLeftTarget);
            robot.BackLeft.setTargetPosition(newLeftTarget);
            robot.FrontRight.setTargetPosition(newRightTarget);
            robot.BackRight.setTargetPosition(newRightTarget);

            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.FrontLeft.setPower(Math.abs(speed));
            robot.BackLeft.setPower(Math.abs(speed));
            robot.FrontRight.setPower(Math.abs(speed));
            robot.BackRight.setPower(Math.abs(speed));

            robot.FrontLeft.setPower(0);
            robot.BackLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.BackRight.setPower(0);


            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
