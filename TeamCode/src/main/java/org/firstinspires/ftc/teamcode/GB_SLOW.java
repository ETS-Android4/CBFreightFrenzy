///////testing purpose
package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *Tel_GB
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="GB_SLOW", group="Linear Opmode")
//@Disabled
public class GB_SLOW extends LinearOpMode {
    public static int convertBoolean(boolean x) {
        if (x) {
            return 1;
        } else return 0;
    }

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor arm = null;
    private DcMotor intake = null;
    private Servo servoArm = null;
    private DcMotor caro = null;

    boolean check = false;
    double i;
    boolean check2 = false;
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double CEN_POS = 0.3;     // Minimum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("br");
        DcMotor motorArm = hardwareMap.dcMotor.get("iarm");
        DcMotor motorIntake = hardwareMap.dcMotor.get("is");
        DcMotor caro = hardwareMap.dcMotor.get("caro");

        servoArm = hardwareMap.get(Servo.class, "sa");

        double leftPower;
        double rightPower;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        servoArm.setPosition(MAX_POS);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //servoArm.setPosition(position);
            double caroPower;
            double caroPower1;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double armC = -gamepad2.left_stick_y;
            double intakeC = -gamepad2.right_stick_y;
            boolean intakeboxservoUP = gamepad2.dpad_up;
            boolean intakeboxservoDN = gamepad2.dpad_down;
            boolean intakeboxservoCEN = gamepad2.dpad_right;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double caro1 = gamepad1.left_trigger;
            double caro2 = gamepad1.right_trigger;


            caroPower = Range.clip(caro1 + 0,-1.0,1);
            caroPower1 = Range.clip(caro2 + 0,-1.0,1);
            caro.setPower(caroPower/1.20);
            caro.setPower(-caroPower1/1.2);

            motorFrontLeft.setPower(frontLeftPower/1.5);
            motorBackLeft.setPower(backLeftPower/1.5);
            motorFrontRight.setPower(frontRightPower/1.5);
            motorBackRight.setPower(backRightPower/1.5);

            leftPower = Range.clip(armC, -1.0, 1.0);
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            motorArm.setPower(-(armC / 3));
            motorIntake.setPower(intakeC);

            if (gamepad1.left_bumper)
            {
                motorFrontLeft.setPower(1);
                motorBackLeft.setPower(1);
                motorFrontRight.setPower(1);
                motorBackRight.setPower(1);
            }
            if (gamepad1.right_bumper)
            {
                motorFrontLeft.setPower(-1);
                motorBackLeft.setPower(-1);
                motorFrontRight.setPower(-1);
                motorBackRight.setPower(-1);
            }

            if (intakeboxservoUP) {
                servoArm.setPosition(MIN_POS);
            }
            if (intakeboxservoDN) {
                servoArm.setPosition(MAX_POS);
            }
            if (intakeboxservoCEN) {
                servoArm.setPosition(CEN_POS);
            }

        }
    }
}