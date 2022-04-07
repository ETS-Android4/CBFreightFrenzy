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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
@Disabled
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
    private RevColorSensorV3 freightSensor = null;

    boolean check = false;
    double i;
    boolean check2 = false;
    static final double MAX_POS = 1;     // Maximum rotational position
    static final double CEN_POS = 0.5;     // Minimum rotational position
    static final double MIN_POS = 0;
    static final double INTAKE_POS = 0.85;  // Minimum rotational position
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    private boolean goldFound;      // Sound file present flags
    private boolean silverFound;

    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;

    boolean lastA = false;                      // Use to track the prior button state.
    boolean lastLB = false;                     // Use to track the prior button state.
    boolean highLevel = false;                  // used to prevent multiple level-based rumbles.
    boolean secondHalf = false;                 // Use to prevent multiple half-time warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime1 = new ElapsedTime();    // Use to determine when end game is starting.

    final double HALF_TIME = 20.0;              // Wait this many seconds before rumble-alert for half-time.
    final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.




    public void runOpMode() throws InterruptedException {
        int silverSoundID = hardwareMap.appContext.getResources().getIdentifier("ss_roger_roger", "raw", hardwareMap.appContext.getPackageName());
        int goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("ss_siren",   "raw", hardwareMap.appContext.getPackageName());


        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        // Display sound status
        telemetry.addData("gold resource",   goldFound ?   "Found" : "NOT found\n Add gold.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", silverFound ? "Found" : "Not found\n Add silver.wav to /src/main/res/raw" );
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

        freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightcolor");

        double leftPower;
        double rightPower;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        runtime1.reset();



        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            servoArm.setPosition(position);
            double caroPower1;
            double caroPower2;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double armC = -gamepad2.right_stick_y;
            double intakeC = -gamepad2.left_stick_y;
            boolean intakeboxservoALDROP = gamepad2.dpad_up;
            boolean intakeboxservoSHDROP = gamepad2.dpad_left;
            boolean intakeboxservoHOLD = gamepad2.dpad_right;
            boolean intakeboxservoDEFAULT = gamepad2.dpad_down;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double caro1 = gamepad1.right_trigger;
            double caro2 = gamepad1.left_trigger;

            caroPower1 = Range.clip(caro1 + 0,-1.0,1);
            caroPower2 = Range.clip(caro2 + 0,-1.0,1);

            telemetry.addData("CaroPower1 - ", caro1);
            telemetry.addData("CaroPower2 - ", caro2);

            if (caroPower1 > 0) {
                caro.setPower(caroPower1/3);
            }
            else if (caroPower2 > 0) {
                caro.setPower(-caroPower2/3);

            }
            else {
                caro.setPower(0);

            }
            //caro.setPower(-caroPower2/3);
            //caro.setPower(-caroPower1/3);

            motorFrontLeft.setPower(frontLeftPower/1.5);
            motorBackLeft.setPower(backLeftPower/1.5);
            motorFrontRight.setPower(frontRightPower/1.5);
            motorBackRight.setPower(backRightPower/1.5);

            leftPower = Range.clip(armC, -1.0, 1.0);
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            motorArm.setPower(-(armC / 3));
            motorIntake.setPower(intakeC);

            boolean currentA = gamepad1.a ;
            boolean currentLB = gamepad1.left_bumper ;

            // Display the current Rumble status.  Just for interest.
            telemetry.addData(">", "Are we RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no" );

            // ----------------------------------------------------------------------------------------
            // Example 1. b) Watch the runtime timer, and run the custom rumble when we hit half-time.
            //               Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            // ----------------------------------------------------------------------------------------
            if ((runtime1.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                secondHalf =true;
            }

            // Display the time remaining while we are still counting down.
            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime1.seconds()) );
            }




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

            if (intakeboxservoALDROP) {
                servoArm.setPosition(MIN_POS);
            }
            if (intakeboxservoSHDROP) {
                servoArm.setPosition(MAX_POS);
            }
            if (intakeboxservoHOLD) {
                servoArm.setPosition(CEN_POS);
            }
            if (intakeboxservoDEFAULT) {
                servoArm.setPosition(INTAKE_POS);
            }

            if (gamepad2.a)  {
                double servoincreaser = 0;

                servoArm.setPosition(servoincreaser + 0.1);
            }

            telemetry.addData("Red - ", freightSensor.red());
            telemetry.addData("Green - ", freightSensor.green());
            telemetry.addData("Blue - ", freightSensor.blue());
            telemetry.addData("Distance - ", freightSensor.getDistance(DistanceUnit.CM));


            telemetry.update();

            double distsensevalue = freightSensor.getDistance(DistanceUnit.CM);

            if (distsensevalue<1.2) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                telemetry.addData("Playing", "Resource Silver");
                telemetry.update();
                if (distsensevalue>1.2) {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                    telemetry.addData("Playing", "Resource Gold");
                    telemetry.update();
                }
            }
            else {
                SoundPlayer.getInstance().stopPlayingAll();
            }









        }
    }
}