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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue_CARO_TR", group="Pushbot")
//@Disabled
public class Blue_CARO_TR extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum_TR robot = new HardwareMecanum_TR();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double CARO_DIST = 80;//inches
    static final double     FORWARD_SPEED = 0.18;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"

    };
    private VuforiaLocalizer vuforia; //start of vuforia
    private TFObjectDetector tfod;





    String VUFORIA_KEY =
            "  AQPmkNv/////AAABmdxLYQ/wu0AklzJL8KSxv7JBzxPLjVotEKmYbHOuh2IRfiFORiDFAmnVudtYfU2lnfHtY52js++UYJP1GQPU2MyXc0SshJVaAVdqYSSs+AXj5hk53ahu6Ce/gwCzdgTQ012TbwUTXJj69VydVB+q75b+UAS/f7U+ddgTOPVulb688iR7I/7aFKTvGC8eeJPM4hPGJOB+zbcI8gd9YErYqhQL4Ot/ek7K0UogrjI2/W4qRxvWlP0GXm0ymybEMlDGovIZWhYykufVvwwAL+TLZld61pJ9m4AMfvzIrvmrIZO5iAmVwNHBtXBiaugvHCgooTncQrn0OcjebK8+Qj7JbDqlcxG/3sVK+RKIcUf9DnWv ";
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        double rightPos = 0;
        initVuforia();
        initTfod();
        robot.init(hardwareMap);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.75, 16.0/9.0);
        }
        if (!opModeIsActive()); {
        while (!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getRecognitions();

                if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        rightPos = recognition.getRight();
                    }
                    telemetry.update();

                }
            }
        }

            telemetry.update();
            if (rightPos > 500) {
                telemetry.addData("Right", "Duck");

            } else if (rightPos < 500 && rightPos > 250) {
                telemetry.addData("Middle", "Duck");

            } else {
                telemetry.addData("Left", "Duck");
            }
            telemetry.update();
        }











        // Send telemetry message to signify robot waiting;
  //      telemetry.addData("Status", "Resetting Encoders");    //
    //    telemetry.update();
        //stop and reset
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
          //      robot.FL.getCurrentPosition(), robot.FR.getCurrentPosition(),
            //    robot.BL.getCurrentPosition(), robot.BR.getCurrentPosition());

//        telemetry.update();


        //  Wait for the game to start (driver presses PLAY)
        waitForStart();


            if (opModeIsActive()) {

                if (rightPos > 500) {
                    telemetry.addData("Right", "Duck");
                    encoderDrive(0.8,6,6,5);
                    encoderStrafe(0.8,17,17,5);
                    encoderDrive(0.8,-6,-6,5);

                    robot.CARO.setPower(FORWARD_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 8.0)) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    encoderStrafe(0.8,-35,-35,5);
                    encoderDrive(0.8,15,15,5);
                    encoderDrive(0.8,-10,-10,5);
                    encoderDrive(TURN_SPEED,-17,17,5);
                    encoderDrive(0.8,48,48,10);
                }
                else if (rightPos < 500 && rightPos > 250) {
                    telemetry.addData("Middle", "Duck");
                    telemetry.addData("Right", "Duck");
                    encoderDrive(0.8,6,6,5);
                    encoderStrafe(0.8,17,17,5);
                    encoderDrive(0.8,-6,-6,5);

                    robot.CARO.setPower(FORWARD_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 8.0)) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    encoderStrafe(0.8,-35,-35,5);
                    encoderDrive(0.8,15,15,5);
                    encoderDrive(0.8,-10,-10,5);
                    encoderDrive(TURN_SPEED,-17,17,5);
                    encoderDrive(0.8,48,48,10);
                }
                else {
                    telemetry.addData("Left", "Duck");
                    telemetry.addData("Right", "Duck");
                    encoderDrive(0.8,6,6,5);
                    encoderStrafe(0.8,17,17,5);
                    encoderDrive(0.8,-6,-6,5);

                    robot.CARO.setPower(FORWARD_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 8.0)) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    encoderStrafe(0.8,-35,-35,5);
                    encoderDrive(0.8,15,15,5);
                    encoderDrive(0.8,-10,-10,5);
                    encoderDrive(TURN_SPEED,-17,17,5);
                    encoderDrive(0.8,48,48,10);
                }
                telemetry.update();
            }



        // Step through each leg of the path,

        // Note: Reverse movement is obtained by setting a negative distance (not speed)





    //    encoderStrafe(DRIVE_SPEED, 17, 17, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout


        //used for caro movment
        /*robot.CARO.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }



        encoderStrafe(DRIVE_SPEED,-13,-13,5.0);
        encoderDrive(TURN_SPEED, 6, -6, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 68, 68, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        //encoderDrive(0.4,5,5,3.0);
*/



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*24
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newRightTarge;
        int newLeftTarge;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // robot.BL.setDirection(DcMotor.Direction.REVERSE);
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.FR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarge = robot.BR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarge = robot.BL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            robot.FL.setTargetPosition(newLeftTarget);
            robot.FR.setTargetPosition(newRightTarget);
            robot.BR.setTargetPosition(newRightTarge);
            robot.BL.setTargetPosition(newLeftTarge);

            // Turn On RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.FL.setPower(Math.abs(speed));
            robot.FR.setPower(Math.abs(speed));
            robot.BL.setPower(Math.abs(speed));
            robot.BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) ||
                    (robot.FL.isBusy() && robot.FR.isBusy() &&
                            robot.BL.isBusy() && robot.BR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarge, newRightTarge);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.FL.getCurrentPosition(),
                        robot.FR.getCurrentPosition());
                telemetry.addData("Path3", "Running at %7d :%7d",
                        robot.BL.getCurrentPosition(),
                        robot.BR.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            //stop and reset
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Turn off RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void encoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget1;
        int newRightTarget1;
        int newRightTarge1;
        int newLeftTarge1;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // robot.BL.setDirection(DcMotor.Direction.REVERSE);
            // Determine new target position, and pass to motor controller
            newLeftTarget1 = robot.FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget1 = robot.FR.getCurrentPosition() + (int) (-rightInches * COUNTS_PER_INCH);
            newRightTarge1 = robot.BR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarge1 = robot.BL.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);
            robot.FL.setTargetPosition(newLeftTarget1);
            robot.FR.setTargetPosition(newRightTarget1);
            robot.BR.setTargetPosition(newRightTarge1);
            robot.BL.setTargetPosition(newLeftTarge1);

            // Turn On RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.FL.setPower(Math.abs(speed));
            robot.FR.setPower(Math.abs(speed));
            robot.BL.setPower(Math.abs(speed));
            robot.BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) ||
                    (robot.FL.isBusy() && robot.FR.isBusy() &&
                            robot.BL.isBusy() && robot.BR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarge1, newRightTarge1);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.FL.getCurrentPosition(),
                        robot.FR.getCurrentPosition());
                telemetry.addData("Path3", "Running at %7d :%7d",
                        robot.BL.getCurrentPosition(),
                        robot.BR.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            //stop and reset
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Turn off RUN_TO_POSITION
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }

    public void encoderCARO(double inches, double speed, double timeoutS){
        int newCAROTARGET;
        if (opModeIsActive()) {
            newCAROTARGET = robot.CARO.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.CARO.setTargetPosition(newCAROTARGET);

             robot.CARO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.CARO.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) || (robot.CARO.isBusy())) {
                //telemetry.addData("Counts_Per_Inch", "Running to %7d", COUNTS_PER_INCH);
                telemetry.addData("newCAROTARGET - inches", "Running to %7d", newCAROTARGET);
                telemetry.addData("Path2", "Running at %7d", robot.CARO.getCurrentPosition());
                telemetry.update();
            }
            robot.CARO.setPower(0);
            robot.CARO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.CARO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



            }

        }

    /*public void encoderCARO(double inches, double pow) {
        robot.CARO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.CARO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //settargetposition is inverse
        //if setpower command for backward is -, then getpowers for both are both positive

        int startPos1 = robot.CARO.getCurrentPosition();
        robot.CARO.setTargetPosition((int)(inches * COUNTS_PER_INCH));
        telemetry.addLine(Integer.toString(robot.CARO.getTargetPosition()) + "<-target       current->" + Integer.toString(robot.CARO.getCurrentPosition()));
        telemetry.update();
//            Thread.sleep(3000);

        while (robot.CARO.getTargetPosition() > startPos1) {
            robot.CARO.setPower(pow);
            telemetry.addLine(Integer.toString(robot.CARO.getTargetPosition()) + "<-target       current->" + Integer.toString(robot.CARO.getCurrentPosition()));
            telemetry.update();
        }



        robot.CARO.setPower(0);

        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Thread.sleep(100);
    }
        }

*/
