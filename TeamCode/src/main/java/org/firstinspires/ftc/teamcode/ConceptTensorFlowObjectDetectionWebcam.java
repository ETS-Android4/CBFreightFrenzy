/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareMecanum;
import org.firstinspires.ftc.teamcode.LogisticFunction;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */

    //Encoders
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .50, correction, rotation;
    PIDController pidRotate, pidDrive;
    static final double COUNTS_PER_MOTOR_REV = 1120;    //ANDYMARK Motor Encoder ticks
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static double DRIVE_SPEED = 0.9;
    LogisticFunction function;
    HardwareMecanum robot = new HardwareMecanum();
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " ARGKFNf/////AAABmeWmTIKr70aQrGH7lC6M8xBPdcMfnaNjD/dopWNwdsWuQbrZLFQZZBr/eFBlpHuykY0IY4f9Y34OVFaL4NRxmFd4ghxNkwK3Cjl/4Jo6bf/v+ovD7Tqdf8cT0A3McQF2rxOPE8fsmaC2TfCr8nZquqbbaTZT7bxtuvi8skuLfHg0BNRGaKtEYyPaJ+wdvAcJZ8+2rZ6q+77Ooh2teMYGmJRe+KDD8LmIMn5Jh/r/Lbm9WqjmxuSV6NxwAwpqTPydgJAE/19fXRVbC4+vGWAiiAxd/UIrLxDtgwekkiudCLSa1r1Y8XjtaTeUUWYXl7+iAxkAOX3ZYa84fFrPGnFvYdhjnIuRGo4AgL6dvb/pQEaK ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException{
        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        function = new LogisticFunction(0.6);
        teleUpdate("status", "Starting runOpMode");
        //robot.init(hardwareMap);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        teleUpdate("WWWWWWWWWWWWWWWWWWWWWWWWWWWWW","");
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        teleUpdate("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEee","");
        imu.initialize(parameters);
        teleUpdate("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQqqq","");
        pidRotate = new PIDController(.003, .00003, 0);
        teleUpdate("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA","");
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();






        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
       encoderDrive(3,0.5,"drive");
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
         /*           List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
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
                        } */
                        telemetry.update();
                    }
                }
            }
        }


    /**
     * Initialize the Vuforia localization engine.
     */
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
    private void initTfod() throws InterruptedException {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //basic test


    }
    public void encoderDrive(double inches, double pow, String driveMode) throws InterruptedException {
        if (driveMode.equals("drive")) {
            //settargetposition is inverse
            //if setpower command for backward is -, then getpowers for both are both positive
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();
            teleUpdate("" + robot.FL.getCurrentPosition() + "   <>   " + robot.FL.getTargetPosition(), "");
            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.FL.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            double currentPosInches;
            if (inches <= 0) {
                robot.FL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                //robot.changeSpeed(power);
                while (robot.FR.getTargetPosition() > robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() > robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() > robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() > robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    teleUpdate("CURRENTPOSINCHES: " + currentPosInches + "", "");
                    power = function.getPowerAt(currentPosInches, -inches, pow, "drive");
                    robot.FL.setPower(power + correction);
                    robot.BR.setPower(power - correction);
                    robot.FR.setPower(power - correction);
                    robot.BL.setPower(power + correction);
                    teleUpdate("currentPos: " + robot.FL.getCurrentPosition() + "    power: " + power + "    correction: " + correction, "");
                }
            } else {
                //robot.changeSpeed(-power);
                robot.FL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                while (robot.FR.getTargetPosition() < robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() < robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() < robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() < robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, inches, pow, "drive");
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower((power - correction));
                    robot.BL.setPower((power + correction));
                    teleUpdate("currentPos: " + robot.FL.getCurrentPosition() + "    power: " + power + "    correction: " + correction, "");
                }
            }
            robot.changeSpeed(0);
        } else if (driveMode.equals("strafe")) {/////LEFT IS POSITIVE
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();

            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                       telemetry.addLine(robot.FL.getTargetPosition()+" <- TARGET");
                        telemetry.addLine(robot.FL.getCurrentPosition()+" <- Current");
                        telemetry.update();
                        Thread.sleep(2000);

            double currentPosInches;
            //power = 0.9;
            robot.changeSpeed(power);
            if (inches > 0) {
                while (robot.FR.getTargetPosition() < robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() > robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() > robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() < robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    power = function.getPowerAt(currentPosInches, inches, pow, "strafe") * 1.1;
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower(-(power + correction));
                    robot.BL.setPower(-(power - correction));             //STRAFE
                    teleUpdate("power: " + power, "");
                }
            } else {
                while (robot.FR.getTargetPosition() > robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() < robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() < robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() > robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, -inches, pow, "strafe") * 1.1;
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower(-(power + correction));
                    robot.BL.setPower(-(power - correction));             //STRAFE
                }
            }
            robot.changeSpeed(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(100);
    }
        private void resetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }
    public void teleUpdate(String label, String description) {
        if (robot != null && robot.FL != null && robot.FR != null) {

        }
        telemetry.addLine().addData(label + ": ", description);
        telemetry.update();
    }
        private double getAngle()
        {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;

            return globalAngle;
        }
    }

