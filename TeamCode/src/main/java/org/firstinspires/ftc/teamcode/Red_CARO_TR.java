package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
*/
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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


//import static org.firstinspires.ftc.teamcode.WebcamTest.VUFORIA_KEY;

@Autonomous(name="Red_CARO_TR", group="Pushbot")

public class Red_CARO_TR extends LinearOpMode
{

    private static int valQUAD = -1;
    private static int valSingle = -1;
    private static int valZero = -1;

    private static float offsetX = .75f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};


    //DRIVE, IMU, AND ACCEL CONSTANTS

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction, rotation;
    PIDController           pidRotate, pidDrive;
    HardwareMecanum_TR robot = new HardwareMecanum_TR();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1120 ;    //ANDYMARK Motor Encoder ticks
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static  double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.575;
    LogisticFunction function;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"

    };
    private VuforiaLocalizer vuforia; //start of vuforia
    private TFObjectDetector tfod;

    double servoStartingPosition = 0.5;

    double distanceBetweenBlocks = 9.5;


    //boolean center;

    boolean left;

    boolean right;


    boolean targetVisible;

    double blockPosition;

    boolean values[] = new boolean[3];

    String positionArray[] = null;


    //for 90 degrees
    //bigger turn = 26.5
    //smaller turn = 21.6

    String VUFORIA_KEY =
            "  AQPmkNv/////AAABmdxLYQ/wu0AklzJL8KSxv7JBzxPLjVotEKmYbHOuh2IRfiFORiDFAmnVudtYfU2lnfHtY52js++UYJP1GQPU2MyXc0SshJVaAVdqYSSs+AXj5hk53ahu6Ce/gwCzdgTQ012TbwUTXJj69VydVB+q75b+UAS/f7U+ddgTOPVulb688iR7I/7aFKTvGC8eeJPM4hPGJOB+zbcI8gd9YErYqhQL4Ot/ek7K0UogrjI2/W4qRxvWlP0GXm0ymybEMlDGovIZWhYykufVvwwAL+TLZld61pJ9m4AMfvzIrvmrIZO5iAmVwNHBtXBiaugvHCgooTncQrn0OcjebK8+Qj7JbDqlcxG/3sVK+RKIcUf9DnWv ";
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
x         */
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

        //if ()

    }
    @Override
//    public void runOpMode() throws InterruptedException{
    public void runOpMode() {

        teleUpdate("status", "Starting runOpMode");
        double rightPos = 0;
        initVuforia();
        initTfod();
        robot.init(hardwareMap);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.75, 16.0 / 9.0);
        }
        robot.init(hardwareMap);
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        function = new LogisticFunction(0.6);
        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

        if (!opModeIsActive());
        {
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
                //level 3
                telemetry.addData("Right", "Duck");

                //level 3
                encoderDrive(8,0.8,"drive");
                fullTurn("clockwise");
                encoderDrive(-23,0.8,"drive");
                encoderDrive(-1,0.8,"strafe");
                robot.CARO.setPower(0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                robot.CARO.setPower(0);
                encoderDrive(2,0.8,"strafe");
                encoderDrive(46,0.8,"drive");

                fullTurn("counterclockwise");
                fullTurn("counterclockwise");
                encoderDrive(4,0.6,"strafe");
                encoderIARM(14,0.6);
                robot.INTAKESERVO.setPower(1);
                encoderDrive(-10,0.5,"strafe");


                //arm code here
                //robot.DROPSERVO.setPosition(0.5);

                encoderDrive(5,0.8,"strafe");
                encoderIARM(-14,0.6);
                //robot.DROPSERVO.setPosition(0);
                robot.INTAKESERVO.setPower(0);
                encoderDrive(49,0.8,"drive");
                encoderDrive(-17,0.8,"strafe");




            } else if (rightPos < 500 && rightPos > 250) {
                //Level 2
                telemetry.addData("Middle", "Duck");

                //level 3
                encoderDrive(8,0.8,"drive");
                fullTurn("clockwise");
                encoderDrive(-23,0.8,"drive");
                encoderDrive(-1,0.8,"strafe");
                robot.CARO.setPower(0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                robot.CARO.setPower(0);
                encoderDrive(2,0.8,"strafe");
                encoderDrive(46,0.8,"drive");

                fullTurn("counterclockwise");
                fullTurn("counterclockwise");

                encoderDrive(4,0.6,"strafe");
                encoderIARM(16.5,0.6);
                robot.INTAKESERVO.setPower(1);
                encoderDrive(-6,0.5,"strafe");


                //arm code here
                //robot.DROPSERVO.setPosition(0.5);

                encoderDrive(4,0.8,"strafe");
                encoderIARM(-16.5,0.6);
                //robot.DROPSERVO.setPosition(0);
                robot.INTAKESERVO.setPower(0);
                encoderDrive(49,0.8,"drive");
                encoderDrive(-17,0.8,"strafe");


            }
            else {
                telemetry.addData("Left", "Duck");
                //level 3
                encoderDrive(8,0.8,"drive");
                fullTurn("clockwise");
                encoderDrive(-23,0.8,"drive");
                encoderDrive(-1,0.8,"strafe");
                robot.CARO.setPower(0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                robot.CARO.setPower(0);
                encoderDrive(2,0.8,"strafe");
                encoderDrive(46,0.8,"drive");

                fullTurn("counterclockwise");
                fullTurn("counterclockwise");
                encoderDrive(4,0.6,"strafe");
                encoderIARM(18,0.6);
                robot.INTAKESERVO.setPower(1);
                encoderDrive(-8,0.5,"strafe");


                //arm code here
                //robot.DROPSERVO.setPosition(0.5);

                encoderDrive(7,0.8,"strafe");
                encoderIARM(-17,0.6);
                //robot.DROPSERVO.setPosition(0);
                robot.INTAKESERVO.setPower(0);
                encoderDrive(49,0.8,"drive");
                encoderDrive(-17,0.8,"strafe");


            }
        }




        waitForStart();

//        encoderDrive(30, "drive");
//        Thread.sleep(1000);//        encoderDrive(-5,1.0,"strafe");
//        encoderDrive(-66,0.9,"drive");
//        halfTurn("counterclockwise");
//        halfTurn("counterclockwise");
//        encoderWobble(-14,0.4);
//        Thread.sleep(100);
//        robot.wobbleServo.setPosition(1.0);
//        encoderDrive(5,1,"strafe");
//        Thread.sleep(100);
//        encoderWobble(14,0.4);

        //   encoderDrive(-10,0.9,"drive");


        //teleUpdate(valQUAD + " <q    " + valSingle + " <s    " + valZero, "");
        //Thread.sleep(3000);

        //webcam.closeCameraDevice();


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        //drive to the blocks and start vuforia
        tfod.shutdown();
    }

    public void semiTurn(String type, int angle) {
        resetAngle();
        if (type.equals("counterclockwise")) {
            long time = System.currentTimeMillis();
            while (getAngle() <= angle & (System.currentTimeMillis() < (time + 6000))) {
                power = (.75 * 2 * 0.684 / 5.063) * (-Math.pow((((getAngle()) + 2.9) / 37.4), 2) + 4.5 * ((getAngle() + 2.9) / 37.4)) + 0.159;
                telemetry.addLine("power: " + power);
                telemetry.addLine("angle: " + getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.FR.setPower(-power);
                robot.BR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void blockServoControlLeft(boolean control){
        if(control){

            //robot.FoundationMoverLeft.setPosition(0.8);    //Pull Position 0.75
        }

        else  {
            //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        }

    }
    public void blockServoControlRight(boolean control){
        if(control){

            //robot.FoundationMoverRight.setPosition(0.2);    //Pull Position 0.75
        }

        else  {
            //robot.FoundationMoverRight.setPosition(0.88);    //Pull Position 0.75
        }

    }

    public void fullTurn(String type){
        resetAngle();
        if(type.equals("counterclockwise")){
            long time = System.currentTimeMillis();
            while ((getAngle() <= 75 & (System.currentTimeMillis()<(time+6000))) && opModeIsActive()) {
                power = (.75*2*0.684/5.063) * (-Math.pow((((getAngle())+2.9)/37.4),2) + 4.5*((getAngle()+2.9)/37.4)) + 0.159;
                telemetry.addLine("power: "+power);
                telemetry.addLine("angle: "+getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.FR.setPower(-power);
                robot.BR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        if(type.equals("clockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() >= -80 && (System.currentTimeMillis()<(time+3000))) {
                power = (.75*2*0.684/5.063) * (-Math.pow((((-getAngle())+2.9)/37.4),2) + 4.5*((-getAngle()+2.9)/37.4)) + 0.159;
                telemetry.addLine(""+power);
                telemetry.addLine(""+getAngle());
                telemetry.update();
                robot.FL.setPower(-power);
                robot.FR.setPower(power);
                robot.BR.setPower(power);
                robot.BL.setPower(-power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void halfTurn(String type){
        telemetry.addLine("performing half turn " +  type);
        resetAngle();
        if(type.equals("counterclockwise")){
            long time = System.currentTimeMillis();
            telemetry.addLine("time: " + time);
            while (getAngle() <= 87 & (System.currentTimeMillis()<(time+10000))) {
                power = (0.75 /*used to be .75*/ *0.684/5.063) * (-Math.pow((((getAngle())+6.5)/19.5),2) + 4.5*((getAngle()+6.5)/19.5)) + 0.159;
                telemetry.addLine("power: " + power);
                telemetry.update();
                telemetry.addLine("angle: " + getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.BR.setPower(-power);
                robot.FR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.BR.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
        }
        if(type.equals("clockwise")){
            long time = System.currentTimeMillis();
            while (getAngle() >= -82 && (System.currentTimeMillis()<(time+2000))) {
                power = (.75*0.684/5.063) * (-Math.pow((((-getAngle())+6.5)/19.5),2) + 4.5*((-getAngle()+6.5)/19.5)) + 0.159;
                teleUpdate(""+power,"");
                robot.FL.setPower(-power);
                robot.BR.setPower(power);
                robot.FR.setPower(power);
                robot.BL.setPower(-power);
            }
            robot.FL.setPower(0);
            robot.BR.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
        }
        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void encoderDrive(double inches, double pow, String driveMode) {
        if(driveMode.equals("drive")){
            //settargetposition is inverse
            //if setpower command for backward is -, then getpowers for both are both positive
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();
            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            double currentPosInches;
            if(inches<=0) {
                robot.FL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                //robot.changeSpeed(power);
                while (robot.FR.getTargetPosition()>robot.FR.getCurrentPosition()||
                        robot.FL.getTargetPosition()>robot.FL.getCurrentPosition()||
                        robot.BR.getTargetPosition()>robot.BR.getCurrentPosition()||
                        robot.BL.getTargetPosition()>robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    teleUpdate("CURRENTPOSINCHES: "+currentPosInches+"","");
                    power = function.getPowerAt(currentPosInches, -inches, pow, "drive");
                    robot.FL.setPower(power + correction);
                    robot.BR.setPower(power - correction);
                    robot.FR.setPower(power - correction);
                    robot.BL.setPower(power + correction);
                }
            }
            else{
                //robot.changeSpeed(-power);
                robot.FL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
                while (robot.FR.getTargetPosition()<robot.FR.getCurrentPosition()||
                        robot.FL.getTargetPosition()<robot.FL.getCurrentPosition()||
                        robot.BR.getTargetPosition()<robot.BR.getCurrentPosition()||
                        robot.BL.getTargetPosition()<robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, inches, pow, "drive");
                    teleUpdate("POWER: "+ power+"","");
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower((power - correction));
                    robot.BL.setPower((power + correction));
                }
            }
            robot.changeSpeed(0);
        }
        else if(driveMode.equals("strafe")){/////LEFT IS POSITIVE
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            resetAngle();

            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int)(inches*COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
//            telemetry.addLine(robot.frontLeft.getTargetPosition()+" <- TARGET");
//            telemetry.addLine(robot.frontLeft.getCurrentPosition()+" <- Current");
//            telemetry.update();
//            Thread.sleep(2000);
            double currentPosInches;
            //power = 0.9;
            robot.changeSpeed(power);
            if(inches>0) {
                while (robot.FR.getTargetPosition()<robot.FR.getCurrentPosition()||
                        robot.FL.getTargetPosition()>robot.FL.getCurrentPosition()||
                        robot.BR.getTargetPosition()>robot.BR.getCurrentPosition()||
                        robot.BL.getTargetPosition()<robot.BL.getCurrentPosition()) {
                    telemetry.addData("Correction", correction);
                    telemetry.addLine(robot.FL.getCurrentPosition()+" <- Current");
                    telemetry.update();
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FR.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    power = function.getPowerAt(currentPosInches, inches, pow, "strafe")*1.1;
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower(-(power + correction));
                    robot.BL.setPower(-(power - correction));             //STRAFE
                }
            }
            else{
                while (robot.FR.getTargetPosition()>robot.FR.getCurrentPosition()||
                        robot.FL.getTargetPosition()<robot.FL.getCurrentPosition()||
                        robot.BR.getTargetPosition()<robot.BR.getCurrentPosition()||
                        robot.BL.getTargetPosition()>robot.BL.getCurrentPosition()) {
                    telemetry.addData("Correction", correction);
                    telemetry.update();
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FR.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, -inches, pow, "strafe")*1.1;
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
//        Thread.sleep(100);
    }




    public void encoderDriveStrafe(double inches, double pow) throws InterruptedException {
        /////LEFT IS POSITIVE
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        resetAngle();

        int startPos1 = robot.FL.getCurrentPosition();
        int startPos2 = robot.BL.getCurrentPosition();
        int startPos3 = robot.FR.getCurrentPosition();
        int startPos4 = robot.BR.getCurrentPosition();
        robot.FL.setTargetPosition((int)(inches*COUNTS_PER_INCH));
        robot.BR.setTargetPosition((int)(inches*COUNTS_PER_INCH));
        robot.FR.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
        robot.BL.setTargetPosition((int)(-inches*COUNTS_PER_INCH));
//            telemetry.addLine(robot.frontLeft.getTargetPosition()+" <- TARGET");
//            telemetry.addLine(robot.frontLeft.getCurrentPosition()+" <- Current");
//            telemetry.update();
//            Thread.sleep(2000);
        double currentPosInches;
        //power = 0.9;
        robot.changeSpeed(power);
        if(inches>0) {
            while (robot.FR.getTargetPosition()<robot.FR.getCurrentPosition()||
                    robot.FL.getTargetPosition()>robot.FL.getCurrentPosition()||
                    robot.BR.getTargetPosition()>robot.BR.getCurrentPosition()||
                    robot.BL.getTargetPosition()<robot.BL.getCurrentPosition()) {
                telemetry.addData("Correction", correction);
                telemetry.addLine(robot.FL.getCurrentPosition()+" <- Current");
                telemetry.update();
                correction = pidDrive.performPID(getAngle());
                currentPosInches = ((robot.FR.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                power = pow;
                robot.FL.setPower((power + correction));
                robot.BR.setPower((power - correction));
                robot.FR.setPower(-(power + correction));
                robot.BL.setPower(-(power - correction));             //STRAFE
            }
        }
        else{
            while (robot.FR.getTargetPosition()>robot.FR.getCurrentPosition()||
                    robot.FL.getTargetPosition()<robot.FL.getCurrentPosition()||
                    robot.BR.getTargetPosition()<robot.BR.getCurrentPosition()||
                    robot.BL.getTargetPosition()>robot.BL.getCurrentPosition()) {
                telemetry.addData("Correction", correction);
                telemetry.update();
                correction = pidDrive.performPID(getAngle());
                currentPosInches = ((robot.FR.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                power = -pow;
                robot.FL.setPower((power + correction));
                robot.BR.setPower((power - correction));
                robot.FR.setPower(-(power + correction));
                robot.BL.setPower(-(power - correction));             //STRAFE
            }
        }
        robot.changeSpeed(0);

        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(100);
    }

    public void encoderIARM(double inches, double pow) {
        robot.IARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.IARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.IARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //settargetposition is inverse
        //if setpower command for backward is -, then getpowers for both are both positive

        int startPos1 = robot.IARM.getCurrentPosition();
        robot.IARM.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        teleUpdate(robot.IARM.getCurrentPosition() + ", " + robot.IARM.getTargetPosition(), "");

//            Thread.sleep(3000);
        if (inches > 0) {
            while ((robot.IARM.getTargetPosition() > robot.IARM.getCurrentPosition()) && opModeIsActive()) {
                robot.IARM.setPower(pow);
                telemetry.addLine(Integer.toString(robot.IARM.getTargetPosition()) + "<-target       current->" + Integer.toString(robot.IARM.getCurrentPosition()));
                telemetry.update();
            }
            robot.IARM.setPower(0);
        } else {
            while ((robot.IARM.getTargetPosition() < robot.IARM.getCurrentPosition()) && opModeIsActive()) {
                robot.IARM.setPower(-pow);
                telemetry.addLine(Integer.toString(robot.IARM.getTargetPosition()) + "<-target       current->" + Integer.toString(robot.IARM.getCurrentPosition()));
                telemetry.update();
            }
            robot.IARM.setPower(0);
        }

        robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Thread.sleep(100);
    }





    public void teleUpdate(String label, String description){
        if (robot != null && robot.FL != null && robot.FR != null) {
            telemetry.addLine().addData("Current Position",  "Running at %7d :%7d :%7d :%7d", robot.FL.getCurrentPosition(), robot.FR.getCurrentPosition(), robot.BL.getCurrentPosition(), robot.BR.getCurrentPosition());
            telemetry.addLine().addData("Target Position",  "Running at %7d :%7d :%7d :%7d", robot.FL.getTargetPosition(), robot.FR.getTargetPosition(), robot.BL.getTargetPosition(), robot.BR.getTargetPosition());
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
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();


}


