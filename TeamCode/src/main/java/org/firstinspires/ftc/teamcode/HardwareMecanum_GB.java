package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class HardwareMecanum_GB {
    /* Public OpMode members. */
    public DcMotor FR   = null;
    public DcMotor  FL  = null;
    public DcMotor  BR  = null;
    public DcMotor  BL  = null;
    public DcMotor  CARO = null;
    public DcMotor  IARM = null;
    public DcMotor INTAKESERVO = null;
    public Servo servoArm = null;
    public DistanceSensor sensorRange = null;
    /* local OpMode members. */
    HardwareMap hwMapG           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum_GB(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMapG = ahwMap;

        // Define and Initialize Motors
        sensorRange = hwMapG.get(DistanceSensor.class, "sensorrange");
        FR = hwMapG.get(DcMotor.class, "fr");
        FL = hwMapG.get(DcMotor.class, "fl");
        BR = hwMapG.get(DcMotor.class, "br");
        BL = hwMapG.get(DcMotor.class, "bl");
        CARO = hwMapG.get(DcMotor.class, "caro");
        IARM = hwMapG.get(DcMotor.class,"iarm");
        servoArm = hwMapG.get(Servo.class, "sa");
        INTAKESERVO = hwMapG.get(DcMotor.class,"is");
        FR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //This motor has to be set the opposite direction of all the other motors for it to work in sync with them
        FL.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        BR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        BL.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void changeMode(DcMotor.RunMode x) {
        FL.setMode(x);
        BL.setMode(x);
        FR.setMode(x);
        BR.setMode(x);
    }

    public void changeSpeed(double x) {
        FL.setPower(x);
        FR.setPower(x);
        BR.setPower(x);
        BL.setPower(x);
    }
}
