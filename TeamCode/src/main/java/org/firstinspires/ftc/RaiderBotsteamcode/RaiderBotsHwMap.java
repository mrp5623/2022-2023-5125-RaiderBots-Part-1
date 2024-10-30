package org.firstinspires.ftc.RaiderBotsteamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RaiderBotsHwMap {

    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;
    public Servo Servo = null;
    //public CRServo ServoCont = null;

    double armPosition, gripPosition, contPower, contStart, contPosition;
    double MIN_POSITION = 0, MAX_POSITION = 1;

    HardwareMap RaiderBotsHwMap;

    public void init(HardwareMap ahwMap) {

        RaiderBotsHwMap = ahwMap;

        FrontLeft = RaiderBotsHwMap.get(DcMotor.class, "FrontLeft");
        BackLeft = RaiderBotsHwMap.get(DcMotor.class, "BackLeft");
        FrontRight = RaiderBotsHwMap.get(DcMotor.class, "FrontRight");
        BackRight = RaiderBotsHwMap.get(DcMotor.class, "BackRight");
        //ServoCont = RaiderBotsHwMap.get(CRServo.class,"ServoCont");
        Servo = RaiderBotsHwMap.get(Servo.class, "servo1");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        //ServoCont.setDirection(CRServo.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        armPosition = .5;
        gripPosition = MAX_POSITION;
        contStart = 0;
        contPosition = contStart += contPower;

    }
 }