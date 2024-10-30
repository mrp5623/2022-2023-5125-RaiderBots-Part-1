package org.firstinspires.ftc.RaiderBotsteamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "RaiderBotsTeleOp", group = "TeleOp")

public class RaiderBotsTeleOp extends OpMode{

    RaiderBotsHwMap robot = new RaiderBotsHwMap();

    @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.speak("Initialized");
        telemetry.addLine("Initialized");
        telemetry.update();
    }
    
    @Override
    public void start() {
        
        telemetry.speak("Let's Go!");
        telemetry.update();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_x;
        double x = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        boolean RTB;
        double RTD = gamepad2.right_trigger;
        RTB = RTD == 1;

        boolean LTB;
        double LTD = gamepad2.left_trigger;
        LTB = LTD == 1;

        double toggleCheck = 0;
        if (gamepad1.a) {
            toggleCheck += 1;}


        if (toggleCheck%2==1) {
            robot.FrontLeft.setPower(((y + x + rx) / denominator) * .2);
            robot.BackLeft.setPower(((y - x + rx) / denominator) * .2);
            robot.FrontRight.setPower(((y - x - rx) / denominator) * .2);
            robot.BackRight.setPower(((y + x - rx) / denominator) * .2);
        }
        else {
            robot.FrontLeft.setPower((y + x + rx) / denominator);
            robot.BackLeft.setPower((y - x + rx) / denominator);
            robot.FrontRight.setPower((y - x - rx) / denominator);
            robot.BackRight.setPower((y + x - rx) / denominator);
        }

        if (gamepad2.right_bumper && robot.armPosition > robot.MIN_POSITION){
            robot.armPosition -= .1;}

        if (RTB && robot.armPosition < robot.MAX_POSITION){
            robot.armPosition += .1;}

        robot.Servo.setPosition(robot.armPosition);

        if (gamepad2.left_bumper && robot.gripPosition > robot.MIN_POSITION){
            robot.gripPosition -= .1;}

        if (LTB && robot.gripPosition < robot.MAX_POSITION){
            robot.gripPosition += .1;}

        if (gamepad2.dpad_up)
            robot.contPower = .2;
        else if (gamepad2.dpad_down)
            robot.contPower = -.2;
        else
            robot.contPower = 0;

        if (gamepad1.x){
            telemetry.speak("Ee-Zee");}

        double FL = robot.FrontLeft.getPower();
        double BL = robot.BackLeft.getPower();
        double FR = robot.FrontRight.getPower();
        double BR = robot.BackRight.getPower();

        if ((FL+BL+FR+BR==4) | (FL+BL+FR+BR==4*.2)){
            telemetry.addLine("forward");}
        else if ((FL+BL>-2 && FL+BL<2 && FR+BR==2) | (FL+BL>-2*.2 && FL+BL<2*.2 && FR+BR==2*.2)){
            telemetry.addLine("front right");}
        else if ((FR+BR>-2 && FR+BR<2 && FL+BL==2) | (FR+BR>-2*.2 && FR+BR<2*.2 && FL+BL==2*.2)){
            telemetry.addLine("front left");}
        else if ((FL+BL+FR+BR==-4) | (FL+BL+FR+BR==-4*.2)){
            telemetry.addLine("backwards");}
        else if ((FR+BR>-2 && FR+BR<2 && FL+BL==-2) | (FR+BR>-2*.2 && FR+BR<2*.2 && FL+BL==-2*.2)){
            telemetry.addLine("back right");}
        else if ((FL+BL>-2 && FL+BL<2 && FR+BR==-2) | (FL+BL>-2*.2 && FL+BL<2*.2 && FR+BR==-2*.2)){
            telemetry.addLine("back left");}
        else if ((FL+BL==2 && FR+BR==-2) | (FL+BL==2*.2 && FR+BR==-2*.2)){
            telemetry.addLine("left turn");}
        else if ((FL+BL==-2 && FR+BR==2) | (FL+BL==-2*.2 && FR+BR==2*.2)){
            telemetry.addLine("right turn");}
        else if ((FL+BR==-2 && FR+BL==2) | (FL+BR==-2*.2 && FR+BL==2*.2)){
            telemetry.addLine("left strafe");}
        else if ((FL+BR==2 && FR+BL==-2) | (FL+BR==2*.2 && FR+BL==-2*.2)){
            telemetry.addLine("right strafe");}
        else {
            telemetry.addLine("still");}

        if (toggleCheck % 2 == 1){
            telemetry.addData("Speed", "Slow");}
        else{
            telemetry.addData("Speed", "Fast");}

        telemetry.addData("Servo", robot.Servo.getPosition());
        //telemetry.addData("Arm", robot.armPosition);
        //telemetry.addData("Cont", robot.contPosition);
        //telemetry.addData("Strafe",rx);
      //telemetry.addData("FL",FL);
      //telemetry.addData("BL",BL);
      //telemetry.addData("FR",FR);
      //telemetry.addData("BR",BR);
      telemetry.update();
    }
}