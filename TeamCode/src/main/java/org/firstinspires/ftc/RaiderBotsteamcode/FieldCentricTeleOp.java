
package org.firstinspires.ftc.RaiderBotsteamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

    /**
     * feel free to change the name or group of your class to better fit your robot
     */
    @TeleOp(name = "DriverRelativeControl", group = "tutorial")
    public class FieldCentricTeleOp extends OpMode {

        RaiderBotsHwMap robot= new RaiderBotsHwMap();

        public BNO055IMU imu;

        Orientation angles;
        Acceleration gravity;
        double driveTurn;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;


        @Override
        public void init() {
            robot.init(hardwareMap);

            robot.BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            composeTelemetry();

            telemetry.speak("Initialized");
            telemetry.addLine("Initialized");
            telemetry.update();

        }
        @Override
        public void start() {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        }

        @Override
        public void loop() {

            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = gamepad1.right_stick_x;
            gamepadYCoordinate = gamepad1.right_stick_y;
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
            gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
            robotDegree = getAngle();
            movementDegree = gamepadDegree - robotDegree;
            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;

            robot.FrontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn);
            robot.BackRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn);
            robot.FrontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn);
            robot.BackLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn);

            telemetry.addData("FL:", robot.FrontLeft.getPower());
            telemetry.addData("FR:", robot.FrontRight.getPower());
            telemetry.addData("BL:", robot.BackLeft.getPower());
            telemetry.addData("BR:", robot.BackRight.getPower());
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Hypot", gamepadHypot);
            telemetry.addData("Degree", gamepadDegree);
            telemetry.addData("MDegree", movementDegree);
            telemetry.addData("driveTurn", driveTurn);
            telemetry.addData("x", gamepadXControl);
            telemetry.addData("y",gamepadYControl);



            telemetry.update();
        }

        void composeTelemetry() {
            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                }
            });
        }


        //allows us to quickly get our z angle
        public double getAngle() {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }

