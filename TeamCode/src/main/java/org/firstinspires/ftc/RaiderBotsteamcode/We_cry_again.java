package org.firstinspires.ftc.RaiderBotsteamcode;


    /** This is the code used for the field-centric driving tutorial
     This is by no means a perfect code
     There are a number of improvements that can be made
     So, feel free to add onto this and make it better
     */

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

    /**
     * feel free to change the name or group of your class to better fit your robot
     */
    @TeleOp(name = "DriverRelativeControl2", group = "tutorial")
    public class We_cry_again extends LinearOpMode {

        /**
         * make sure to change these motors to your team's preference and configuration
         */
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backRight;
        private DcMotor backLeft;

        public BNO055IMU imu;

        Orientation angles;
        Acceleration gravity;

        @Override
        public void runOpMode() {

            /**
             * you can change the variable names to make more sense
             */
            double driveTurn;
            //double driveVertical;
            //double driveHorizontal;

            double gamepadXCoordinate;
            double gamepadYCoordinate;
            double gamepadHypot;
            double gamepadDegree;
            double robotDegree;
            double movementDegree;
            double gamepadXControl;
            double gamepadYControl;

            /**
             * make sure to change this to how your robot is configured
             */
            frontLeft = hardwareMap.dcMotor.get("FrontLeft");
            frontRight = hardwareMap.dcMotor.get("FrontRight");
            backRight = hardwareMap.dcMotor.get("BackRight");
            backLeft = hardwareMap.dcMotor.get("BackLeft");

            //might need to change the motors being reversed
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            /**
             * make sure you've configured your imu properly and with the correct device name
             */
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            composeTelemetry();

            waitForStart();

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


            while (opModeIsActive()) {
                driveTurn = -gamepad1.left_stick_x;
                //driveVertical = -gamepad1.right_stick_y;
                //driveHorizontal = gamepad1.right_stick_x;

                gamepadXCoordinate = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
                gamepadYCoordinate = gamepad1.right_stick_y; //this simply gives our y vaue relative to the driver
                gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                //finds just how much power to give the robot based on how much x and y given by gamepad
                //range.clip helps us keep our power within positive 1
                // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
                gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
                //the inverse tangent of opposite/adjacent gives us our gamepad degree
                robotDegree = getAngle();
                //gives us the angle our robot is at
                movementDegree = gamepadDegree - robotDegree;
                //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
                gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the adjacent side, we can get our needed x value to power our motors
                gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the opposite side, we can get our needed y value to power our motors

                /**
                 * again, make sure you've changed the motor names and variables to fit your team
                 */

                //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
                //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
                //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
                frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
                backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

            /*frontRight.setPower(driveVertical - driveHorizontal + driveTurn);
            backRight.setPower(driveVertical + driveHorizontal + driveTurn);
            frontLeft.setPower(driveVertical + driveHorizontal - driveTurn);
            backLeft.setPower(driveVertical - driveHorizontal - driveTurn);*/
                telemetry.addData("FL:", frontLeft.getPower());
                telemetry.addData("FR:", frontLeft.getPower());
                telemetry.addData("BL:", backLeft.getPower());
                telemetry.addData("BR:", backRight.getPower());
                telemetry.addData("Angle", getAngle());
                telemetry.addData("Hypot", gamepadHypot);
                telemetry.addData("Degree", gamepadDegree);
                telemetry.addData("MDegree", movementDegree);
                telemetry.addData("x", gamepadXControl);
                telemetry.addData("y",gamepadYControl);
                telemetry.update();
            }

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

