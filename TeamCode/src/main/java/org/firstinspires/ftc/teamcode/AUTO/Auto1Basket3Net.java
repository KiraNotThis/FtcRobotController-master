package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

@Autonomous(name = "Auto 1 basket 3 Net ", group = "Robot")
public class Auto1Basket3Net extends LinearOpMode {
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");

        //Define orientation of a robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // IMU settings
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);

        // Reset angle before starting
        imu.resetYaw();

        touchVertical = hardwareMap.get(TouchSensor.class, "sensor_touch");//in configuration change t

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        Vertical = hardwareMap.get(DcMotor.class, "Vertical");
        Horizontal = hardwareMap.get(DcMotor.class, "Horizontal");

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.11);
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        VerClaw.setPosition(0.6);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.45);
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.52);


        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        Vertical.setDirection(DcMotor.Direction.FORWARD);
        Horizontal.setDirection(DcMotor.Direction.FORWARD);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        double contstant_angle = getHeading();//the first ideal zero of robot

        telemetry.addData("Vertical", Vertical.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Horizontal", Horizontal.getCurrentPosition());
        telemetry.update();
        //GO TO THE BASKET 1
        driveSide(0.2, 4, contstant_angle, 0, 1, 0.05);
        verticalUp(-4300, -1);//let the VerSlider go up
        driveStraight(0.6, 32, contstant_angle, 0, 1, 0.05);
        sleep(200);
        //PLACE 1ST SAMPLE IN BASKET
        VerClaw.setPosition(0.3);//open the VerClaw
        sleep(200);

        //GO TO THE WALL
        driveSide(0.6, 55, contstant_angle - 90, 0, 1, 0.05);
        driveSide(-0.6, 95, contstant_angle - 90, 0, 1, 0.05);
        driveStraight(0.6, 20, contstant_angle - 90, 0, 1, 0.05);
        verticalZero(1);//let the VerSlider go down
        //driveStraight(0.4, 10,contstant_angle + 90, 0, 1, 0.05);
        //driveSide(-0.4, 30, contstant_angle - 90, 0, 1, 0.05);

        //GO FOR SAMPLE
        //driveSide(0.4, 8.4, contstant_angle-90, 10, 0.5, 0.05);
        horizontalForward(-20, -0.5);
        HorClaw.setPosition(0.27);//open claw
        HorRotate.setPosition(0.94);//rotate to the sample


        horizontalForward(-1280, -0.5);


        horizontalForward(70, 0.5);
        HorClaw.setPosition(0.52);//close claw
        sleep(500);
        HorRotate.setPosition(0.47);//rotate to vertical claw
        horizontalForward(1430, 0.5);//close horizontal slider
        VerClaw.setPosition(0.6);
        sleep(500);
        verticalUp(-4300, -1);//let the VerSlider go up

        driveStraight(0.3, 40, contstant_angle - 180, 0, 1, 0.05);
        VerClaw.setPosition(0.3);
        sleep(500);
    }
    //______________________________*Straight_________________________________//

    /**
     * Moves the robot forward while maintaining direction using IMU.
     * Supports acceleration, deceleration, and configurable IMU correction.
     *
     * @param driveSpeed          The maximum driving speed (0.0 - 1.0).
     * @param distance            The target distance to travel in centimeters.
     * @param startAngle          The initial IMU heading to maintain.
     * @param rampUpTime          Time in milliseconds for acceleration (0 disables acceleration).
     * @param slowdownStartFactor The fraction of the total distance where deceleration starts (1.0 disables deceleration).
     * @param kP                  The proportional correction factor for IMU drift.
     */
    public void driveStraight(double driveSpeed, double distance, double startAngle, double rampUpTime, double slowdownStartFactor, double kP) {
        // Reset encoders before starting movement
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targetTicks = (int) (PULSES_PER_CM * distance);
        double slowdownStart = targetTicks * slowdownStartFactor; // Point where deceleration begins
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {
            int currentTicks = Math.abs(LeftFront.getCurrentPosition());
            double currentAngle = getHeading();
            double angleError = startAngle - currentAngle;

            // Acceleration logic
            double speedFactor = 1.0;
            if (rampUpTime > 0) {
                long elapsedTime = System.currentTimeMillis() - startTime;
                speedFactor = Math.min(1.0, elapsedTime / rampUpTime);
            }

            // Deceleration logic
            double slowdownFactor = 1.0;
            if (slowdownStartFactor < 1.0 && currentTicks > slowdownStart) {
                slowdownFactor = Math.max(0.2, 1.0 - ((currentTicks - slowdownStart) / (targetTicks - slowdownStart)));
            }

            double adjustedSpeed = driveSpeed * slowdownFactor * speedFactor;
            double correction = angleError * kP;  // Use kP for IMU correction

            telemetry.addData("Straight Movement", "Target: %5d, Current: %5d", targetTicks, currentTicks);
            telemetry.addData("IMU Angle", "%.2f", currentAngle);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            // Apply correction for maintaining a straight path
            LeftFront.setPower(adjustedSpeed + correction);
            LeftBack.setPower(adjustedSpeed + correction);
            RightFront.setPower(adjustedSpeed - correction);
            RightBack.setPower(adjustedSpeed - correction);
        }

        // Allow "coasting" before full stop
        LeftFront.setPower(0.05);
        LeftBack.setPower(0.05);
        RightFront.setPower(0.05);
        RightBack.setPower(0.05);
        sleep(200);

        // Full stop
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }


    //______________________________Straight*_________________________________//


    //______________________________*Side_____________________________________//

    public void driveSide(double driveSpeed, double distance, double startAngle, double rampUpTime, double slowdownStartFactor, double kP) {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targetTicks = (int) (PULSES_PER_CM * distance);
        double slowdownStart = targetTicks * slowdownStartFactor; // Point where deceleration begins
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {
            int currentTicks = Math.abs(LeftFront.getCurrentPosition());
            double currentAngle = getHeading();
            double angleError = startAngle - currentAngle;

            // Acceleration logic
            double speedFactor = 1.0;
            if (rampUpTime > 0) {
                long elapsedTime = System.currentTimeMillis() - startTime;
                speedFactor = Math.min(1.0, elapsedTime / rampUpTime);
            }

            // Deceleration logic
            double slowdownFactor = 1.0;
            if (slowdownStartFactor < 1.0 && currentTicks > slowdownStart) {
                slowdownFactor = Math.max(0.2, 1.0 - ((currentTicks - slowdownStart) / (targetTicks - slowdownStart)));
            }

            double adjustedSpeed = driveSpeed * slowdownFactor * speedFactor;
            double correction = angleError * kP;  // Use kP for IMU correction

            telemetry.addData("Straight Movement", "Target: %5d, Current: %5d", targetTicks, currentTicks);
            telemetry.addData("IMU Angle", "%.2f", currentAngle);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            // Apply correction for maintaining a straight path
            LeftFront.setPower(adjustedSpeed + correction);
            LeftBack.setPower(-adjustedSpeed + correction);
            RightFront.setPower(-adjustedSpeed - correction);
            RightBack.setPower(adjustedSpeed - correction);

                /*telemetry.addData("Angle", "%.2f", currentAngle);
                telemetry.addData("Correction", "%.2f", angleError);
                telemetry.update();

                // Коррекция направления на основе угла
                LeftFront.setPower(adjustedSpeed + angleError * 0.05);
                LeftBack.setPower(-adjustedSpeed + angleError * 0.05);
                RightFront.setPower(-adjustedSpeed - angleError * 0.05);
                RightBack.setPower(adjustedSpeed - angleError * 0.05);*/
        }

        LeftFront.setPower(0.05);
        LeftBack.setPower(0.05);
        RightFront.setPower(0.05);
        RightBack.setPower(0.05);
        sleep(200);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }
    //______________________________Side*_____________________________________//


    //___________________________VerticalPosition*____________________________//
    public void verticalUp(double position, double power) {
        Vertical.setTargetPosition((int) position);
        Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Vertical.setPower(power);

        while (opModeIsActive() && Vertical.isBusy()) {
            telemetry.addData("Current Position", Vertical.getCurrentPosition());
            telemetry.update();
        }
    }
    //___________________________VerticalPosition*____________________________//

    //___________________________VerticalZero*________________________________//
    public void verticalZero(double power) {

        while (opModeIsActive() && !touchVertical.isPressed()) {
            Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Vertical.setPower(power);  // Keep moving down
            telemetry.addData("Vertical Motor", "Moving Down");
            telemetry.update();
        }

        // Stop the vertical motor once the sensor is pressed
        Vertical.setPower(0);
        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //___________________________VerticalZero*_______________________________//

    //___________________________HorizontalPosition*____________________________//
    public void horizontalForward(double position, double power) {
        Horizontal.setTargetPosition((int) position);
        Horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Horizontal.setPower(power);

        while (opModeIsActive() && Horizontal.isBusy()) {
            telemetry.addData("Current Position", Horizontal.getCurrentPosition());
            telemetry.update();
        }
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //___________________________HorizontalPosition*____________________________//

    //___________________________DriveRotate____________________________________//
    public void driveRotate(double rotateSpeed, double angle) {
        imu.resetYaw();

        while (opModeIsActive() && Math.abs(getHeading()) < angle) {

            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();

            LeftFront.setPower(rotateSpeed);
            LeftBack.setPower(rotateSpeed);
            RightFront.setPower(-rotateSpeed);
            RightBack.setPower(-rotateSpeed);
        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        sleep(300);
    }
    //___________________________DriveRotate____________________________________//

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}