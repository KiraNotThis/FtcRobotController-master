package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Basket 3", group = "Robot")
public class Basket3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);
        imu.resetYaw();
        sleep(300);

        // IMU settings

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        Vertical = hardwareMap.get(DcMotor.class, "Vertical");
        Horizontal = hardwareMap.get(DcMotor.class, "Horizontal");

        touchHorizontal = hardwareMap.get(TouchSensor.class, "sensor_touch_hor");
        touchVertical = hardwareMap.get(TouchSensor.class, "sensor_touch");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");

        VerRotate.setPosition(verrotate_chamber);
        VerClaw.setPosition(verclaw_close);
        HorRotate.setPosition(horrotate_middle);
        HorClaw.setPosition(horclaw_open);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        Horizontal.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        encoders();
        encodersVH();
        /*while (!isStarted()) {
            telemetry.addData("IMU Heading", "%.2f", getHeading());
            telemetry.update();
        }*/

        while (opModeInInit()) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.update();
        }


        waitForStart();
        double constant_angle = getHeading();//the first ideal zero of robot

        /*horizontalForward(1100, 0.5);
        VerClaw.setPosition(verclaw_close);
        safeSleep(500);
        HorClaw.setPosition(horclaw_open);
        safeSleep(500);*/



        // Placing 1st sample
        Thread sliderBasket1 = new Thread(() -> {
            if (opModeIsActive()) verticalUp(high_basket, -1);
        });
        
        Thread driveFirst = new Thread(() -> {
            if (opModeIsActive()) driveStraight(0.7, 30, constant_angle, 500, 0.6, 0.05);
            if (opModeIsActive()) driveSide(-0.8, 65, constant_angle, 500, 0.7, 0.05);
            if (opModeIsActive()) driveStraight(-0.5, way_basket, constant_angle, 0, 1, 0.05);
        });
        
        sliderBasket1.start();
        driveFirst.start();
        
        try {
            sliderBasket1.join();
            driveFirst.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        placingsample();

        // Taking 2nd from the ground
        Thread sliderZero1 = new Thread(() -> {
            if (opModeIsActive()) verticalZero(1);
            if (opModeIsActive()) VerClaw.setPosition(verclaw_open);
        });

        Thread driveSecond = new Thread(() -> {
            if (opModeIsActive()) driveStraight(0.5, way_basket, constant_angle, 0, 1, 0.05);
            if (opModeIsActive()) driveSide(0.5, way_sample_1, constant_angle, 0, 1, 0.05);

        });

        sliderZero1.start();
        driveSecond.start();

        try {
            sliderZero1.join();
            driveSecond.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        takingsample();

        //Transfer 2nd sample to VerClaw
        Thread sliderBasket2 = new Thread(() -> {
            if (opModeIsActive()) VerClaw.setPosition(verclaw_close);
            safeSleep(500);
            if (opModeIsActive()) HorClaw.setPosition(horclaw_open);
            safeSleep(500);
            if (opModeIsActive()) verticalUp(high_basket, -1);

        });

        //Placing 2nd in the basket
        Thread driveThird = new Thread(() -> {
            if (opModeIsActive()) driveSide(-0.5, way_sample_1 + 5, constant_angle, 0, 1, 0.05);
            if (opModeIsActive()) driveStraight(-0.5, way_basket, constant_angle, 0, 1, 0.05);
        });

        sliderBasket2.start();
        driveThird.start();

        try {
            sliderBasket2.join();
            driveThird.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        placingsample();

        //Taking 3rd sample from the ground
        Thread sliderZero2 = new Thread(() -> {
            if (opModeIsActive()) verticalZero(1);
            if (opModeIsActive()) VerClaw.setPosition(verclaw_open);
        });

        Thread driveFourth = new Thread(() -> {
            if (opModeIsActive()) driveStraight(0.5, way_basket, constant_angle, 0, 1, 0.05);
            if (opModeIsActive()) driveSide(0.5, way_sample_2, constant_angle, 0, 1, 0.05);
        });

        sliderZero2.start();
        driveFourth.start();

        try {
            sliderZero2.join();
            driveFourth.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        takingsample();

        //Transfer 3rd sample to VerClaw
        Thread sliderBasket3 = new Thread(() -> {
            if (opModeIsActive()) VerClaw.setPosition(verclaw_close);
            safeSleep(500);
            if (opModeIsActive()) HorClaw.setPosition(horclaw_open);
            safeSleep(500);
            if (opModeIsActive()) verticalUp(high_basket, -1);
        });

        //Placing 3rd sample in the basket
        Thread driveFifth = new Thread(() -> {
            if (opModeIsActive()) driveSide(-0.5, way_sample_2 + 5, constant_angle, 0, 1, 0.05);
            if (opModeIsActive()) driveStraight(-0.5, way_basket_3, constant_angle, 0, 1, 0.05);
        });

        sliderBasket3.start();
        driveFifth.start();

        try {
            sliderBasket3.join();
            driveFifth.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        placingsample();

    }

    private void takingsample() {
        horizontalForward(-400, -0.8);
        HorRotate.setPosition(horrotate_ground);
        safeSleep(500);
        horizontal_by_distance(-0.8);
        HorClaw.setPosition(horclaw_close);
        safeSleep(500);
        HorRotate.setPosition(horrotate_transfer);
        safeSleep(500);
        horizontalForward(1150, 0.8);
    }
    private void safeSleep(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            sleep(10);
        }
    }


    private void placingsample() {
        VerRotate.setPosition(verrotate_player);
        safeSleep(800);
        VerClaw.setPosition(verclaw_open);
        safeSleep(500);
        VerClaw.setPosition(verclaw_close);
        safeSleep(500);
        VerRotate.setPosition(verrotate_chamber);
        safeSleep(500);
    }

    private void encodersVH() {
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void encoders() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        encoders();

        int targetTicks = (int) (PULSES_PER_CM * distance);
        double slowdownStart = targetTicks * slowdownStartFactor; // Point where deceleration begins
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {
            int currentTicks = Math.abs(LeftFront.getCurrentPosition());
            double currentAngle = getHeading();
            double angleError = currentAngle - startAngle;

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
        movestop();
    }
    //______________________________Straight*_________________________________//


    //______________________________*Side_____________________________________//

    public void driveSide(double driveSpeed, double distance, double startAngle, double rampUpTime, double slowdownStartFactor, double kP) {
        encoders();

        int targetTicks = (int) (PULSES_PER_CM * distance);
        double slowdownStart = targetTicks * slowdownStartFactor; // Point where deceleration begins
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {
            int currentTicks = Math.abs(LeftFront.getCurrentPosition());
            double currentAngle = getHeading();
            double angleError = currentAngle - startAngle;

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
        }
        movestop();
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
    public void horizontal_by_distance(double power) {

        // Двигаемся, пока расстояние больше 5 см
        while (opModeIsActive() && distanceSensor.getDistance(DistanceUnit.CM) > 5.5) {
            Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Horizontal.setPower(power);
        }

        // Останавливаем мотор
        Horizontal.setPower(0);
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Устанавливаем цель: ещё -300 тиков
        Horizontal.setTargetPosition(-250);
        Horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Horizontal.setPower(power);

        // Ждём, пока мотор едет к цели
        while (opModeIsActive() && Horizontal.isBusy()) {
            // Можно добавить отладку:
            // telemetry.addData("Current Pos", Horizontal.getCurrentPosition());
            // telemetry.update();
        }

        // После достижения позиции — остановить мотор
        Horizontal.setPower(0);
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




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
    private void movestop() {
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

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}