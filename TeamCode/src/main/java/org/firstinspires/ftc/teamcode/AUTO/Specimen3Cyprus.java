package org.firstinspires.ftc.teamcode.AUTO;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;
@Disabled
@Autonomous(name = "3 Specimen", group = "Robot")
public class Specimen3Cyprus extends LinearOpMode {

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

        touchVertical = hardwareMap.get(TouchSensor.class, "sensor_touch");//in configuration change the name
        touchHorizontal = hardwareMap.get(TouchSensor.class, "sensor_touch_hor");

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

        encoders();

        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        double contstant_angle = getHeading();//the first ideal zero of robot

        //GO TO THE CHAMBER 1
        Thread sliderMiddle1 = new Thread(() -> verticalUp(-1500, -0.9));
        Thread driveFirst = new Thread(() -> {
            driveStraight(1, 62, contstant_angle, 500, 0.45, 0.05);
        });

        sliderMiddle1.start();
        driveFirst.start();

        try {
            sliderMiddle1.join();
            driveFirst.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE 1st SPECIMEN
        verticalUp(-2600, -1);//place specimen 1
        VerClaw.setPosition(0.3);//to open

        //GO FOR THE SAMPLE
        Thread sliderZero1 = new Thread(() -> verticalZero(1));
        Thread driveSecond = new Thread(() -> {
            driveStraight(-0.3, 13, contstant_angle, 0, 1, 0.05);//Drive back
            sleep(50);
            driveSide(1, 160, contstant_angle, 500, 0.5, 0.05);
        });

        sliderZero1.start();
        driveSecond.start();

        try {
            sliderZero1.join();
            driveSecond.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //TAKE SAMPLE AND PUT IN OBSERVATION
        HorRotate.setPosition(0.98);//rotate to the sample
        sleep(500);
        HorClaw.setPosition(0.27);//open claw
        horizontalForward(-850, -0.5);//move the specimen a little bit
        horizontalForward(60, 0.5);//move backwards to take the specimen

        HorClaw.setPosition(0.52);//close claw
        sleep(500);
        HorRotate.setPosition(0.45);//rotate to vertical claw
        horizontalForward(790, 0.5);//close horizontal slider
        VerClaw.setPosition(0.6);// close VerClaw
        sleep(500);
        HorClaw.setPosition(0.27);//open horizontal claw
        sleep(500);
        VerRotate.setPosition(0.88);//rotate to put in the observation zone
        driveStraight(-0.8, 20, contstant_angle, 0, 1, 0.05);//Drive backwards
        VerClaw.setPosition(0.3);//to open
        driveStraight(-0.8, 5, contstant_angle + 15, 0, 1, 0.05);


        //GO TO OBSERVATION 1st
        //driveStraight(-0.8, 30, contstant_angle, 0, 1, 0.05);//Drive backwards
        driveSide(-0.3, 43, contstant_angle, 0, 0.5, 0.05);//Drive sideways
        driveStraight(-0.27, 23, contstant_angle, 0, 1, 0.05);//Drive backwards for the sample.

        //TAKE SPECIMEN FROM OBSERVATION 1st
        VerClaw.setPosition(0.6);// close VerClaw
        sleep(500);
        VerRotate.setPosition(0.11);//rotate to chamber

        //GO TO THE CHAMBER 2
        Thread sliderMiddle2 = new Thread(() -> verticalUp(-1500, -1));
        Thread driveThird = new Thread(() -> {
            driveStraight(0.8, 7, contstant_angle, 0, 1, 0.05);
            driveSide(-0.8, 95, contstant_angle, 450, 0.5, 0.05);
            driveStraight(0.8, 47, contstant_angle, 500, 0.5, 0.05);
            //driveStraight(-0.2, 5, contstant_angle, 0, 1, 0.05);
        });

        sliderMiddle2.start();
        driveThird.start();

        try {
            sliderMiddle2.join();
            driveThird.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE 2nd SPECIMEN
        verticalUp(-2600, -1);//place specimen 1
        VerClaw.setPosition(0.3);//to open
        sleep(500);
        VerClaw.setPosition(0.6);//to close
        sleep(500);
        VerRotate.setPosition(0.88);//rotate for observation
        sleep(500);
        VerClaw.setPosition(0.3);//to open

        //GO TO OBSERVATION 2nd
        Thread sliderZero2 = new Thread(() -> verticalZero(0.9));
        Thread driveFourth = new Thread(() -> {
            driveStraight(-1, 50, contstant_angle, 500, 0.5, 0.05);
            driveSide(1, 100, contstant_angle, 500, 0.5, 0.05);
            driveStraight(-0.3, 17, contstant_angle, 0, 1, 0.05);
        });

        sliderZero2.start();
        driveFourth.start();

        try {
            sliderZero2.join();
            driveFourth.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //TAKE SPECIMEN FROM OBSERVATION 3rd
        VerClaw.setPosition(0.6);// close VerClaw
        sleep(500);
        VerRotate.setPosition(0.11);//rotate to chamber

        //GO TO THE CHAMBER 3
        Thread sliderMiddle3 = new Thread(() -> verticalUp(-1500, -0.9));
        Thread driveFifth = new Thread(() -> {
            driveStraight(1, 7, contstant_angle, 0, 0.5, 0.05);
            driveSide(-1, 97, contstant_angle, 500, 0.5, 0.05);
            driveStraight(1, 55, contstant_angle, 500, 0.5, 0.05);
            //driveStraight(-0.2, 5, contstant_angle, 0, 1, 0.05);
        });

        sliderMiddle3.start();
        driveFifth.start();

        try {
            sliderMiddle3.join();
            driveFifth.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE 3rd SPECIMEN
        verticalUp(-2600, -1);//place specimen 1
        VerClaw.setPosition(0.3);//to open

        //GO PARKING
        driveDiagonal(1, 95);//Drive diagonal to the observation
        driveSide(1, 80, contstant_angle, 5, 0.8, 0.05);//Drive sideways

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
        }
        movestop();
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
    //______________________________Side*_____________________________________//


    //___________________________*Diagonal____________________________________//

    public void driveDiagonal(double driveSpeed, double distance) {
        encoders();

        int targetTicks = (int) (PULSES_PER_CM * distance);

        double slowdownStart = targetTicks * 0.7;

        while (opModeIsActive() && Math.abs(RightFront.getCurrentPosition()) < targetTicks) {
            int currentTicks = Math.abs(RightFront.getCurrentPosition());
            double slowdownFactor = 1.0;

            if (currentTicks > slowdownStart) {
                slowdownFactor = Math.max(0.2, 1.0 - ((currentTicks - slowdownStart) / (targetTicks - slowdownStart)));
            }

            double adjustedSpeed = driveSpeed * slowdownFactor;

            telemetry.addData("Position", "%5d / %5d", currentTicks, targetTicks);
            telemetry.addData("Speed Factor", "%.2f", slowdownFactor);
            telemetry.update();

            LeftFront.setPower(0);
            LeftBack.setPower(-adjustedSpeed);
            RightFront.setPower(-adjustedSpeed);
            RightBack.setPower(0);
        }

        LeftBack.setPower(-0.05);
        RightFront.setPower(-0.05);

        sleep(200);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }
    //___________________________Diagonal*____________________________________//

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

    //___________________________HorizontalZero*____________________________//
    public void horizontalZero(double power) {

        while (opModeIsActive() && !touchHorizontal.isPressed()) {
            Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Horizontal.setPower(power);  // Keep moving down
        }

        // Stop the vertical motor once the sensor is pressed
        Horizontal.setPower(0);
    }
    //___________________________HorizontalalZero*____________________________//

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}