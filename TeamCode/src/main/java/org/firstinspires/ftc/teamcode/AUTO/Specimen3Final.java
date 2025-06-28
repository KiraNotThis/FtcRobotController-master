package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

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
@Disabled
@Autonomous(name = "3K Specimen", group = "Robot")
public class Specimen3Final extends LinearOpMode {

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
        //Define orientation of a robot

        // IMU settings

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");
        Vertical = hardwareMap.get(DcMotor.class, "Vertical");
        Horizontal = hardwareMap.get(DcMotor.class, "Horizontal");

        touchHorizontal = hardwareMap.get(TouchSensor.class, "sensor_touch_hor");
        touchVertical = hardwareMap.get(TouchSensor.class, "sensor_touch");

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

        encodersVH();
        encoders();
        while (!isStarted()) {
            telemetry.addData("IMU Heading", "%.2f", getHeading());
            telemetry.update();
        }

        while (opModeInInit()) {
            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        double contstant_angle = getHeading();//the first ideal zero of robot

        // === STEP 1: FIRST SLIDE UP AND MOVE ===
        Thread sliderMiddle1 = new Thread(() -> {
            if (opModeIsActive()) verticalUp(middle_chamber, -0.9);
        });
        Thread driveFirst = new Thread(() -> {
            if (opModeIsActive()) driveStraight(1, 63, contstant_angle, 500, 0.45, 0.05);
        });
        sliderMiddle1.start();
        driveFirst.start();
        try {
            sliderMiddle1.join();
            driveFirst.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        // === STEP 2: PLACE FIRST SPECIMEN ===
        verticalUp(high_chamber, -1);
        VerClaw.setPosition(verclaw_open);
        safeSleep(500);

        // === STEP 3: GO FOR SAMPLE ===
        Thread sliderZero1 = new Thread(() -> {
            if (opModeIsActive()) verticalZero(1);
        });
        Thread driveSecond = new Thread(() -> {
            if (opModeIsActive()) driveStraight(-0.3, 15, contstant_angle, 0, 1, 0.05);
            safeSleep(50);
            if (opModeIsActive()) driveSide(1, 137, contstant_angle, 500, 0.5, 0.05);
        });
        sliderZero1.start();
        driveSecond.start();
        try {
            sliderZero1.join();
            driveSecond.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        horizontalForward(-800, -0.7);
        HorRotate.setPosition(horrotate_ground);
        safeSleep(500);
        HorClaw.setPosition(horclaw_close);
        safeSleep(500);
        HorRotate.setPosition(0.74);
        safeSleep(500);

// === STEP 4: DELIVER SPECIMEN ===
        Thread horizontalForward1 = new Thread(() -> {
            if (opModeIsActive()) horizontalForward(100, 0.7);
            VerRotate.setPosition(verrotate_player);
        });
        Thread driveThird = new Thread(() -> {
            if (opModeIsActive()) driveStraight(1, 105, contstant_angle - 135, 0, 0.9, 0.05);
        });
        horizontalForward1.start();
        driveThird.start();
        try {
            horizontalForward1.join();
            driveThird.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        HorClaw.setPosition(horclaw_open);
        safeSleep(500);

        Thread rotate = new Thread(() -> {
            HorRotate.setPosition(horrotate_middle);
            horizontalForward(450, 0.7);
        });
        Thread drive4 = new Thread(() -> {
            driveStraight(-1, 10, contstant_angle - 135, 0, 1, 0.05);
            driveStraight(-1, 95, contstant_angle, 0, 1, 0.05);
        });
        rotate.start();
        drive4.start();
        try {
            rotate.join();
            drive4.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        driveStraight(-0.3, 9, contstant_angle, 0, 1, 0.05);
        VerClaw.setPosition(verclaw_close);
        safeSleep(500);
        VerRotate.setPosition(verrotate_chamber);

// === STEP 5: GO TO CHAMBER 2 ===
        Thread sliderMiddle2 = new Thread(() -> {
            verticalUp(middle_chamber, -1);
        });
        Thread drive5 = new Thread(() -> {
            driveStraight(0.8, 7, contstant_angle, 0, 1, 0.05);
            driveSide(-0.8, 145, contstant_angle, 450, 0.7, 0.05);
            driveStraight(0.8, 57, contstant_angle, 500, 0.5, 0.05);
        });
        sliderMiddle2.start();
        drive5.start();
        try {
            sliderMiddle2.join();
            drive5.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        verticalUp(high_chamber, -1);
        VerClaw.setPosition(verclaw_open);
        safeSleep(500);
        VerRotate.setPosition(verrotate_player);
        safeSleep(500);
        VerClaw.setPosition(verclaw_open);

// === STEP 6: OBSERVATION 2 ===
        Thread sliderZero2 = new Thread(() -> {
            verticalZero(0.9);
        });
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
            Thread.currentThread().interrupt();
        }

        VerClaw.setPosition(verclaw_close);
        safeSleep(500);
        VerRotate.setPosition(verrotate_chamber);

// === STEP 7: CHAMBER 3 ===
        Thread sliderMiddle3 = new Thread(() -> {
            verticalUp(middle_chamber, -0.9);
        });
        Thread driveFifth = new Thread(() -> {
            driveStraight(1, 7, contstant_angle, 0, 0.5, 0.05);
            driveSide(-1, 97, contstant_angle, 500, 0.5, 0.05);
            driveStraight(1, 65, contstant_angle, 500, 0.5, 0.05);
        });
        sliderMiddle3.start();
        driveFifth.start();
        try {
            sliderMiddle3.join();
            driveFifth.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        verticalUp(high_chamber, -1);
        VerClaw.setPosition(verclaw_open);

// === STEP 8: PARKING ===
        driveDiagonal(1, 95);
        driveSide(1, 80, contstant_angle, 5, 0.8, 0.05);

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
    private void encodersVH() {
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void safeSleep(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            sleep(10);
        }
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