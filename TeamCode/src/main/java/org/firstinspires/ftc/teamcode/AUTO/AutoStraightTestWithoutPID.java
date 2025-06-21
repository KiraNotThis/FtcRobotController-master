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

@Autonomous(name = "Straight Test Without PID", group = "Robot")
public class AutoStraightTestWithoutPID extends LinearOpMode {

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

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.12);
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        VerClaw.setPosition(0.51);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.08);
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.5);


        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        encoders();

        while (opModeInInit()) {
            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();

        double contstant_angle = getHeading();//the first ideal zero of robot
        driveStraight(0.7, 100);
        driveStraight(-0.7, 100);


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
                 The proportional correction factor for IMU drift.
     */
    public void driveStraight(double driveSpeed, double distance) {
        encoders();

        int targetTicks = (int) (PULSES_PER_CM * distance);
        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {


            // Apply correction for maintaining a straight path
            LeftFront.setPower(driveSpeed);
            LeftBack.setPower(driveSpeed);
            RightFront.setPower(driveSpeed);
            RightBack.setPower(driveSpeed);
        }
        movestop();
    }
    //______________________________Straight*_________________________________//


    private void movestop() {
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