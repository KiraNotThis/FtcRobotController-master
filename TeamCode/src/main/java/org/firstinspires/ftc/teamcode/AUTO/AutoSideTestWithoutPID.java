package org.firstinspires.ftc.teamcode.AUTO;


import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.LeftBack;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.LeftFront;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.PULSES_PER_CM;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.RightBack;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.RightFront;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.imu;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Side Test Without PID", group = "Robot")
public class AutoSideTestWithoutPID extends LinearOpMode {

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


        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        encoders();

        while (opModeInInit()) {
            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();

        double contstant_angle = getHeading();//the first ideal zero of robot
        driveSide(0.7, 100, contstant_angle, 0, 1, 0.05);


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

    //______________________________Side*_____________________________________//


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