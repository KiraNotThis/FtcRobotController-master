package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "PID Calibration", group = "Calibration")
public class AutoPIDCalibration extends LinearOpMode {

    double kP = 0.05;
    double kI = 0.0;
    double kD = 0.0;

    double distanceCM = 100;
    double speed = 0.5;

    boolean isMoving = false;

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
        sleep(300); // Дать IMU время сброситься

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");

        VerRotate.setPosition(0.12);
        VerClaw.setPosition(0.51);
        HorRotate.setPosition(0.08);
        HorClaw.setPosition(0.5);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        encoders();

        while (!isStarted()) {
            telemetry.addData("Waiting to start", "Use DPAD and bumpers to adjust PID");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("IMU Heading", "%.2f", getHeading());
            telemetry.addData("Press A to run test", "");
            telemetry.update();

            adjustPIDWithGamepad();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (!isMoving) {
                adjustPIDWithGamepad();

                if (gamepad1.a) {
                    isMoving = true;

                    double initialHeading = getHeading();
                    telemetry.addData("Start Heading", "%.2f", initialHeading);
                    telemetry.update();

                    driveStraightPID(speed, distanceCM, 0, kP, kI, kD);  // целевой угол = 0

                    double finalHeading = getHeading();
                    telemetry.addData("Final Heading", "%.2f", finalHeading);
                    telemetry.update();

                    isMoving = false;
                }
            }

            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("IMU Heading", "%.2f", getHeading());
            telemetry.update();
        }
    }

    private void adjustPIDWithGamepad() {
        if (gamepad1.dpad_up) {
            kP += 0.001;
            sleep(150);
        } else if (gamepad1.dpad_down) {
            kP = Math.max(0, kP - 0.001);
            sleep(150);
        }

        if (gamepad1.dpad_right) {
            kI += 0.0005;
            sleep(150);
        } else if (gamepad1.dpad_left) {
            kI = Math.max(0, kI - 0.0005);
            sleep(150);
        }

        if (gamepad1.right_bumper) {
            kD += 0.001;
            sleep(150);
        } else if (gamepad1.left_bumper) {
            kD = Math.max(0, kD - 0.001);
            sleep(150);
        }
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

    public void driveStraightPID(double driveSpeed, double distanceCM, double targetAngle,
                                 double kP, double kI, double kD) {

        encoders();
        int targetTicks = (int) (PULSES_PER_CM * distanceCM);

        double integral = 0;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftBack.getCurrentPosition()) < targetTicks) {

            double currentAngle = getHeading();
            double error = AngleUnit.normalizeDegrees(currentAngle - targetAngle);

            long now = System.currentTimeMillis();
            double deltaTime = Math.max(1e-6, (now - lastTime) / 1000.0);

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            double correction = kP * error + kI * integral + kD * derivative;
            correction = Math.max(-0.1, Math.min(0.1, correction));

            double leftPower = Math.max(-1, Math.min(1, driveSpeed + correction));
            double rightPower = Math.max(-1, Math.min(1, driveSpeed - correction));

            LeftFront.setPower(leftPower);
            LeftBack.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightBack.setPower(rightPower);

            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.addData("Encoder Ticks", LeftFront.getCurrentPosition());
            telemetry.update();

            lastError = error;
            lastTime = now;
        }

        movestop();
    }

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
