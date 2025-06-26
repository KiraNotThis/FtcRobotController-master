package org.firstinspires.ftc.teamcode.AUTO;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.Horizontal;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.LeftBack;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.LeftFront;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.PULSES_PER_CM;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.RightBack;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.RightFront;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.Vertical;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.distanceSensor;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.high_basket;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horclaw_close;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horclaw_open;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_ground;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_middle;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_transfer;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.imu;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.touchHorizontal;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.touchVertical;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verclaw_close;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verclaw_open;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verrotate_chamber;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verrotate_player;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.way_basket;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.way_sample_1;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.way_sample_2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Drive Distance Sensor", group = "Robot")
public class DistanceSensorTest extends LinearOpMode {

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

        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "sensor_distance");


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

        waitForStart();
        double constant_angle = getHeading();//the first ideal zero of robot
        driveDistanceStraigt(0.2, 20, constant_angle, 0, 0.05);
    }

    private void safeSleep(long millis) {
        long endTime = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            sleep(10);
        }
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

    public void driveDistanceStraigt(double driveSpeed, double targetDistanceCm, double startAngle, double rampUpTime, double kP) {
        encoders();

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && distanceSensor.getDistance(DistanceUnit.CM) < targetDistanceCm) {
            double currentAngle = getHeading();
            double angleError = currentAngle - startAngle;

            // Ускорение
            double speedFactor = 1.0;
            if (rampUpTime > 0) {
                long elapsedTime = System.currentTimeMillis() - startTime;
                speedFactor = Math.min(1.0, (double) elapsedTime / rampUpTime);
            }

            double adjustedSpeed = driveSpeed * speedFactor;
            double correction = angleError * kP;

            telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Speed", "%.2f", adjustedSpeed);
            telemetry.addData("Angle", "%.2f", currentAngle);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            // Прямолинейное движение с коррекцией угла
            LeftFront.setPower(adjustedSpeed + correction);
            LeftBack.setPower(adjustedSpeed + correction);
            RightFront.setPower(adjustedSpeed - correction);
            RightBack.setPower(adjustedSpeed - correction);
        }

        movestop();  // Остановка всех моторов
    }
    private void movestop() {
        /*LeftFront.setPower(0.05);
        LeftBack.setPower(0.05);
        RightFront.setPower(0.05);
        RightBack.setPower(0.05);
        sleep(200);*/
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