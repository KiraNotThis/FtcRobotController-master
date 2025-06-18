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

@Autonomous(name = "PID Calibration", group = "Calibration")
public class AutoPIDCalibration extends LinearOpMode {

    // PID коэффициенты для калибровки
    double kP = 0.05;
    double kI = 0.0;
    double kD = 0.0;

    double distanceCM = 50;  // расстояние для теста
    double speed = 0.5;      // скорость движения

    boolean isMoving = false; // флаг, чтобы не менять коэффициенты во время движения

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");

        // Настройка ориентации робота
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);
        imu.resetYaw();

        // Инициализация моторов и серв
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

        while (!isStarted()) {
            telemetry.addData("Waiting to start", "Use DPAD and bumpers to adjust PID");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("Press A to run test", "");
            telemetry.update();

            // Обновление коэффициентов до старта
            adjustPIDWithGamepad();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (!isMoving) {
                adjustPIDWithGamepad();

                if (gamepad1.a) {
                    isMoving = true;
                    double startAngle = getHeading();
                    driveStraightPID(speed, distanceCM, startAngle, kP, kI, kD);
                    isMoving = false;
                }
            }

            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("Press A to start move", "");
            telemetry.update();
        }
    }

    private void adjustPIDWithGamepad() {
        // Плавное изменение коэффициентов с задержкой для удобства
        if (gamepad1.dpad_up) {
            kP += 0.001;
            sleep(150);
        } else if (gamepad1.dpad_down) {
            kP -= 0.001;
            if (kP < 0) kP = 0;
            sleep(150);
        }

        if (gamepad1.dpad_right) {
            kI += 0.0005;
            sleep(150);
        } else if (gamepad1.dpad_left) {
            kI -= 0.0005;
            if (kI < 0) kI = 0;
            sleep(150);
        }

        if (gamepad1.right_bumper) {
            kD += 0.001;
            sleep(150);
        } else if (gamepad1.left_bumper) {
            kD -= 0.001;
            if (kD < 0) kD = 0;
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

    /**
     * Движение вперед с PID контролем угла
     *
     * @param driveSpeed  скорость (0.0 - 1.0)
     * @param distanceCM  расстояние в сантиметрах
     * @param targetAngle целевой угол
     * @param kP          коэффициент пропорциональности
     * @param kI          коэффициент интеграла
     * @param kD          коэффициент дифференциала
     */
    public void driveStraightPID(double driveSpeed, double distanceCM, double targetAngle,
                                 double kP, double kI, double kD) {

        encoders();

        int targetTicks = (int) (PULSES_PER_CM * distanceCM);

        double integral = 0;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {

            double currentAngle = getHeading();
            double error = targetAngle - currentAngle;
            error = (error + 180) % 360 - 180;

            long now = System.currentTimeMillis();
            double deltaTime = (now - lastTime) / 1000.0;
            if (deltaTime < 1e-6) deltaTime = 1e-6;

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            double correction = kP * error + kI * integral + kD * derivative;

            double leftPower = driveSpeed + correction;
            double rightPower = driveSpeed - correction;

            leftPower = Math.max(-1, Math.min(1, leftPower));
            rightPower = Math.max(-1, Math.min(1, rightPower));

            LeftFront.setPower(leftPower);
            LeftBack.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightBack.setPower(rightPower);

            telemetry.addData("Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Encoder Position", LeftFront.getCurrentPosition());
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
