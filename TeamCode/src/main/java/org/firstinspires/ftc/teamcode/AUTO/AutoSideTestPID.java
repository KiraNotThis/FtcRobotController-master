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

@Autonomous(name = "Side Test PID", group = "Robot")
public class AutoSideTestPID extends LinearOpMode {

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

    //______________________________*Side_________________________________//

    /**
     * Movement with PID and angle
     *
     * @param driveSpeed  speed (0.0 - 1.0)
     * @param distanceCM  distance in cm
     * @param targetAngle starting angle
     * @param kP          proportional coefficient
     * @param kI          integral coefficient
     * @param kD          differential coefficient
     */
    public void driveSide(double driveSpeed, double distanceCM, double targetAngle,
                              double kP, double kI, double kD) {

        encoders(); // stop and reset encoders
        int targetTicks = (int) (PULSES_PER_CM * distanceCM); // getting target ticks

        double integral = 0;// we don't have a mistake in the start
        double lastError = 0;// we don't have last error -- that's why it is 0
        long lastTime = System.currentTimeMillis(); // to get current time in ms

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < targetTicks) {//if we still need to move
            double currentAngle = getHeading();//taking a current angle of the robot
            double error = targetAngle - currentAngle; //error's calculating the between "ideal" angle and the current

            //doing error from [-180; 180]
            error = (error + 180) % 360 - 180;

            long now = System.currentTimeMillis(); // getting current time
            double deltaTime = (now - lastTime) / 1000.0; // doing the time in seconds not in ms

            integral += error * deltaTime; //adding our error on time for integral
            double derivative = (error - lastError) / deltaTime; // calculating how fast the error is changing

            double correction = kP * error + kI * integral + kD * derivative; //calculating a correction based on coefficients



            double leftFrontPower = driveSpeed + correction;
            double leftBackPower = -(driveSpeed + correction);
            double rightFrontPower = -(driveSpeed - correction);
            double rightBackPower = driveSpeed - correction;

            //checking and аixing if power is out of range(<-1 or >1)
            leftFrontPower = Math.max(-1.0, Math.min(1.0, leftFrontPower));
            leftBackPower = Math.max(-1.0, Math.min(1.0, leftBackPower));
            rightFrontPower = Math.max(-1.0, Math.min(1.0, rightFrontPower));
            rightBackPower = Math.max(-1.0, Math.min(1.0, rightBackPower));


            LeftFront.setPower(leftFrontPower);
            LeftBack.setPower(leftBackPower);
            RightFront.setPower(rightFrontPower);
            RightBack.setPower(rightBackPower);


            telemetry.addData("Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            lastError = error;//remaking lasterror for our current error
            lastTime = now;// remaking lasttime for our current time
        }

        movestop();
    }
    //______________________________Side*_________________________________//


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