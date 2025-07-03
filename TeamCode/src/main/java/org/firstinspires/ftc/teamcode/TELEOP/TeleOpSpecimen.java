package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;


@TeleOp(name = "TeleOp Specimen", group = "Robot")
public class TeleOpSpecimen extends LinearOpMode {

    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private DcMotor Horizontal = null;
    private DcMotor Vertical = null;

    private Servo VerClaw;
    private Servo VerRotate;
    private Servo HorClaw;
    private Servo HorRotate;

    private TouchSensor touchSensor;

    // BUTTONS
    boolean aBefore = false;
    boolean bBefore = false;
    boolean xBefore = false;
    boolean yBefore = false;
    boolean b1Before = false;
    boolean lbBefore = false;
    boolean rbBefore = false;

    // SPEED
    double x = 0.7;
    double speed = 0.8;
    int positionState = 0;

    // FLAGS
    boolean horClawClosed = false;
    boolean verClawClosed = false;
    boolean verRotated = false;
    boolean horRotated = false;

    @Override
    public void runOpMode() {
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        Horizontal = hardwareMap.get(DcMotor.class, "Horizontal");
        Vertical = hardwareMap.get(DcMotor.class, "Vertical");

        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        VerClaw.setPosition(verclaw_open);

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(verrotate_player);

        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(horclaw_open);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(horrotate_middle);

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        Horizontal.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        double forward;
        double rotate;
        double side;

        double horizontal;

        waitForStart();

        while (opModeIsActive()) {
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = -gamepad1.left_stick_x;
            horizontal = gamepad2.right_stick_y;

            // DRIVING
            LeftFront.setPower((forward - rotate + side) * speed);
            LeftBack.setPower((forward - rotate - side) * speed);
            RightFront.setPower((forward + rotate - side) * speed);
            RightBack.setPower((forward + rotate + side) * speed);

            // HORIZONTAL SLIDER
            Horizontal.setPower(horizontal * x);

            // CHANGING DIRECTION
            if (gamepad1.right_bumper && !rbBefore) {
                speed = -speed;
            }
            rbBefore = gamepad1.right_bumper;

            // CHANGING SPEED
            if (gamepad1.left_bumper && !lbBefore) {
                speed = (speed == 0.4) ? 0.8 : 0.4;
            }
            lbBefore = gamepad1.left_bumper;

            // CHANGING SPEED OF SLIDER
            if (gamepad1.b && !b1Before) {
                x = (x == 0.3) ? 0.7 : 0.3;
            }
            b1Before = gamepad1.b;

            // VERTICAL : MIDDLE CHAMBER
            if (gamepad2.dpad_right) {
                Vertical.setTargetPosition(middle_chamber);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.8);
            }

            // VERTICAL : HIGH CHAMBER
            if (gamepad2.dpad_up) {
                Vertical.setTargetPosition(high_chamber);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.8);
            }

            // VERTICAL : ZERO POSITION
            if (gamepad2.left_bumper) {
                Thread resetThread = new Thread(() -> {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    while (opModeIsActive() && !touchSensor.isPressed()) {
                        Vertical.setPower(1);
                    }
                    Vertical.setPower(0);
                    Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                });
                resetThread.start();
            }

            // MAGIC BUTTON
            if (gamepad2.right_bumper) {
                Thread sequenceThread = new Thread(() -> {
                    VerClaw.setPosition(verclaw_open);
                    sleep(500);
                    VerRotate.setPosition(verrotate_player);
                    sleep(600);

                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    while (opModeIsActive() && !touchSensor.isPressed()) {
                        Vertical.setPower(1);
                    }

                    Vertical.setPower(0);
                    Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    // Сброс энкодеров шасси
                    /*LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    // Включение в RUN_USING_ENCODER
                    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Движение вперёд
                    LeftFront.setPower(0.8);
                    LeftBack.setPower(0.8);
                    RightFront.setPower(0.8);
                    RightBack.setPower(0.8);
                    sleep(650);

                    // Разворот
                    LeftFront.setPower(-0.8);
                    LeftBack.setPower(0.8);
                    RightFront.setPower(0.8);
                    RightBack.setPower(-0.8);
                    sleep(870);

                    // Стоп
                    LeftFront.setPower(0);
                    LeftBack.setPower(0);
                    RightFront.setPower(0);
                    RightBack.setPower(0);*/
                });
                sequenceThread.start();
            }

            // VERTICAL CLAW
            if (gamepad2.y && !yBefore) {
                verClawClosed = !verClawClosed;
                VerClaw.setPosition(verClawClosed ? verclaw_close : verclaw_open);
            }
            yBefore = gamepad2.y;

            // VERTICAL ROTATE
            if (gamepad2.a && !aBefore) {
                verRotated = !verRotated;
                VerRotate.setPosition(verRotated ? verrotate_chamber : verrotate_player);
            }
            aBefore = gamepad2.a;

            // HORIZONTAL CLAW
            if (gamepad2.x && !xBefore) {
                horClawClosed = !horClawClosed;
                HorClaw.setPosition(horClawClosed ? horclaw_close : horclaw_open);
            }
            xBefore = gamepad2.x;

            // HORIZONTAL ROTATE
            if (gamepad2.b && !bBefore) {
                horRotated = !horRotated;
                HorRotate.setPosition(horRotated ? horrotate_ground : horrotate_middle);
            }
            bBefore = gamepad2.b;

            telemetry.addData("Vertical Motor Position", Vertical.getCurrentPosition());
            telemetry.update();
        }
    }
}
