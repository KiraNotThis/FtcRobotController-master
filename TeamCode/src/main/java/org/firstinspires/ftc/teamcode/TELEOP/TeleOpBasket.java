package org.firstinspires.ftc.teamcode.TELEOP;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.HorRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerClaw;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.VerRotate;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.high_basket;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.high_chamber;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horclaw_close;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horclaw_open;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_ground;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_middle;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.horrotate_transfer;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.low_basket;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.middle_chamber;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verclaw_close;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verclaw_open;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verrotate_chamber;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.verrotate_player;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOp Basket", group = "Robot")
public class TeleOpBasket extends LinearOpMode {
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

    // Кнопки
    boolean aBefore = false;
    boolean bBefore = false;
    boolean xBefore = false;
    boolean yBefore = false;
    boolean b1Before = false;
    boolean lbBefore = false;
    boolean rbBefore = false;

    // Логика
    double x = 0.7;
    double speed = 0.8;
    int positionState = 0;

    // Флаги состояния
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

            // Движение робота
            LeftFront.setPower((forward - rotate + side) * speed);
            LeftBack.setPower((forward - rotate - side) * speed);
            RightFront.setPower((forward + rotate - side) * speed);
            RightBack.setPower((forward + rotate + side) * speed);

            Horizontal.setPower(horizontal * x);

            // Переключение направления
            if (gamepad1.right_bumper && !rbBefore) {
                speed = -speed;
            }
            rbBefore = gamepad1.right_bumper;

            // Переключение скорости
            if (gamepad1.left_bumper && !lbBefore) {
                speed = (speed == 0.4) ? 0.8 : 0.4;
            }
            lbBefore = gamepad1.left_bumper;

            // Переключение скорости слайдера
            if (gamepad1.b && !b1Before) {
                x = (x == 0.3) ? 0.7 : 0.3;
            }
            b1Before = gamepad1.b;

            // Подъём вертикального слайдера
            if (gamepad2.dpad_right) {
                Vertical.setTargetPosition(low_basket);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.8);
            }

            if (gamepad2.dpad_up) {
                Vertical.setTargetPosition(high_basket);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.8);
            }

            // Обнуление вертикального слайдера
            if (gamepad2.left_bumper) {
                VerClaw.setPosition(verclaw_open);
                sleep(500);
                VerRotate.setPosition(verrotate_chamber);
                sleep(500);
                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(1);
                }
                Vertical.setPower(0);
                Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Специальная последовательность
            if (gamepad2.right_bumper) {
                /*VerClaw.setPosition(verclaw_open);
                sleep(500);
                HorRotate.setPosition(horrotate_transfer);
                sleep(500);*/
                VerClaw.setPosition(verclaw_close);
                sleep(500);
                HorClaw.setPosition(horclaw_open);
                sleep(500);
                VerRotate.setPosition(verrotate_player);
                sleep(500);
                Vertical.setTargetPosition(high_basket);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.8);
            }

            // Управление вертикальной клешнёй
            if (gamepad2.y && !yBefore) {
                verClawClosed = !verClawClosed;
                VerClaw.setPosition(verClawClosed ? verclaw_close : verclaw_open);
            }
            yBefore = gamepad2.y;

            // Управление вращением вертикальной клешни
            if (gamepad2.a && !aBefore) {
                verRotated = !verRotated;
                VerRotate.setPosition(verRotated ? verrotate_chamber : verrotate_player);
            }
            aBefore = gamepad2.a;

            // Управление горизонтальной клешнёй
            if (gamepad2.x && !xBefore) {
                horClawClosed = !horClawClosed;
                HorClaw.setPosition(horClawClosed ? horclaw_close : horclaw_open);
            }
            xBefore = gamepad2.x;

            if (gamepad2.b && !bBefore) {
                horRotated = !horRotated;
                HorRotate.setPosition(horRotated ? horrotate_ground : horrotate_transfer);
            }
            bBefore = gamepad2.b;

            // Управление положением горизонтальной клешни
            /*if (gamepad2.b && !bBefore) {
                positionState = (positionState + 1) % 3;
                switch (positionState) {
                    case 0:
                        HorRotate.setPosition(horrotate_ground);
                        break;
                    case 1:
                        HorRotate.setPosition(horrotate_middle);
                        break;
                    case 2:
                        HorRotate.setPosition(horrotate_lying);
                        break;
                }
            }
            bBefore = gamepad2.b*/

            telemetry.addData("Vertical Motor Position", Vertical.getCurrentPosition());
            telemetry.update();
        }
    }
}
