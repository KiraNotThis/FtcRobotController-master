package org.firstinspires.ftc.teamcode.TELEOP;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOp Basket", group = "Robot")
public class TeleOpBasket extends LinearOpMode {
    private DcMotor LeftFront, LeftBack, RightFront, RightBack;
    private DcMotor Horizontal, Vertical;
    private Servo VerClaw, VerRotate, HorClaw, HorRotate;
    private TouchSensor touchSensor;

    boolean aBefore = false, bBefore = false, xBefore = false, yBefore = false;
    boolean b1Before = false, lbBefore = false, rbBefore = false, specialSeqActive = false;

    double x = 0.7;
    double speed = 0.8;
    boolean horClawClosed = false;
    boolean verClawClosed = false;
    boolean verRotated = false;
    boolean horRotated = false;

    long sequenceStartTime = 0;

    enum SpecialSeqState { IDLE, STEP1, STEP2, STEP3, STEP4 }
    SpecialSeqState seqState = SpecialSeqState.IDLE;

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
        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");

        VerClaw.setPosition(verclaw_open);
        VerRotate.setPosition(verrotate_player);
        HorClaw.setPosition(horclaw_open);
        HorRotate.setPosition(horrotate_middle);

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        Horizontal.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // DRIVING
            double forward = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            double side = -gamepad1.left_stick_x;
            double horizontal = gamepad2.right_stick_y;

            LeftFront.setPower((forward - rotate + side) * speed);
            LeftBack.setPower((forward - rotate - side) * speed);
            RightFront.setPower((forward + rotate - side) * speed);
            RightBack.setPower((forward + rotate + side) * speed);

            Horizontal.setPower(horizontal * x);

            if (gamepad1.right_bumper && !rbBefore) speed = -speed;
            rbBefore = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lbBefore) speed = (speed == 0.4) ? 0.8 : 0.4;
            lbBefore = gamepad1.left_bumper;

            if (gamepad1.b && !b1Before) x = (x == 0.3) ? 0.7 : 0.3;
            b1Before = gamepad1.b;

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

            // MAGIC BUTTON ZERO
            if (gamepad2.left_bumper) {
                VerClaw.setPosition(verclaw_open);
                VerRotate.setPosition(verrotate_chamber);
                Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (!touchSensor.isPressed()) {
                    Vertical.setPower(1);
                } else {
                    Vertical.setPower(0);
                    Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            // MAGIC BUTTON
            if (gamepad2.right_bumper && !specialSeqActive) {
                specialSeqActive = true;
                seqState = SpecialSeqState.STEP1;
                sequenceStartTime = System.currentTimeMillis();
            }

            if (specialSeqActive) {
                long elapsed = System.currentTimeMillis() - sequenceStartTime;
                switch (seqState) {
                    case STEP1:
                        VerClaw.setPosition(verclaw_close);
                        if (elapsed > 500) {
                            seqState = SpecialSeqState.STEP2;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case STEP2:
                        HorClaw.setPosition(horclaw_open);
                        if (elapsed > 500) {
                            seqState = SpecialSeqState.STEP3;
                            sequenceStartTime = System.currentTimeMillis();
                        }
                        break;
                    case STEP3:
                        VerRotate.setPosition(verrotate_player);
                        if (elapsed > 500) {
                            seqState = SpecialSeqState.STEP4;
                            Vertical.setTargetPosition(high_basket);
                            Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Vertical.setPower(-0.8);
                            specialSeqActive = false;
                            seqState = SpecialSeqState.IDLE;
                        }
                        break;
                }
            }

            // CLAWS
            if (gamepad2.y && !yBefore) {
                verClawClosed = !verClawClosed;
                VerClaw.setPosition(verClawClosed ? verclaw_close : verclaw_open);
            }
            yBefore = gamepad2.y;

            if (gamepad2.a && !aBefore) {
                verRotated = !verRotated;
                VerRotate.setPosition(verRotated ? verrotate_chamber : verrotate_player);
            }
            aBefore = gamepad2.a;

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

            // ===== Телеметрия =====
            telemetry.addData("Vertical Motor Position", Vertical.getCurrentPosition());
            telemetry.update();
        }
    }
}
