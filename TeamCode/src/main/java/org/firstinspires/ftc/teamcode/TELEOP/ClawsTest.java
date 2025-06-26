package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;
@Disabled
@TeleOp(name="Claws Test", group="Robot")
public class ClawsTest extends LinearOpMode {
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

    boolean aBefore = false;
    boolean bBefore = false;
    boolean xBefore = false;
    boolean yBefore = false;
    boolean b1Before = false;
    boolean a1Before = false;
    boolean x1Before = false;
    boolean y1Before = false;
    boolean lbBefore = false; //changing speed
    boolean rbBefore = false;//changing direction
    double x = 0.7;
    double speed = 0.7;
    TouchSensor touchSensor;

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
        VerClaw.setPosition(verclaw_close);

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(verrotate_player);

        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(horclaw_close);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(horrotate_middle);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        Horizontal.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        double forward;
        double rotate;
        double side;

        double horizontal;
        double vertical;

        waitForStart();

        while (opModeIsActive()) {
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = -gamepad1.left_stick_x;

            horizontal = gamepad2.right_stick_y;
            vertical = gamepad2.left_stick_y;

            LeftFront.setPower((forward + rotate + side) * speed);
            LeftBack.setPower((forward + rotate - side) * speed);
            RightFront.setPower((forward - rotate - side) * speed);
            RightBack.setPower((forward - rotate + side) * speed);

            Horizontal.setPower(horizontal * x);

            // Telemetry updates

            telemetry.addData("HorClaw Position", HorClaw.getPosition());
            telemetry.addData("HorRotate Position", HorRotate.getPosition());
            telemetry.addData("VerClaw Position", VerClaw.getPosition());
            telemetry.addData("VerRotate Position", VerRotate.getPosition());
            telemetry.update();


            if (gamepad2.y && !yBefore) {
                yBefore = true;
                HorClaw.setPosition(HorClaw.getPosition() + 0.01);
            }
                yBefore = gamepad2.y;


            if (gamepad2.a && !aBefore) {
                    aBefore = true;
                    HorClaw.setPosition(HorClaw.getPosition() - 0.01);
            }
            aBefore = gamepad2.a;

            if (gamepad2.x && !xBefore) {
                    xBefore = true;
                    HorRotate.setPosition(HorRotate.getPosition() + 0.01);
            }
            xBefore = gamepad2.x;

            if (gamepad2.b && !bBefore) {
                    bBefore = true;
                    HorRotate.setPosition(HorRotate.getPosition() - 0.01);
            }
            bBefore = gamepad2.b;

            if (gamepad1.y && !y1Before) {
                y1Before = true;
                VerClaw.setPosition(VerClaw.getPosition() + 0.01);
            }
            y1Before = gamepad1.y;


            if (gamepad1.a && !a1Before) {
                a1Before = true;
                VerClaw.setPosition(VerClaw.getPosition() - 0.01);
            }
            a1Before = gamepad1.a;

            if (gamepad1.x && !x1Before) {
                x1Before = true;
                VerRotate.setPosition(VerRotate.getPosition() + 0.01);
            }
            x1Before = gamepad1.x;

            if (gamepad1.b && !b1Before) {
                b1Before = true;
                VerRotate.setPosition(VerRotate.getPosition() - 0.01);
            }
            b1Before = gamepad1.b;

            }
        }
    }

