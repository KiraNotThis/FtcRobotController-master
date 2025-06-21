package org.firstinspires.ftc.teamcode.TELEOP;

import static org.firstinspires.ftc.teamcode.AUTO.Globals.Horizontal;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.touchHorizontal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="SliderTest", group="Robot")
public class SliderTest extends LinearOpMode {
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
    boolean lbBefore = false; // changing speed
    boolean rbBefore = false; // changing direction
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
        touchHorizontal = hardwareMap.get(TouchSensor.class, "sensor_touch_hor");
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        VerClaw.setPosition(0.4);

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.17);

        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.2);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.45);

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



            // Update Vertical motor target positions based on dpad inputs
            if (gamepad2.dpad_down) { // middle of chambers
                Vertical.setTargetPosition(-1700);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if (gamepad2.dpad_right) { // high chamber
                Vertical.setTargetPosition(-3000);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if (gamepad2.dpad_up) { // high basket
                Vertical.setTargetPosition(-4100);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if (gamepad2.dpad_left) { // low basket
                Vertical.setTargetPosition(-1500);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if (gamepad2.left_bumper) {
                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(0.6); // Keep moving down
                }
                Vertical.setPower(0);
            }

            if (gamepad2.right_bumper) {
                VerClaw.setPosition(0.1); // open
                sleep(400);
                VerClaw.setPosition(0.32); // close
                sleep(400);
                VerRotate.setPosition(0.81); // rotate
                sleep(600);
                VerClaw.setPosition(0.1);
                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(0.6); // Keep moving down
                }
                Vertical.setPower(0);
            }



            // Telemetry for Vertical motor position
            if (touchHorizontal.isPressed()) {
                telemetry.addData("Touch Sensor", "Pressed");
                telemetry.update();
            }
            else {
                telemetry.addData("Touch Sensor", " NOT Pressed");
                telemetry.update();
            }
        }
    }
}
