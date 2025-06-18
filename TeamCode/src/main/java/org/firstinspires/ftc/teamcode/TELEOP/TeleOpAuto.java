package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@TeleOp(name="TeleOp Auto", group="Robot")
public class TeleOpAuto extends LinearOpMode {
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
        VerClaw.setPosition(0.6);

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.88);

        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.52);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.35);

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

        while (opModeIsActive())   {

            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = -gamepad1.left_stick_x;

            horizontal = gamepad2.right_stick_y;

            LeftFront.setPower((forward + rotate + side)*speed);
            LeftBack.setPower((forward + rotate - side)*speed);
            RightFront.setPower((forward - rotate - side)*speed);
            RightBack.setPower((forward - rotate + side)*speed);

            Horizontal.setPower(horizontal * x);

            if(gamepad1.right_bumper && !rbBefore){//change direction of the robot
                rbBefore = true;
                speed = -speed;
            }
            rbBefore = gamepad1.right_bumper;

            if(gamepad1.left_bumper && !lbBefore){//change speed of the robot
                lbBefore = true;
                if (speed == 0.3){
                    speed = 0.7;
                }
                else{
                    speed = 0.3;
                }
            }
            lbBefore = gamepad1.left_bumper;

            if (gamepad1.b && !b1Before){//change speed of the horizontal slider
                if (x == 0.3){
                    x = 0.7;
                }
                else{
                    x = 0.3;
                }
            }
            b1Before = gamepad1.b;

            if(gamepad2.dpad_right) {//middle of chambers
                Vertical.setTargetPosition(-1600);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if(gamepad2.dpad_up) {//high chamber
                Vertical.setTargetPosition(-2600);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if(gamepad2.dpad_left) {//high basket
                Vertical.setTargetPosition(-4100);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);
            }

            if(gamepad2.dpad_down) {//low basket
                Vertical.setTargetPosition(-1000);
                Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Vertical.setPower(-0.6);

            }

            if(gamepad2.left_bumper){//Slider in the Zero position
                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(1);  // Keep moving down
                }
                Vertical.setPower(0);
                Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.addData("Vertical Motor Position", Vertical.getCurrentPosition());
            telemetry.update();

            if(gamepad2.right_bumper) {//Preparing for taking specimen from observation zone
                VerClaw.setPosition(0.3);//open
                sleep(500);
                VerClaw.setPosition(0.6);//close
                sleep(500);
                VerRotate.setPosition(0.88);//rotate
                sleep(600);
                VerClaw.setPosition(0.3);

                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(1);  // Keep moving down
                }

                Vertical.setPower(0);

                Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LeftFront.setPower(0.8);
                LeftBack.setPower(0.8);
                RightFront.setPower(0.8);
                RightBack.setPower(0.8);
                sleep(650);

                LeftFront.setPower(-0.8);
                LeftBack.setPower(0.8);
                RightFront.setPower(0.8);
                RightBack.setPower(-0.8);
                sleep(870);
            }

            if(gamepad2.y && !yBefore){//Control Vertical Claw
                yBefore = true;
                if(VerClaw.getPosition() == 0.6) {
                    VerClaw.setPosition(0.3);
                }
                else{
                    VerClaw.setPosition(0.6);
                }
            }
            yBefore = gamepad2.y;

            if(gamepad2.a && !aBefore){//Control Vertical Rotate
                aBefore = true;
                if(VerRotate.getPosition() == 0.88) {
                    VerRotate.setPosition(0.11);
                }
                else{
                    VerRotate.setPosition(0.88);
                }
            }
            aBefore = gamepad2.a;

            if(gamepad2.x && !xBefore){//Control Horizontal Claw
                xBefore = true;
                if(HorClaw.getPosition() == 0.27) {
                    HorClaw.setPosition(0.52);
                }
                else{
                    HorClaw.setPosition(0.27);
                }
            }
            xBefore = gamepad2.x;

            if(gamepad2.b && !bBefore){//Control Horizontal Rotate
                bBefore = true;
                if(HorRotate.getPosition() == 0.5) {
                    HorRotate.setPosition(0.94);
                }
                else{
                    HorRotate.setPosition(0.5);
                }
            }
            bBefore = gamepad2.b;
        }

    }
}