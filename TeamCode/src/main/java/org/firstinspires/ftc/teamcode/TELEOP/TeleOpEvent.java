package org.firstinspires.ftc.teamcode.TELEOP;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="TeleOp Event", group="Robot")
public class TeleOpEvent extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    int positionState = 0; // обьяви вверху, например в теле класса




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
        VerClaw.setPosition(0.51);


        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.78);


        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.5);


        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.08);


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
        double vertical;


        waitForStart();


        while (opModeIsActive())   {


            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = -gamepad1.left_stick_x;


            horizontal = gamepad2.right_stick_y;


            LeftFront.setPower((forward - rotate + side)*0.5);
            LeftBack.setPower((forward - rotate - side)*0.5);
            RightFront.setPower((forward + rotate - side)*0.5);
            RightBack.setPower((forward + rotate + side)*0.5);


            Horizontal.setPower(horizontal * 0.5);


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
                VerClaw.setPosition(0.4);//open
                sleep(500);
                VerClaw.setPosition(0.6);//close
                sleep(500);
                VerRotate.setPosition(0.78);//rotate
                sleep(600);
                VerClaw.setPosition(0.4);


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
                if(VerClaw.getPosition() == 0.52) {
                    VerClaw.setPosition(0.25);
                }
                else{
                    VerClaw.setPosition(0.52);
                }
            }
            yBefore = gamepad2.y;


            if(gamepad2.a && !aBefore){//Control Vertical Rotate
                aBefore = true;
                if(VerRotate.getPosition() == 0.78) {
                    VerRotate.setPosition(0.12);
                }
                else{
                    VerRotate.setPosition(0.78);
                }
            }
            aBefore = gamepad2.a;


            if(gamepad2.x && !xBefore){//Control Horizontal Claw
                xBefore = true;
                if(HorClaw.getPosition() == 0.3) {
                    HorClaw.setPosition(0.5);
                }
                else{
                    HorClaw.setPosition(0.3);
                }
            }
            xBefore = gamepad2.x;






            if(gamepad2.b && !bBefore){//Control Horizontal Rotate
                bBefore = true;


                positionState = (positionState + 1) % 3; // циклическое переключение от 0 до 2


                switch (positionState) {
                    case 0:
                        HorRotate.setPosition(1);
                        break;
                    case 1:
                        HorRotate.setPosition(0.55);
                        break;
                    case 2:
                        HorRotate.setPosition(0.08);
                        break;
                }
            }
            bBefore = gamepad2.b;
        }


    }
}

