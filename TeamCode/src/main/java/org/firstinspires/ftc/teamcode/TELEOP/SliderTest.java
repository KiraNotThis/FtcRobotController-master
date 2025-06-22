package org.firstinspires.ftc.teamcode.TELEOP;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;
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
        VerClaw.setPosition(verclaw_close);

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(verrotate_chamber);

        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(horclaw_open);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.08);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        Horizontal.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            Vertical.setPower(vertical * x);



            // Update Vertical motor target positions based on dpad inputs

            if(gamepad2.left_bumper){//Slider in the Zero position
                while (opModeIsActive() && !touchSensor.isPressed()) {
                    Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Vertical.setPower(1);  // Keep moving down
                }
                Vertical.setPower(0);
                Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.addData("Vertical Motor Position", Vertical.getCurrentPosition());
            telemetry.addData("Horizontal Motor Position", Horizontal.getCurrentPosition());
            telemetry.update();


            // Telemetry for Vertical motor position
            /*if (touchVertical.isPressed()) {
                telemetry.addData("Touch Sensor", "Pressed");
                telemetry.update();
            }
            else {
                telemetry.addData("Vertical", Vertical.getCurrentPosition());
                telemetry.update();
            }*/
        }
    }
}
