package org.firstinspires.ftc.teamcode.TELEOP;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@TeleOp(name="TeleOp Test", group="Robot")
public class TeleOpTest extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    double speed = 0.7;

    @Override
    public void runOpMode() {
        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        double forward;
        double rotate;
        double side;

        waitForStart();

        while (opModeIsActive())   {
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = -gamepad1.left_stick_x;

            LeftFront.setPower((forward + rotate + side)*speed);
            LeftBack.setPower((forward + rotate - side)*speed);
            RightFront.setPower((forward - rotate - side)*speed);
            RightBack.setPower((forward - rotate + side)*speed);
        }
    }
}

