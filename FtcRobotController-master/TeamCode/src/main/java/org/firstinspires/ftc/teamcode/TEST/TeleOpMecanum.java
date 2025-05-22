package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp Mecanum", group="Robot")
@Disabled
public class TeleOpMecanum extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    @Override
    public void runOpMode() {
        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        double forward;
        double rotate;
        double side;

        waitForStart();

        while (opModeIsActive())   {

            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            side = gamepad1.left_stick_x;

            LeftFront.setPower(forward + rotate + side);
            LeftBack.setPower(forward + rotate - side);
            RightFront.setPower(forward - rotate - side);
            RightBack.setPower(forward - rotate + side);


        }

    }
}