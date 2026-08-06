package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "GamePad")
public class GamePad extends OpMode {

    private DcMotor MotorTR;
    private DcMotor MotorTL;
    private DcMotor MotorBR;
    private DcMotor MotorBL;

    @Override
    public void init() {

        MotorTR = hardwareMap.get(DcMotor.class, "front right");
        MotorTL = hardwareMap.get(DcMotor.class, "front left");
        MotorBR = hardwareMap.get(DcMotor.class, "back right");
        MotorBL = hardwareMap.get(DcMotor.class, "back left");

        // Reverse the right side
        MotorTR.setDirection(DcMotor.Direction.REVERSE);
        MotorBR.setDirection(DcMotor.Direction.REVERSE);

        MotorTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorSpeed(double speed) {
        MotorTR.setPower(speed);
        MotorTL.setPower(speed);
        MotorBR.setPower(speed);
        MotorBL.setPower(speed);
    }

    @Override
    public void loop() {

        // Controller inputs
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Mecanum drive calculations
        double frontLeftPower  = forward + strafe + rotate;
        double backLeftPower   = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower  = forward + strafe - rotate;

        // Normalize motor powers
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        // Set motor powers
        MotorTL.setPower(frontLeftPower);
        MotorTR.setPower(frontRightPower);
        MotorBL.setPower(backLeftPower);
        MotorBR.setPower(backRightPower);

        // Telemetry
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);
        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);
        telemetry.update();
    }
}