package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "GamePad")
public class GamePad extends OpMode {

    private DcMotor MotorTR;
    private DcMotor MotorTL;
    private DcMotor MotorBR;
    private DcMotor MotorBL;

    private DistanceSensor distanceSensor;

    @Override
    public void init() {

        // Motors
        MotorTR = hardwareMap.get(DcMotor.class, "front right");
        MotorTL = hardwareMap.get(DcMotor.class, "front left");
        MotorBR = hardwareMap.get(DcMotor.class, "back right");
        MotorBL = hardwareMap.get(DcMotor.class, "back left");

        // Reverse right side motors
        MotorTR.setDirection(DcMotor.Direction.REVERSE);
        MotorBR.setDirection(DcMotor.Direction.REVERSE);

        MotorTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Distance Sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Controller inputs
        double forward = gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        // Mecanum calculations
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
        double speedMultiplier = 0.5;

        if (gamepad1.left_bumper) {
            speedMultiplier = 1.0;
        } else if (gamepad1.right_bumper) {
            speedMultiplier = 0.25;
        }

// Apply speed multiplier
        frontLeftPower *= speedMultiplier;
        backLeftPower *= speedMultiplier;
        frontRightPower *= speedMultiplier;
        backRightPower *= speedMultiplier;


        // Drive motors
        MotorTL.setPower(frontLeftPower);
        MotorTR.setPower(frontRightPower);
        MotorBL.setPower(backLeftPower);
        MotorBR.setPower(backRightPower);

        // Read distance
        double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);

        // Telemetry
        telemetry.addData("Distance (cm)", "%.1f", distanceCM);

        if (distanceCM < 10) {
            telemetry.addLine("⚠ Object Very Close!");
        }

        telemetry.update();
    }
}