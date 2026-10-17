package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "robotAutonomousCode")
public class robotAutonomousCode extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor intake;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double DRIVE_POWER = 0.5;
    private static final double INTAKE_POWER = 1.0;

    private static final double DRIVE_FORWARD_SECONDS = 1.5;
    private static final double INTAKE_SECONDS = 3.0;

    @Override
    public void runOpMode() {

        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_motor");
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_motor");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_motor");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_motor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        telemetry.addLine("Autonomous initialized");
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Step", "Driving forward");
            telemetry.update();

            driveMecanum(DRIVE_POWER, 0, 0);
            waitSeconds(DRIVE_FORWARD_SECONDS);
            stopDrive();
        }

        if (opModeIsActive()) {
            telemetry.addData("Step", "Running intake");
            telemetry.update();

            intake.setPower(INTAKE_POWER);
            waitSeconds(INTAKE_SECONDS);
            intake.setPower(0);
        }

        stopDrive();
        intake.setPower(0);

        telemetry.addData("Step", "Done");
        telemetry.update();
    }

    private void driveMecanum(double y, double x, double rotation) {

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rotation),
                1.0);

        frontLeft.setPower((y + x + rotation) / denominator);
        backLeft.setPower((y - x + rotation) / denominator);
        frontRight.setPower((y - x - rotation) / denominator);
        backRight.setPower((y + x - rotation) / denominator);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void waitSeconds(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            idle();
        }
    }
}
