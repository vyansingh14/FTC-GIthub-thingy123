package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "robotAutonomousCode", group = "Autonomous")
public class robotAutonomousCode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Mechanism used to shoot (swap in your shooter motor if you have a separate one)
    private DcMotor intake;

    private final ElapsedTime runtime = new ElapsedTime();

    // ---- Tuning values ----
    private static final double DRIVE_POWER = 1.0;
    private static final double RETURN_POWER = 1.0;
    private static final double DRIVE_FORWARD_SECONDS = 1.0;
    private static final double SHOOT_WAIT_SECONDS = 4.0;
    private static final double SHOOT_POWER = 1.0;
    private static final double RETURN_TIMEOUT_SECONDS = 6.0;

    @Override
    public void runOpMode() {

        // Hardware init
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Left side reversed (same as the version where driving worked)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Hold position when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        resetDriveEncoders();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // Remember where we started so we can return to exactly this spot
        int flStart = frontLeft.getCurrentPosition();
        int frStart = frontRight.getCurrentPosition();
        int blStart = backLeft.getCurrentPosition();
        int brStart = backRight.getCurrentPosition();

        // 1) Drive forward for about one second
        telemetry.addData("Step", "Driving forward");
        telemetry.update();

        setDrivePower(DRIVE_POWER);
        waitSeconds(DRIVE_FORWARD_SECONDS);
        stopDrive();

        // 2) Wait 4 seconds while shooting
        if (opModeIsActive()) {
            telemetry.addData("Step", "Shooting");
            telemetry.update();

            intake.setPower(SHOOT_POWER);
            waitSeconds(SHOOT_WAIT_SECONDS);
            intake.setPower(0);
        }

        // 3) Return to the starting position using encoders
        if (opModeIsActive()) {
            telemetry.addData("Step", "Returning");
            telemetry.update();

            returnToPosition(flStart, frStart, blStart, brStart);
        }

        stopAllMotors();

        telemetry.addData("Step", "Done");
        telemetry.update();
    }

    private void returnToPosition(int flTarget, int frTarget, int blTarget, int brTarget) {

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // In RUN_TO_POSITION, power is always positive; the target sets the direction
        setDrivePower(RETURN_POWER);

        runtime.reset();
        while (opModeIsActive()
                && runtime.seconds() < RETURN_TIMEOUT_SECONDS
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {

            telemetry.addData("FL", "%d -> %d", frontLeft.getCurrentPosition(), flTarget);
            telemetry.addData("FR", "%d -> %d", frontRight.getCurrentPosition(), frTarget);
            telemetry.addData("BL", "%d -> %d", backLeft.getCurrentPosition(), blTarget);
            telemetry.addData("BR", "%d -> %d", backRight.getCurrentPosition(), brTarget);
            telemetry.update();
        }

        stopDrive();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        setDrivePower(0);
    }

    private void stopAllMotors() {
        stopDrive();
        intake.setPower(0);
    }

    private void waitSeconds(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            idle();
        }
    }
}
