
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

    // Launcher motor
    private DcMotor launcherMotor;

    private final ElapsedTime runtime = new ElapsedTime();

    // Tuning values
    private static final double DRIVE_POWER = 0.3;
    private static final double RETURN_POWER = 0.3;
    private static final double DRIVE_FORWARD_SECONDS = 1.0;
    private static final double SHOOT_WAIT_SECONDS = 4.0;
    private static final double SHOOT_POWER = 1.0;
    private static final double RETURN_TIMEOUT_SECONDS = 6.0;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_motor");

        // Initialize launcher motor
        launcherMotor = hardwareMap.get(
                DcMotor.class, "launcherMotor");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake drive motors when stopped
        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure launcher
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setPower(0);

        // Reset drive encoders
        resetDriveEncoders();

        telemetry.addLine("Robot Ready");
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopAllMotors();
            return;
        }

        // Save the starting encoder positions
        int flStart = frontLeft.getCurrentPosition();
        int frStart = frontRight.getCurrentPosition();
        int blStart = backLeft.getCurrentPosition();
        int brStart = backRight.getCurrentPosition();

        // STEP 1: Drive forward
        telemetry.addData("Step", "Driving Forward");
        telemetry.update();

        setDrivePower(DRIVE_POWER);
        waitSeconds(DRIVE_FORWARD_SECONDS);
        stopDrive();

        // STEP 2: Run launcher for 4 seconds
        if (opModeIsActive()) {
            telemetry.addData("Step", "Launching");
            telemetry.update();

            launcherMotor.setPower(SHOOT_POWER);
            waitSeconds(SHOOT_WAIT_SECONDS);
            launcherMotor.setPower(0);
        }

        // STEP 3: Return to starting encoder positions
        if (opModeIsActive()) {
            telemetry.addData("Step", "Returning");
            telemetry.update();

            returnToPosition(
                    flStart, frStart, blStart, brStart);
        }

        // STEP 4: Stop all motors
        stopAllMotors();

        telemetry.addData("Step", "Done");
        telemetry.update();
    }

    private void returnToPosition(
            int flTarget, int frTarget,
            int blTarget, int brTarget) {

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Positive power; encoder targets determine direction
        setDrivePower(RETURN_POWER);

        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < RETURN_TIMEOUT_SECONDS
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {

            telemetry.addData("Front Left",
                    "%d -> %d",
                    frontLeft.getCurrentPosition(), flTarget);
            telemetry.addData("Front Right",
                    "%d -> %d",
                    frontRight.getCurrentPosition(), frTarget);
            telemetry.addData("Back Left",
                    "%d -> %d",
                    backLeft.getCurrentPosition(), blTarget);
            telemetry.addData("Back Right",
                    "%d -> %d",
                    backRight.getCurrentPosition(), brTarget);

            telemetry.update();
            idle();
        }

        stopDrive();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetDriveEncoders() {
        frontLeft.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        launcherMotor.setPower(0);
    }

    private void waitSeconds(double seconds) {
        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < seconds) {
            idle();
        }
    }
}

