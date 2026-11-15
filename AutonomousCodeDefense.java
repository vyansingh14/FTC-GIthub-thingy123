package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PARK autonomous: drive off the starting spot, move to the parking spot, and stop.
 * No shooting, no intake, no turning, and no holding against opponents.
 *
 * Before pressing Start, on gamepad1:  X = BLUE,  B = RED (red mirrors the strafe direction).
 *
 * Every distance below is a placeholder. Measure them on the real FIELD.
 */
@Autonomous(name = "robotAutonomousPark", group = "Autonomous")
public class robotAutonomousPark extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor[] wheels;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;

    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Low power and a short timeout, so if something is in the way the robot gives up quickly
    // instead of pushing into it for a long time.
    private static final double DRIVE_POWER = 0.3;
    private static final double DRIVE_TIMEOUT_SECONDS = 4.0;

    // Mecanum strafing travels less than the encoder math says. TUNE: raise (for example 1.15)
    // until a measured strafe matches. 1.0 = no correction.
    private static final double STRAFE_CORRECTION = 1.0;

    // Wait this long before moving, so our partner can leave their starting spot first. TUNE.
    private static final double START_DELAY_SECONDS = 0.0;

    // ---- Route (written for BLUE). forward: + is forward. strafe: + is right. All TUNE. ----
    private static final double LEAVE_FORWARD_INCHES = 24.0;  // drive off the starting spot
    private static final double PARK_STRAFE_INCHES = -24.0;   // slide over to the parking spot

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        wheels = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---- Pick an alliance while waiting for Start ----
        boolean isRed = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                isRed = false;
            } else if (gamepad1.b) {
                isRed = true;
            }

            telemetry.addLine("X = BLUE   B = RED");
            telemetry.addData("Alliance", isRed ? "RED (mirrored)" : "BLUE");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Red is the mirror image of blue, so strafe the other way.
        double side = isRed ? -1.0 : 1.0;

        waitSeconds(START_DELAY_SECONDS);

        // STEP 1: Leave the starting spot
        driveForwardInches(LEAVE_FORWARD_INCHES, DRIVE_POWER);

        // STEP 2: Move to the parking spot
        strafeRightInches(PARK_STRAFE_INCHES * side, DRIVE_POWER);

        // Done: stay parked (BRAKE keeps the wheels from rolling)
        setDrivePower(0);
    }

    private void driveForwardInches(double inches, double power) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH);
        runToPosition(counts, counts, counts, counts, power);
    }

    // Positive inches strafes right, negative strafes left
    private void strafeRightInches(double inches, double power) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH * STRAFE_CORRECTION);
        runToPosition(counts, -counts, -counts, counts, power);
    }

    // Order: front left, front right, back left, back right
    private void runToPosition(int fl, int fr, int bl, int br, double power) {

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontLeft.setTargetPosition(fl);
        frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);
        backRight.setTargetPosition(br);

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        setDrivePower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < DRIVE_TIMEOUT_SECONDS
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {
            idle();
        }

        setDrivePower(0);

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setDrivePower(double power) {
        for (DcMotor wheel : wheels) {
            wheel.setPower(power);
        }
    }

    private void waitSeconds(double seconds) {

        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < seconds) {
            idle();
        }
    }
}
