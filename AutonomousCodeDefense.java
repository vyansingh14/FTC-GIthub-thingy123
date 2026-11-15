package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * DEFENSE autonomous: for when our robot is big and not good at shooting, and our partner does the shooting.
 * This OpMode never shoots or runs the intake. It drives to a blocking spot, then HOLDS that spot
 * (actively resisting being pushed) until autonomous ends.
 *
 * Before pressing Start, on gamepad1:
 *   Plan:     D-pad UP    = BLOCK_FLOWERS   drive to the spot that blocks opponents from the FLOWERS
 *             D-pad LEFT  = GUARD_PARTNER   drive to a spot that shields our partner's shooting area
 *             D-pad RIGHT = STAY_CLEAR      just move out of our partner's way (safest)
 *   Alliance: X = BLUE (routes below are written for BLUE),  B = RED (mirrors left/right and turns)
 *
 * IMPORTANT: every distance below is a placeholder. Measure them on the real field.
 * Also read the manual's in-match rules (Section 11.4) about contact, blocking, and pinning,
 * especially during AUTO, before relying on this in a match.
 */
@Autonomous(name = "robotAutonomousDefense", group = "Autonomous")
public class robotAutonomousDefense extends LinearOpMode {

    private enum Plan { BLOCK_FLOWERS, GUARD_PARTNER, STAY_CLEAR }

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor[] wheels;

    private IMU imu;

    // Heading the robot is supposed to be facing: 0 = starting direction, positive = left
    private double targetHeading = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;

    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ---- Driving ----
    // Moderate power: a heavy robot overshoots more, and we don't want to hit anyone at speed.
    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_TIMEOUT_SECONDS = 10.0;
    // Mecanum strafing travels less than the encoder math says. TUNE: raise (for example 1.15)
    // until a measured strafe matches. 1.0 = no correction.
    private static final double STRAFE_CORRECTION = 1.0;

    // Power used to hold the spot. Higher resists pushing better, but draws more current.
    private static final double HOLD_POWER = 0.5;

    // Wait this long before moving, so our partner can leave their starting spot and shoot first. TUNE.
    private static final double START_DELAY_SECONDS = 0.0;

    // ---- IMU turning ----
    private static final double TURN_POWER = 0.5;
    private static final double TURN_MIN_POWER = 0.12;
    private static final double TURN_KP = 0.02;
    private static final double TURN_TOLERANCE_DEGREES = 1.0;
    private static final double TURN_TIMEOUT_SECONDS = 3.0;
    private static final double TURN_SETTLE_SECONDS = 0.25;

    // ---- Routes (written for BLUE). forward: + is forward. strafe: + is right. turn: + is left. ----
    // All TUNE: measure from your starting spot on the real FIELD.

    // BLOCK_FLOWERS: get in front of the opponents' path to the FLOWERS.
    private static final double BLOCK_FORWARD_INCHES = 60.0;
    private static final double BLOCK_STRAFE_INCHES = 24.0;
    private static final double BLOCK_TURN_DEGREES = 0.0;

    // GUARD_PARTNER: park between the opponents' lane and our partner's shooting spot.
    private static final double GUARD_FORWARD_INCHES = 30.0;
    private static final double GUARD_STRAFE_INCHES = -12.0;
    private static final double GUARD_TURN_DEGREES = 90.0;

    // STAY_CLEAR: slide out of the way and stop.
    private static final double CLEAR_FORWARD_INCHES = 0.0;
    private static final double CLEAR_STRAFE_INCHES = -18.0;
    private static final double CLEAR_TURN_DEGREES = 0.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        wheels = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        // IMU: Control Hub logo facing up, USB ports pointing toward the front of the robot
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---- Pick a plan and an alliance while waiting for Start ----
        Plan plan = Plan.STAY_CLEAR;
        boolean isRed = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                plan = Plan.BLOCK_FLOWERS;
            } else if (gamepad1.dpad_left) {
                plan = Plan.GUARD_PARTNER;
            } else if (gamepad1.dpad_right) {
                plan = Plan.STAY_CLEAR;
            }

            if (gamepad1.x) {
                isRed = false;
            } else if (gamepad1.b) {
                isRed = true;
            }

            telemetry.addLine("D-pad: UP = block flowers | LEFT = guard partner | RIGHT = stay clear");
            telemetry.addLine("X = BLUE   B = RED");
            telemetry.addData("Plan", plan);
            telemetry.addData("Alliance", isRed ? "RED (mirrored)" : "BLUE");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Red is the mirror image of blue: strafe and turn the other way.
        double side = isRed ? -1.0 : 1.0;

        waitSeconds(START_DELAY_SECONDS);

        switch (plan) {
            case BLOCK_FLOWERS:
                goTo(BLOCK_FORWARD_INCHES, BLOCK_STRAFE_INCHES * side, BLOCK_TURN_DEGREES * side);
                break;
            case GUARD_PARTNER:
                goTo(GUARD_FORWARD_INCHES, GUARD_STRAFE_INCHES * side, GUARD_TURN_DEGREES * side);
                break;
            case STAY_CLEAR:
                goTo(CLEAR_FORWARD_INCHES, CLEAR_STRAFE_INCHES * side, CLEAR_TURN_DEGREES * side);
                break;
        }

        // Stay put and resist being pushed until autonomous ends.
        holdPosition();
    }

    // ================= Moving into position =================

    // Drive forward, then strafe, then turn (skips any move that is zero).
    private void goTo(double forwardInches, double strafeInches, double turnLeftDegrees) {

        if (forwardInches != 0) {
            driveForwardInches(forwardInches, DRIVE_POWER);
        }
        if (strafeInches != 0) {
            strafeRightInches(strafeInches, DRIVE_POWER);
        }
        if (turnLeftDegrees != 0) {
            turnToHeading(targetHeading + turnLeftDegrees);
        }
    }

    // Lock each wheel to where it is now. If an opponent pushes the robot, the motors push back.
    private void holdPosition() {

        for (DcMotor wheel : wheels) {
            wheel.setTargetPosition(wheel.getCurrentPosition());
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheel.setPower(HOLD_POWER);
        }

        while (opModeIsActive()) {
            telemetry.addLine("Holding position");
            telemetry.addData("Heading", "%.1f", getHeading());
            telemetry.update();
            idle();
        }

        setDrivePower(0);
    }

    // ================= Driving =================

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

        // Back to normal mode, so the turns that follow actually spin the wheels
        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // ================= Turning (IMU) =================

    // Turn until the IMU heading matches the target heading. Targets are absolute,
    // so a small error in one turn doesn't carry into the next one.
    private void turnToHeading(double target) {

        targetHeading = normalizeDegrees(target);

        setDrivePower(0);
        waitSeconds(TURN_SETTLE_SECONDS);

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < TURN_TIMEOUT_SECONDS) {

            double error = normalizeDegrees(targetHeading - getHeading());

            if (Math.abs(error) <= TURN_TOLERANCE_DEGREES) {
                break;
            }

            // Slow down as it gets close; positive error means turn left
            double power = Math.abs(error) * TURN_KP;
            power = Math.max(TURN_MIN_POWER, Math.min(TURN_POWER, power));
            power = Math.signum(error) * power;

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Target heading", "%.1f", targetHeading);
            telemetry.addData("Heading", "%.1f", getHeading());
            telemetry.update();
            idle();
        }

        setDrivePower(0);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    // ================= Utilities =================

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
