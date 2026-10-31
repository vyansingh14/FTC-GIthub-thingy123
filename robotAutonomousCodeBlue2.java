package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "robotAutonomousCodeBlue2", group = "Autonomous")
public class robotAutonomousCodeBlue2 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor launcherMotor;
    private CRServo feederServo;

    private DcMotor intakeMotor;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;

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

    private static final double LAUNCHER_POWER = 0.575;
    private static final double FEEDER_POWER = 1.0;
    private static final int FEEDER_STEPS = 4;          // 4 spins of 90 degrees = one full 360 degree turn
    private static final long FEEDER_PULSE_MS = 250;   // TUNE: how long the feeder spins to turn about 90 degrees
    private static final long FEEDER_GAP_MS = 500;     // pause between feeder spins

    private static final double DRIVE_POWER = 0.5;
    private static final double PRE_SHOOT_DRIVE_INCHES = 5.0;

    // IMU turning
    private static final double TURN_POWER = 0.5;               // fastest turn power
    private static final double TURN_MIN_POWER = 0.12;          // slowest power, so it never stalls near the target
    private static final double TURN_KP = 0.02;                 // power per degree of error
    private static final double TURN_TOLERANCE_DEGREES = 1.0;   // close enough
    private static final double TURN_TIMEOUT_SECONDS = 3.0;
    private static final double TURN_SETTLE_SECONDS = 0.25;    // pause before each turn so the robot stops rolling

    private static final double DRIVE_TIMEOUT_SECONDS = 10.0;

    private static final double INTAKE_SECONDS = 4.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");

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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setPower(0);

        feederServo.setDirection(CRServo.Direction.REVERSE);
        feederServo.setPower(0);

        intakeServoLeft.setPower(0);
        intakeServoRight.setPower(0);
        intakeMotor.setPower(0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // STEP 1: Back up 0.5 inches (no forward drive before shooting)
        driveForwardInches(-0.5, DRIVE_POWER);

        // STEP 2: Strafe right 24 inches before shooting (use strafeLeftInches to go the other way)
        strafeRightInches(24, DRIVE_POWER);

        // STEP 3: Move backward 9.45 inches before shooting
        driveForwardInches(-9.45, DRIVE_POWER);

        // STEP 4: Launch (launcher spins while the feeder turns 90 degrees four times, with a 500 ms gap between turns)
        shootWithPulsedFeeder();

        // STEP 5: Turn right 90 degrees
        turnRight(90);

        // STEP 6: Drive forward 32 inches
        driveForwardInches(32, DRIVE_POWER);

        // STEP 7: Turn right 90 degrees
        turnRight(90);

        // STEP 8: Drive forward 90 inches
        driveForwardInches(90, DRIVE_POWER);

        // STEP 9: Strafe left 6 inches
        strafeLeftInches(6, DRIVE_POWER);

        // STEP 10: Drive forward 6 inches
        driveForwardInches(6, DRIVE_POWER);

        // STEP 11: Strafe left 6 more inches
        strafeLeftInches(6, DRIVE_POWER);

        stopAllMotors();
    }

    private void driveForwardInches(double inches, double power) {

        int counts = (int)Math.round(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(counts);
        frontRight.setTargetPosition(counts);
        backLeft.setTargetPosition(counts);
        backRight.setTargetPosition(counts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Positive inches strafes right, negative strafes left
    private void strafeRightInches(double inches, double power) {

        int counts = (int)Math.round(inches * COUNTS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(counts);
        frontRight.setTargetPosition(-counts);
        backLeft.setTargetPosition(-counts);
        backRight.setTargetPosition(counts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeLeftInches(double inches, double power) {
        strafeRightInches(-inches, power);
    }

    // Turn left by this many degrees (measured with the IMU)
    private void turnLeft(double degrees) {
        turnToHeading(targetHeading + degrees);
    }

    // Turn right by this many degrees (measured with the IMU)
    private void turnRight(double degrees) {
        turnToHeading(targetHeading - degrees);
    }

    // Turn until the IMU heading matches the target heading. Targets are absolute,
    // so a small error in one turn doesn't carry into the next one.
    private void turnToHeading(double target) {

        targetHeading = normalizeDegrees(target);

        // Let the wheels stop completely so leftover forward motion doesn't carry into the turn
        setDrivePower(0);
        waitSeconds(TURN_SETTLE_SECONDS);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    // Robot heading in degrees: 0 at start, positive = turned left
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Wrap an angle into the range -180 to 180
    private double normalizeDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    // The launcher spins while the feeder turns 90 degrees at a time, with a gap between turns,
    // until it has made one full 360 degree turn (FEEDER_STEPS spins). Then everything stops
    // and the robot moves on.
    private void shootWithPulsedFeeder() {

        launcherMotor.setPower(LAUNCHER_POWER);

        for (int i = 0; i < FEEDER_STEPS && opModeIsActive(); i++) {

            feederServo.setPower(FEEDER_POWER);
            sleep(FEEDER_PULSE_MS);

            feederServo.setPower(0);
            sleep(FEEDER_GAP_MS);
        }

        launcherMotor.setPower(0);
        feederServo.setPower(0);
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopAllMotors() {
        setDrivePower(0);

        launcherMotor.setPower(0);
        feederServo.setPower(0);

        intakeMotor.setPower(0);
        intakeServoLeft.setPower(0);
        intakeServoRight.setPower(0);
    }

    private void waitSeconds(double seconds) {

        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < seconds) {
            idle();
        }
    }
}
