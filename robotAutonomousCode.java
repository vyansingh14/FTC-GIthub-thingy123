package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "robotAutonomousCodeBlue1_far", group = "Autonomous")
public class robotAutonomousCodeBlue1_far extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor launcherMotor;
    private CRServo feederServo;

    private DcMotor intakeMotor;
    private CRServo intakeLeftServo;
    private CRServo intakeRightServo;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;

    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double LAUNCHER_POWER = 1.0;
    private static final double FEEDER_POWER = 1.0;
    private static final double SHOOT_SECONDS = 4.0;

    private static final double DRIVE_POWER = 0.5;
    private static final double PRE_SHOOT_DRIVE_INCHES = 8.0;

    private static final double TURN_POWER = 0.5;
    private static final double TURN_TIME_SECONDS = 0.8;

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
        intakeLeftServo = hardwareMap.get(CRServo.class, "intakeLeftServo");
        intakeRightServo = hardwareMap.get(CRServo.class, "intakeRightServo");

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

        intakeLeftServo.setPower(0);
        intakeRightServo.setPower(0);
        intakeMotor.setPower(0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // STEP 1: Drive forward 8 inches, then back up 0.5 inches
        driveForwardInches(PRE_SHOOT_DRIVE_INCHES, DRIVE_POWER);
        driveForwardInches(-0.5, DRIVE_POWER);

        // STEP 2: Launch (launcher and feeder together for 4 seconds)
        launcherMotor.setPower(LAUNCHER_POWER);
        feederServo.setPower(FEEDER_POWER);

        waitSeconds(SHOOT_SECONDS);

        launcherMotor.setPower(0);
        feederServo.setPower(0);

        // STEP 3: Turn left
        turnLeft(TURN_POWER, TURN_TIME_SECONDS);

        // STEP 4: Drive forward 46 inches
        driveForwardInches(46, DRIVE_POWER);

        // STEP 5: Turn right
        turnRight(TURN_POWER, TURN_TIME_SECONDS);

        // STEP 6: Drive forward 45 inches
        driveForwardInches(45, DRIVE_POWER);

        // STEP 7: Strafe sideways 11 inches (right; use strafeLeftInches to go the other way)
        strafeRightInches(11, DRIVE_POWER);

        // STEP 8: Drive forward about 5 inches
        driveForwardInches(5, DRIVE_POWER);

        // STEP 9: Turn left 90 degrees
        turnLeft(TURN_POWER, TURN_TIME_SECONDS);

        // STEP 10: Drive forward 6.5 inches
        driveForwardInches(6.5, DRIVE_POWER);

        // STEP 11: Strafe sideways 1.5 inches (right) before the intakes turn on
        strafeRightInches(1.5, DRIVE_POWER);

        // STEP 12: Spin left intake servo, right intake servo, and intake motor for 4 seconds
        intakeMotor.setPower(1.0);
        intakeLeftServo.setPower(1.0);
        intakeRightServo.setPower(1.0);

        waitSeconds(INTAKE_SECONDS);

        intakeMotor.setPower(0);
        intakeLeftServo.setPower(0);
        intakeRightServo.setPower(0);

        // STEP 13: Back up 5 inches
        driveForwardInches(-5, DRIVE_POWER);

        // STEP 14: Turn right
        turnRight(TURN_POWER, TURN_TIME_SECONDS);

        // STEP 15: Drive forward 65 inches
        driveForwardInches(65, DRIVE_POWER);

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

    // Left wheels backward, right wheels forward
    private void turnLeft(double power, double seconds) {

        frontLeft.setPower(-power);
        backLeft.setPower(-power);

        frontRight.setPower(power);
        backRight.setPower(power);

        waitSeconds(seconds);

        setDrivePower(0);
    }

    // Left wheels forward, right wheels backward
    private void turnRight(double power, double seconds) {

        frontLeft.setPower(power);
        backLeft.setPower(power);

        frontRight.setPower(-power);
        backRight.setPower(-power);

        waitSeconds(seconds);

        setDrivePower(0);
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
        intakeLeftServo.setPower(0);
        intakeRightServo.setPower(0);
    }

    private void waitSeconds(double seconds) {

        runtime.reset();

        while (opModeIsActive()
                && runtime.seconds() < seconds) {
            idle();
        }
    }
}
