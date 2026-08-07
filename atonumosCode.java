package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Random;

@Autonomous(name = "atonumosCode")
public class atonumosCode extends LinearOpMode {

    // Motors
    private DcMotor MotorTR;
    private DcMotor MotorTL;
    private DcMotor MotorBR;
    private DcMotor MotorBL;

    // Distance Sensor
    private DistanceSensor distance;

    private final Random random = new Random();

    // ===== Speeds (4x slower) =====
    private static final double DRIVE_POWER = 0.05;
    private static final double TURN_POWER = 0.1t ;

    // ===== Distances =====
    private static final double STOP_DISTANCE = 40.0;   // cm
    private static final double OPEN_DISTANCE = 60.0;   // cm

    @Override
    public void runOpMode() {

        // Motors
        MotorTR = hardwareMap.get(DcMotor.class, "front right");
        MotorTL = hardwareMap.get(DcMotor.class, "front left");
        MotorBR = hardwareMap.get(DcMotor.class, "back right");
        MotorBL = hardwareMap.get(DcMotor.class, "back left");

        // Same directions as TeleOp
        MotorTR.setDirection(DcMotor.Direction.REVERSE);
        MotorBR.setDirection(DcMotor.Direction.REVERSE);

        MotorTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Distance Sensor
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double dist = distance.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", "%.1f", dist);
            telemetry.update();

            if (dist > STOP_DISTANCE) {

                driveForward(DRIVE_POWER);

            } else {

                stopMotors();
                sleep(300);

                // Back up a little
                driveBackward(0.10);
                sleep(700);

                stopMotors();
                sleep(300);

                // Find a new path
                searchForOpening();
            }
        }

        stopMotors();
    }

    // ===========================
    // Drive Functions
    // ===========================

    private void driveForward(double power) {

        // Inverted because your robot was driving backwards
        MotorTL.setPower(-power);
        MotorTR.setPower(-power);
        MotorBL.setPower(-power);
        MotorBR.setPower(-power);
    }

    private void driveBackward(double power) {

        MotorTL.setPower(power);
        MotorTR.setPower(power);
        MotorBL.setPower(power);
        MotorBR.setPower(power);
    }

    private void turnLeft(double power) {

        MotorTL.setPower(-power);
        MotorBL.setPower(-power);
        MotorTR.setPower(power);
        MotorBR.setPower(power);
    }

    private void turnRight(double power) {

        MotorTL.setPower(power);
        MotorBL.setPower(power);
        MotorTR.setPower(-power);
        MotorBR.setPower(-power);
    }

    private void stopMotors() {

        MotorTL.setPower(0);
        MotorTR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
    }

    // ===========================
    // Obstacle Avoidance
    // ===========================

    private void searchForOpening() {

        boolean turnLeft = random.nextBoolean();

        while (opModeIsActive()) {

            double dist = distance.getDistance(DistanceUnit.CM);

            telemetry.addData("Searching", dist);
            telemetry.update();

            if (dist > OPEN_DISTANCE) {

                stopMotors();
                sleep(300);

                // Drive into the opening
                driveForward(DRIVE_POWER);
                sleep(1200);

                stopMotors();
                return;
            }

            if (turnLeft) {
                turnLeft(TURN_POWER);
            } else {
                turnRight(TURN_POWER);
            }
        }
    }
}
