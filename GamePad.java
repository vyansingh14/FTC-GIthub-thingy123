package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "GamePad")
public class GamePad extends OpMode {

    // ---- Tunable constants: drive ----
    private static final double SPEED_FAST   = 1.0;
    private static final double SPEED_NORMAL = 0.5;
    private static final double SPEED_SLOW   = 0.25;

    // ---- Tunable constants: intake ----
    // Continuous servos take power from -1.0 to 1.0. Flip the sign if the intake spins the wrong way.
    private static final double INTAKE_SERVO_POWER = -1.0;
    private static final double INTAKE_MOTOR_POWER = 1.0;

    // ---- Tunable constants: launcher ----
    // Feeder is a continuous servo: power from -1.0 to 1.0. Flip the sign if it spins the wrong way.
    private static final double FEEDER_POWER = 1.0;
    // Launcher speed, picked with the gamepad1 d-pad. The choice stays until you pick another one:
    // d-pad up = fast, d-pad left or right = normal, d-pad down = slow.
    private static final double LAUNCHER_POWER_FAST   = 0.6;
    private static final double LAUNCHER_POWER_NORMAL = 0.575;
    private static final double LAUNCHER_POWER_SLOW   = 0.55;

    // ---- Tunable constants: arm ----
    // Elbow is a DC motor, so this is a power multiplier (0.0 to 1.0).
    // Elbow speed on gamepad2, chosen with the bumpers like the drive speed on gamepad1:
    // hold left bumper = fast, hold right bumper = slow, neither = normal.
    private static final double ARM_ELBOW_POWER_FAST   = 0.4;
    private static final double ARM_ELBOW_POWER_NORMAL = 0.3;
    private static final double ARM_ELBOW_POWER_SLOW   = 0.2;

    private static final double CLAW_OPEN_POSITION   = 0.6;
    private static final double CLAW_CLOSED_POSITION = 0.3;

    // Joystick values smaller than this are treated as zero, so the arm
    // doesn't creep from stick drift when the joystick is released.
    private static final double JOYSTICK_DEADZONE = 0.05;

    // ---- Hardware: drivetrain (gamepad1) ----
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    // ---- Hardware: intake (gamepad1) ----
    private CRServo intakeServoLeft;  // optional: null if not found
    private CRServo intakeServoRight; // optional: null if not found
    private DcMotor intakeMotor;      // optional: null if not found

    // ---- Hardware: launcher (gamepad1) ----
    private CRServo feederServo;   // optional: null if not found
    private DcMotor launcherMotor; // optional: null if not found

    // ---- Hardware: arm (gamepad2) ----
    private DcMotor armElbowMotor; // optional: null if not found
    private Servo armClawServo;    // optional: null if not found

    // Names of any devices that failed to load. Shown on the Driver Station
    // the WHOLE match (not just during init, where it used to get wiped on Start).
    private final List<String> missingDevices = new ArrayList<>();

    // ---- Intake state ----
    private boolean lastX = false;
    private boolean intakeOn = false;

    // ---- Launcher state ----
    private boolean lastA = false;
    private boolean feederOn = false;

    private boolean lastB = false;
    private boolean launcherOn = false;
    // ---- Drive direction state ----
    // Y on gamepad1 swaps which end of the robot is the "front" (the launcher/arm end).
    private boolean lastY = false;
    private boolean driveReversed = false;

    private double launcherPower = LAUNCHER_POWER_FAST; // starts in fast mode
    private String launcherMode = "FAST";

    @Override
    public void init() {
        // Drive motors are required. If a name is wrong, the Robot Controller
        // reports a clear "device not found" error here instead of crashing later.
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorBackRight  = hardwareMap.get(DcMotor.class, "back_right_motor");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");

        // FIX: on most FTC drivetrains (goBILDA, REV) it's the LEFT side that gets reversed.
        // The old code reversed the right side AND left the forward stick un-negated. Those two
        // mistakes cancelled out for forward/backward, but made strafe and rotate go the wrong way.
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft}) {
            // RUN_WITHOUT_ENCODER works whether or not encoder cables are plugged in.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // BRAKE stops the robot crisply when the stick is released.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Intake and launcher parts are optional: the robot can still drive if they're missing.
        try {
            intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        } catch (Exception e) {
            intakeServoLeft = null;
            missingDevices.add("intakeServoLeft (CR servo)");
        }

        try {
            intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
            // The two intake servos face each other, so one is reversed to pull the ball in.
            intakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeServoRight = null;
            missingDevices.add("intakeServoRight (CR servo)");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            intakeMotor = null;
            missingDevices.add("intakeMotor (motor)");
        }

        try {
            feederServo = hardwareMap.get(CRServo.class, "feederServo");
        } catch (Exception e) {
            feederServo = null;
            missingDevices.add("feederServo (must be configured as Continuous Rotation Servo)");
        }

        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT lets the launcher wheel spin down on its own instead of slamming to a stop.
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            launcherMotor = null;
            missingDevices.add("launcherMotor (motor)");
        }

        try {
            armElbowMotor = hardwareMap.get(DcMotor.class, "armElbowMotor");
            armElbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            armElbowMotor = null;
            missingDevices.add("armElbowMotor (motor)");
        }

        try {
            armClawServo = hardwareMap.get(Servo.class, "armClawServo");
            // The claw servo is mounted mirrored, so its whole direction is flipped.
            // (Same as turning every position p into 1 - p.) Remove this line to undo.
            armClawServo.setDirection(Servo.Direction.REVERSE);
            armClawServo.setPosition(CLAW_OPEN_POSITION);
        } catch (Exception e) {
            armClawServo = null;
            missingDevices.add("armClawServo (servo)");
        }

        showMissingDevices();
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // gamepad1 drives, runs the intake and the launcher. gamepad2 only runs the arm and claw.
        driveMecanum();
        handleIntake();
        handleLauncher();
        handleArm();
        updateTelemetry();
    }

    private void driveMecanum() {
        // FIX: the gamepad reports pushing the stick UP as a NEGATIVE number, so it has to be flipped.
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        // Y toggles reverse mode. In reverse, the launcher/arm end is the front:
        // pushing the stick up drives toward the launcher, and left/right strafe flip to match.
        // Turning stays the same, since turning right is still turning right.
        boolean yPressed = gamepad1.y && !lastY;
        lastY = gamepad1.y;
        if (yPressed) {
            driveReversed = !driveReversed;
        }
        if (driveReversed) {
            forward = -forward;
            strafe  = -strafe;
        }

        double frontLeftPower  = forward + strafe + rotate;
        double backLeftPower   = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower  = forward + strafe - rotate;

        // Normalize so no wheel is asked for more than 100% power
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

        double speedMultiplier = SPEED_NORMAL;
        if (gamepad1.left_bumper) {
            speedMultiplier = SPEED_FAST;
        } else if (gamepad1.right_bumper) {
            speedMultiplier = SPEED_SLOW;
        }

        motorFrontLeft.setPower(frontLeftPower * speedMultiplier);
        motorFrontRight.setPower(frontRightPower * speedMultiplier);
        motorBackLeft.setPower(backLeftPower * speedMultiplier);
        motorBackRight.setPower(backRightPower * speedMultiplier);
    }

    private void handleIntake() {
        // X toggles the whole intake (both continuous servos + the intake motor) on and off.
        boolean xPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;

        if (xPressed) {
            intakeOn = !intakeOn;
        }

        double servoPower = intakeOn ? INTAKE_SERVO_POWER : 0.0;
        if (intakeServoLeft != null)  intakeServoLeft.setPower(servoPower);
        if (intakeServoRight != null) intakeServoRight.setPower(servoPower);
        if (intakeMotor != null)      intakeMotor.setPower(intakeOn ? INTAKE_MOTOR_POWER : 0.0);
    }

    private void handleLauncher() {
        // A toggles the feeder on and off. It is separate from the intake (X).
        boolean aPressed = gamepad1.a && !lastA;
        lastA = gamepad1.a;

        if (aPressed) {
            feederOn = !feederOn;
        }

        if (feederServo != null) {
            feederServo.setPower(isFeederRunning() ? FEEDER_POWER : 0.0);
        }

        // B toggles the launcher motor on and off.
        boolean bPressed = gamepad1.b && !lastB;
        lastB = gamepad1.b;

        if (bPressed) {
            launcherOn = !launcherOn;
        }

        // D-pad picks the launcher speed. It works whether the launcher is on or off.
        if (gamepad1.dpad_up) {
            launcherPower = LAUNCHER_POWER_FAST;
            launcherMode = "FAST";
        } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            launcherPower = LAUNCHER_POWER_NORMAL;
            launcherMode = "NORMAL";
        } else if (gamepad1.dpad_down) {
            launcherPower = LAUNCHER_POWER_SLOW;
            launcherMode = "SLOW";
        }

        if (launcherMotor != null) {
            launcherMotor.setPower(launcherOn ? launcherPower : 0.0);
        }
    }

    private void handleArm() {
        // Left stick Y drives the elbow motor. BRAKE holds the arm when the stick is released.
        if (armElbowMotor != null) {
            double elbowSpeed = ARM_ELBOW_POWER_NORMAL;
            if (gamepad2.left_bumper) {
                elbowSpeed = ARM_ELBOW_POWER_FAST;
            } else if (gamepad2.right_bumper) {
                elbowSpeed = ARM_ELBOW_POWER_SLOW;
            }

            double elbowStick = -gamepad2.left_stick_y;
            double elbowPower = Math.abs(elbowStick) > JOYSTICK_DEADZONE
                    ? elbowStick * elbowSpeed
                    : 0.0;
            armElbowMotor.setPower(elbowPower);
        }

        // A closes the claw, B opens it.
        if (armClawServo != null) {
            if (gamepad2.a) {
                armClawServo.setPosition(CLAW_CLOSED_POSITION);
            } else if (gamepad2.b) {
                armClawServo.setPosition(CLAW_OPEN_POSITION);
            }
        }
    }

    private boolean isFeederRunning() {
        return feederOn;
    }

    private void showMissingDevices() {
        for (String name : missingDevices) {
            telemetry.addLine("MISSING: " + name);
        }
    }

    private void updateTelemetry() {
        showMissingDevices();

        telemetry.addData("Front (Y)", driveReversed ? "LAUNCHER END" : "INTAKE END");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");

        // Shows the power actually being sent, or MISSING if the device never loaded.
        telemetry.addData("Feeder (A)", feederServo == null
                ? "MISSING - check config name/type"
                : String.format("%s  power %.2f", isFeederRunning() ? "ON" : "OFF", feederServo.getPower()));
        telemetry.addData("Launcher (B)", launcherMotor == null
                ? "MISSING - check config name"
                : String.format("%s  %s mode  power %.2f",
                launcherOn ? "ON" : "OFF", launcherMode, launcherMotor.getPower()));

        telemetry.addData("Sticks fwd/strafe/rot", "%.2f / %.2f / %.2f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Wheels FL/FR", "%.2f / %.2f",
                motorFrontLeft.getPower(), motorFrontRight.getPower());
        telemetry.addData("Wheels BL/BR", "%.2f / %.2f",
                motorBackLeft.getPower(), motorBackRight.getPower());

        if (armElbowMotor != null) {
            telemetry.addData("Elbow Power", "%.2f", armElbowMotor.getPower());
        }
        if (armClawServo != null) {
            telemetry.addData("Claw Commanded Position", armClawServo.getPosition());
        }
        // No telemetry.update() here: OpMode sends telemetry automatically after each loop()
    }
}
