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
    // Launcher power, adjusted with the gamepad2 d-pad:
    // each tap of d-pad up adds LAUNCHER_STEP, each tap of d-pad down subtracts it.
    private static final double LAUNCHER_POWER_START = 0.6;  // power when the OpMode starts
    private static final double LAUNCHER_STEP        = 0.02; // change per tap
    private static final double LAUNCHER_POWER_MIN   = 0.0;
    private static final double LAUNCHER_POWER_MAX   = 1.0;

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

    // How far the right trigger has to be squeezed to count as a press (0.0 to 1.0).
    private static final double TRIGGER_PRESS = 0.5;

    // ---- Hardware: drivetrain (gamepad1) ----
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    // ---- Hardware: intake (gamepad2) ----
    private CRServo intakeServoLeft;  // optional: null if not found
    private CRServo intakeServoRight; // optional: null if not found
    private DcMotor intakeMotor;      // optional: null if not found

    // ---- Hardware: launcher (gamepad2) ----
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
    private boolean lastFeederButton = false;
    private boolean feederOn = false;

    private boolean lastTrigger = false;
    private boolean launcherOn = false;
    // ---- Drive direction state ----
    // Y on gamepad1 swaps which end of the robot is the "front" (the launcher/arm end).
    private boolean lastY = false;
    private boolean driveReversed = false;

    private double launcherPower = LAUNCHER_POWER_START;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

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
        // gamepad1 = drivetrain only.
        // gamepad2 = intake, feeder, launcher, arm and claw.
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
        // gamepad2 X toggles the whole intake (both continuous servos + the intake motor) on and off.
        boolean xPressed = gamepad2.x && !lastX;
        lastX = gamepad2.x;

        if (xPressed) {
            intakeOn = !intakeOn;
        }

        double servoPower = intakeOn ? INTAKE_SERVO_POWER : 0.0;
        if (intakeServoLeft != null)  intakeServoLeft.setPower(servoPower);
        if (intakeServoRight != null) intakeServoRight.setPower(servoPower);
        if (intakeMotor != null)      intakeMotor.setPower(intakeOn ? INTAKE_MOTOR_POWER : 0.0);
    }

    private void handleLauncher() {
        // gamepad2 Y toggles the feeder on and off. It is separate from the intake (X).
        boolean feederPressed = gamepad2.y && !lastFeederButton;
        lastFeederButton = gamepad2.y;

        if (feederPressed) {
            feederOn = !feederOn;
        }

        if (feederServo != null) {
            feederServo.setPower(isFeederRunning() ? FEEDER_POWER : 0.0);
        }

        // gamepad2 right trigger toggles the launcher motor on and off.
        // (A and B on gamepad2 are already the claw.)
        boolean triggerDown = gamepad2.right_trigger > TRIGGER_PRESS;
        boolean triggerPressed = triggerDown && !lastTrigger;
        lastTrigger = triggerDown;

        if (triggerPressed) {
            launcherOn = !launcherOn;
        }

        // gamepad2 d-pad up/down nudges the launcher power by LAUNCHER_STEP per tap.
        // Holding the button only counts once; tap again for another step.
        // Works whether the launcher is on or off.
        boolean upPressed = gamepad2.dpad_up && !lastDpadUp;
        lastDpadUp = gamepad2.dpad_up;
        boolean downPressed = gamepad2.dpad_down && !lastDpadDown;
        lastDpadDown = gamepad2.dpad_down;

        if (upPressed) {
            launcherPower += LAUNCHER_STEP;
        } else if (downPressed) {
            launcherPower -= LAUNCHER_STEP;
        }
        // Keep it between MIN and MAX, and round to 2 decimals so it reads 0.62, not 0.6200000001.
        launcherPower = Math.max(LAUNCHER_POWER_MIN, Math.min(LAUNCHER_POWER_MAX, launcherPower));
        launcherPower = Math.round(launcherPower * 100.0) / 100.0;

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
        telemetry.addData("Intake (GP2 X)", intakeOn ? "ON" : "OFF");

        // Shows the power actually being sent, or MISSING if the device never loaded.
        telemetry.addData("Feeder (GP2 Y)", feederServo == null
                ? "MISSING - check config name/type"
                : String.format("%s  power %.2f", isFeederRunning() ? "ON" : "OFF", feederServo.getPower()));
        telemetry.addData("Launcher (GP2 RT)", launcherMotor == null
                ? "MISSING - check config name"
                : String.format("%s  set to %.2f  (d-pad up/down)",
                launcherOn ? "ON" : "OFF", launcherPower));

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
