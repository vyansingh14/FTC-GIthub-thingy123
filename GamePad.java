package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "GamePad")
public class GamePad extends OpMode {

    // ---- Tunable constants: drive ----
    private static final double SPEED_FAST   = 1.0;
    private static final double SPEED_NORMAL = 0.5;
    private static final double SPEED_SLOW   = 0.25;

    // ---- Tunable constants: launcher servo ----
    // Servo positions MUST be between 0.0 and 1.0.
    private static final double SERVO_HOME_POSITION      = 0.22;
    private static final double SERVO_LAUNCH_POSITION    = 1.0; // adjust to whatever position actually fires the launcher
    private static final long   SERVO_LAUNCH_DURATION_MS = 300;

    private static final double CLOSE_OBJECT_THRESHOLD_CM = 10;

    // ---- Tunable constants: arm ----
    // Base and elbow are driven proportionally by the joysticks, so these are
    // "position change per loop iteration" values, not absolute positions.
    // Tune these until the arm moves at a comfortable speed.
    private static final double ARM_BASE_SERVO_SPEED  = 0.01;
    private static final double ARM_ELBOW_SERVO_SPEED = 0.01;

    private static final double ARM_BASE_START_POSITION  = 0.5;
    private static final double ARM_ELBOW_START_POSITION = 0.5;

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
    private DistanceSensor distanceSensor; // optional: null if not found
    private Servo launcherServo;           // optional: null if not found

    // ---- Hardware: arm (gamepad2) ----
    private Servo armBaseServo;   // optional: null if not found
    private Servo armElbowServo;  // optional: null if not found
    private Servo armClawServo;   // optional: null if not found

    // ---- Launcher servo state ----
    private boolean lastA = false;
    private boolean servoMoving = false;
    private final ElapsedTime servoTimer = new ElapsedTime();

    // ---- Arm state ----
    // Tracked in code because you can't read the current position back from a
    // standard servo; these track where we last told it to go.
    private double armBasePosition  = ARM_BASE_START_POSITION;
    private double armElbowPosition = ARM_ELBOW_START_POSITION;

    @Override
    public void init() {
        // Drive motors are required. If a name is wrong, the Robot Controller
        // reports a clear "device not found" error here instead of crashing later.
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorBackRight  = hardwareMap.get(DcMotor.class, "back_right_motor");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");

        // Reverse right side motors so positive power drives all wheels "forward"
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft}) {
            // RUN_WITHOUT_ENCODER works whether or not encoder cables are plugged in.
            // Switch back to RUN_USING_ENCODER if you have encoders wired and want velocity control.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // BRAKE stops the robot crisply when the stick is released.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Sensor and launcher servo are optional: the robot can still drive if they're missing.
        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        } catch (Exception e) {
            telemetry.addLine("ERROR: distance sensor 'distance' not found");
        }

        try {
            launcherServo = hardwareMap.get(Servo.class, "launcherServo");
            launcherServo.setPosition(SERVO_HOME_POSITION);
        } catch (Exception e) {
            launcherServo = null;
            telemetry.addLine("ERROR: servo 'launcherServo' not found");
        }

        // Arm servos are optional too, so a missing one doesn't take down the whole OpMode.
        try {
            armBaseServo = hardwareMap.get(Servo.class, "armBaseServo");
            armBaseServo.setPosition(armBasePosition);
        } catch (Exception e) {
            armBaseServo = null;
            telemetry.addLine("ERROR: servo 'armBaseServo' not found");
        }

        try {
            armElbowServo = hardwareMap.get(Servo.class, "armElbowServo");
            armElbowServo.setPosition(armElbowPosition);
        } catch (Exception e) {
            armElbowServo = null;
            telemetry.addLine("ERROR: servo 'armElbowServo' not found");
        }

        try {
            armClawServo = hardwareMap.get(Servo.class, "armClawServo");
            armClawServo.setPosition(CLAW_OPEN_POSITION);
        } catch (Exception e) {
            armClawServo = null;
            telemetry.addLine("ERROR: servo 'armClawServo' not found");
        }

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // gamepad1 drives the robot, gamepad2 runs the arm.
        driveMecanum();
        handleLauncher();
        handleArm();
        updateTelemetry();
    }

    private void driveMecanum() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

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

    private void handleLauncher() {
        if (launcherServo == null) return;

        boolean aPressed = gamepad1.a && !lastA;
        lastA = gamepad1.a;

        if (aPressed && !servoMoving) {
            launcherServo.setPosition(SERVO_LAUNCH_POSITION);
            servoMoving = true;
            servoTimer.reset();
        }

        if (servoMoving && servoTimer.milliseconds() >= SERVO_LAUNCH_DURATION_MS) {
            launcherServo.setPosition(SERVO_HOME_POSITION);
            servoMoving = false;
        }
    }

    private void handleArm() {
        // Left stick moves the base, right stick moves the elbow (middle joint).
        // Because these are position servos (not continuous-rotation), we nudge
        // a stored target position each loop rather than setting power directly.
        double baseStick  = gamepad2.left_stick_y;
        double elbowStick = -gamepad2.right_stick_y;

        if (Math.abs(baseStick) > JOYSTICK_DEADZONE && armBaseServo != null) {
            armBasePosition += baseStick * ARM_BASE_SERVO_SPEED;
            armBasePosition = Range.clip(armBasePosition, 0.0, 1.0);
            armBaseServo.setPosition(armBasePosition);
        }

        if (Math.abs(elbowStick) > JOYSTICK_DEADZONE && armElbowServo != null) {
            armElbowPosition += elbowStick * ARM_ELBOW_SERVO_SPEED;
            armElbowPosition = Range.clip(armElbowPosition, 0.0, 1.0);
            armElbowServo.setPosition(armElbowPosition);
        }

        // A opens the claw, B closes it.
        if (armClawServo != null) {
            if (gamepad2.a) {
                armClawServo.setPosition(CLAW_CLOSED_POSITION);
            } else if (gamepad2.b) {
                armClawServo.setPosition(CLAW_OPEN_POSITION);
            }
        }
    }

    private void updateTelemetry() {
        if (distanceSensor != null) {
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", "%.1f", distanceCM);
            if (distanceCM < CLOSE_OBJECT_THRESHOLD_CM) {
                telemetry.addLine("Warning: object very close!");
            }
        }

        if (launcherServo != null) {
            // This is the last commanded position, not a physical reading
            telemetry.addData("Launcher Servo Commanded Position", launcherServo.getPosition());
        }

        if (armBaseServo != null) {
            telemetry.addData("Arm Base Position", "%.2f", armBasePosition);
        }
        if (armElbowServo != null) {
            telemetry.addData("Arm Elbow Position", "%.2f", armElbowPosition);
        }
        if (armClawServo != null) {
            telemetry.addData("Claw Commanded Position", armClawServo.getPosition());
        }

        // No telemetry.update() here: OpMode sends telemetry automatically after each loop()
    }
}
