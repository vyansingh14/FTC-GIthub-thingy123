package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GamePad")
public class GamePad extends OpMode {

    // ---- Tunable constants: drive ----
    private static final double SPEED_FAST   = 1.0;
    private static final double SPEED_NORMAL = 0.5;
    private static final double SPEED_SLOW   = 0.25;

    // ---- Tunable constants: intake ----
    // Continuous servos take power from -1.0 to 1.0. Flip the sign if the intake spins the wrong way.
    private static final double INTAKE_SERVO_POWER = 1.0;
    private static final double INTAKE_MOTOR_POWER = 1.0;

    // ---- Tunable constants: launcher ----
    // Feeder is a continuous servo: power from -1.0 to 1.0. Flip the sign if it spins the wrong way.
    private static final double FEEDER_POWER = 1.0;

    private static final double LAUNCHER_MOTOR_POWER = 1.0;

    // ---- Tunable constants: arm ----
    // Elbow is a DC motor, so this is a power multiplier (0.0 to 1.0).
    private static final double ARM_ELBOW_POWER = 0.5;

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

    // ---- Intake state ----
    private boolean lastX = false;
    private boolean intakeOn = false;

    // ---- Launcher state ----
    private boolean lastA = false;
    private boolean feederOn = false;

    private boolean lastB = false;
    private boolean launcherOn = false;

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

        // Intake and launcher parts are optional: the robot can still drive if they're missing.
        try {
            intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        } catch (Exception e) {
            intakeServoLeft = null;
            telemetry.addLine("ERROR: servo 'intakeServoLeft' not found");
        }

        try {
            intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
            // The two intake servos face each other, so one is reversed to pull the ball in.
            // Remove this line if both servos end up spinning the same way physically.
            intakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeServoRight = null;
            telemetry.addLine("ERROR: servo 'intakeServoRight' not found");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addLine("ERROR: motor 'intakeMotor' not found");
        }

        try {
            feederServo = hardwareMap.get(CRServo.class, "feederServo");
        } catch (Exception e) {
            feederServo = null;
            telemetry.addLine("ERROR: servo 'feederServo' not found");
        }

        try {
            launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT lets the launcher wheel spin down on its own instead of slamming to a stop.
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            launcherMotor = null;
            telemetry.addLine("ERROR: motor 'launcherMotor' not found");
        }

        // Elbow motor and claw servo are optional so a missing one doesn't take down the OpMode.
        try {
            armElbowMotor = hardwareMap.get(DcMotor.class, "armElbowMotor");
            armElbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            armElbowMotor = null;
            telemetry.addLine("ERROR: motor 'armElbowMotor' not found");
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
        // gamepad1 drives, runs the intake and the launcher. gamepad2 only runs the arm and claw.
        driveMecanum();
        handleIntake();
        handleLauncher();
        handleArm();
        updateTelemetry();
    }

    private void driveMecanum() {
        double forward = gamepad1.left_stick_y;
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
        // A toggles the feeder servo, which keeps spinning to move balls up to the launcher motor.
        boolean aPressed = gamepad1.a && !lastA;
        lastA = gamepad1.a;

        if (aPressed) {
            feederOn = !feederOn;
        }

        if (feederServo != null) {
            feederServo.setPower(feederOn ? FEEDER_POWER : 0.0);
        }

        // B toggles the launcher motor on and off.
        boolean bPressed = gamepad1.b && !lastB;
        lastB = gamepad1.b;

        if (bPressed) {
            launcherOn = !launcherOn;
        }

        if (launcherMotor != null) {
            launcherMotor.setPower(launcherOn ? LAUNCHER_MOTOR_POWER : 0.0);
        }
    }

    private void handleArm() {
        // Left stick Y drives the elbow motor. BRAKE holds the arm when the stick is released.
        if (armElbowMotor != null) {
            double elbowStick = -gamepad2.left_stick_y;
            double elbowPower = Math.abs(elbowStick) > JOYSTICK_DEADZONE
                    ? elbowStick * ARM_ELBOW_POWER
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

    private void updateTelemetry() {
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Launcher Motor", launcherOn ? "ON" : "OFF");

        telemetry.addData("Feeder", feederOn ? "ON" : "OFF");

        if (armElbowMotor != null) {
            telemetry.addData("Elbow Power", "%.2f", armElbowMotor.getPower());
        }
        if (armClawServo != null) {
            telemetry.addData("Claw Commanded Position", armClawServo.getPosition());
        }

        // No telemetry.update() here: OpMode sends telemetry automatically after each loop()
    }
}
