package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ManualDrive", group = "Linear OpMode")
public class ManualDrive extends LinearOpMode {

    // Wheel drive
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    // Claw servo (servo0)
    private static final int    CYCLE_MS0  = 50;
    private static final double MAX_POS0   = 0.50;
    private static final double MIN_POS0   = 0.65;
    private Servo  servo0;
    private double position0   = MAX_POS0;

    // Arm (viper slide)
    private DcMotor armMotor   = null;

    // Wrist joint servo (servo1)
    private static final int    CYCLE_MS1      = 50;
    private static final double JOINT_MAX_POS1 = 0.00;
    private static final double JOINT_MIN_POS1 = 0.30;
    private Servo  servo1;
    private double position1   = 0.0;

    // Plane launcher
    private static final int    CYCLE_MS_PLANE = 20;
    private static final double PLANE_MAX      = 0.50;
    private static final double PLANE_MIN      = 0.00;
    private Servo  planeServo;
    private double planePosition = PLANE_MIN;

    // Hanging
    private DcMotor hangMotor = null;
    private Servo   hangServo = null;

    // Intake
    private DcMotor intake    = null;

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run ManualDrive");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive();
            useServo();
            moveArm();
            moveJoint();
            shootPlane();
            hang();
            intake();
            addTelemetry();
            telemetry.update();
        }
    }

    // ==================== Drive ====================

    public void drive() {
        double max;

        double axial   = -gamepad1.left_stick_y;  // forward/back
        double lateral =  gamepad1.left_stick_x;  // strafe
        double yaw     =  gamepad1.right_stick_x; // rotate

        double leftFrontPower  = (axial + lateral + yaw) * 0.75;
        double rightFrontPower = (axial - lateral - yaw) * 0.75;
        double leftBackPower   = (axial - lateral + yaw) * 0.75;
        double rightBackPower  = (axial + lateral - yaw) * 0.75;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // D‑pad override moves
        if (gamepad1.dpad_up) {
            setDriveAll(0.25);
        } else if (gamepad1.dpad_down) {
            setDriveAll(-0.25);
        } else if (gamepad1.dpad_left) {
            leftFrontDrive.setPower(-0.75);
            rightFrontDrive.setPower(0.75);
            leftBackDrive.setPower(0.75);
            rightBackDrive.setPower(-0.75);
        } else if (gamepad1.dpad_right) {
            leftFrontDrive.setPower(0.75);
            rightFrontDrive.setPower(-0.75);
            leftBackDrive.setPower(-0.75);
            rightBackDrive.setPower(0.75);
        }
    }

    private void setDriveAll(double p) {
        leftFrontDrive.setPower(p);
        rightFrontDrive.setPower(p);
        leftBackDrive.setPower(p);
        rightBackDrive.setPower(p);
    }

    // ==================== Claw servo (servo0) ====================

    public void useServo() {
        // NOTE: original logic used gamepad2.b in both branches; preserved here
        if (gamepad2.b && position0 != MAX_POS0) {
            // close
            position0 = MAX_POS0;
        } else if (gamepad2.b) {
            // open
            position0 = MIN_POS0;
        }

        servo0.setPosition(position0);
        sleep(CYCLE_MS0);
        idle();
    }

    // ==================== Viper slide (armMotor) ====================

    public void moveArm() {
        if (gamepad2.right_bumper) {
            armMotor.setPower(0.5);
        } else if (gamepad2.right_trigger > 0) {
            armMotor.setPower(-0.5);
        } else {
            armMotor.setPower(0.0);
        }
    }

    // ==================== Hanging ====================

    public void hang() {
        if (gamepad2.dpad_up) {
            hangMotor.setPower(1.0);
        } else if (gamepad2.dpad_down) {
            hangMotor.setPower(-1.0);
        } else {
            hangMotor.setPower(0.0);
        }

        if (gamepad2.dpad_right) {
            hangServo.setPosition(1.0);
        } else if (gamepad2.dpad_left) {
            hangServo.setPosition(0.0);
        }
    }

    // ==================== Wrist joint (servo1) ====================

    public void moveJoint() {
        if (gamepad2.a) {
            if (gamepad2.y && position1 != JOINT_MAX_POS1) {
                position1 = JOINT_MAX_POS1;
            } else if (gamepad2.y) {
                position1 = JOINT_MIN_POS1;
            }

            servo1.setPosition(position1);
            sleep(CYCLE_MS1);
            idle();
        }
    }

    // ==================== Plane launcher ====================

    public void shootPlane() {
        if (gamepad1.a && planePosition != PLANE_MAX) {
            planePosition = PLANE_MAX;
        } else if (gamepad1.a) {
            planePosition = PLANE_MIN;
        }

        planeServo.setPosition(planePosition);
        sleep(CYCLE_MS_PLANE);
        idle();
    }

    // ==================== Intake ====================

    public void intake() {
        if (gamepad2.left_trigger > 0) {
            intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    // ==================== Telemetry ====================

    public void addTelemetry() {
        if (position1 == JOINT_MAX_POS1) {
            telemetry.addData("Joint Position", "DOWN");
        } else {
            telemetry.addData("Joint Position", "UP");
        }

        if (position0 == MAX_POS0) {
            telemetry.addData("Claw Position", "CLOSED");
        } else {
            telemetry.addData("Claw Position", "OPEN");
        }

        telemetry.addData("Plane Position", planePosition);
        telemetry.addData("Hang Motor Power", hangMotor.getPower());
    }

    // ==================== Initialization ====================

    public void initializeRobot() {
        // Drive motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Claw servo
        servo0 = hardwareMap.get(Servo.class, "clawServo");

        // Arm motor
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wrist servo
        servo1 = hardwareMap.get(Servo.class, "wristServo");
        servo1.setDirection(Servo.Direction.FORWARD);

        // Plane servo
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setDirection(Servo.Direction.FORWARD);

        // Hanging
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        hangServo = hardwareMap.get(Servo.class, "hangServo");

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
    }
}
