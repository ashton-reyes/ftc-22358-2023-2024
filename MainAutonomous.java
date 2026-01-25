package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;


/*
@Autonomous(name = "Blue Cam Auto Test Time", group = "Test")
*/
public class AutoTestBlueTime extends LinearOpMode {


    // Camera / TensorFlow
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/Red2.tflite";


    private static final String[] LABELS = {
        "Red"
    };


    /**
     * TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;


    /**
     * Vision portal instance.
     */
    private VisionPortal visionPortal;


    // Drive motors
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;


    // Intake
    private DcMotor intake = null;


    // Arm and servos
    private DcMotor armMotor = null;
    private Servo   clawServo;
    private Servo   wristServo;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        initTfod();
        initMotor();
        initIntake();
        initArm();


        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetryTfod();
        telemetry.update();


        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double xPos = telemetryTfod();
                telemetry.update();


                if (xPos >= 100 && xPos <= 400) {
                    // TODO: path for middle position
                } else if (xPos >= 380) {
                    // TODO: path for right/left position (depending on calibration)
                } else {
                    // Default path
                    forward(1.1);
                    sleep(1000);


                    outake(2);
                    sleep(1000);


                    backward(0.3);
                    sleep(1000);


                    turn(true);
                    backward(1.4);
                    sleep(1000);


                    dropPixel();


                    sideways(0.5);
                    sleep(1000);


                    forward(0.3);
                    sideways(1.5);
                }


                break;
            }
        }


        // Save CPU resources when camera is no longer needed.
        visionPortal.close();
    }


    // ======================= Vision / TFOD =======================


    private void initTfod() {
        // Create the TensorFlow processor.
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS)
            .build();


        // Create the vision portal.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.addProcessor(tfod);


        visionPortal = builder.build();


        // Confidence threshold for recognitions.
        tfod.setMinResultConfidence(0.75f);
    }


    private double telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());


        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2.0;


            telemetry.addData("", "");
            telemetry.addData(
                "Image",
                "%s (%.0f %% Conf.)",
                recognition.getLabel(),
                recognition.getConfidence() * 100.0
            );
            telemetry.addData("x Position", "%.0f", x);
            telemetry.addData(
                "- Size",
                "%.0f x %.0f",
                recognition.getWidth(),
                recognition.getHeight()
            );


            return x;
        }


        return -1;
    }


    // ======================= Hardware init =======================


    private void initMotor() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackMotor");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }


    private void initIntake() {
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
    }


    private void initArm() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);


        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(0.0);


        wristServo = hardwareMap.get(Servo.class, "wristServo");
        wristServo.setPosition(0.0);
    }


    // ======================= Motion helpers =======================


    public void sideways(double time) {
        ElapsedTime timer = new ElapsedTime();


        while (timer.seconds() <= time && opModeIsActive()) {
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
        }


        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        sleep(200);
    }


    public void forward(double time) {
        ElapsedTime timer = new ElapsedTime();


        while (timer.seconds() <= time && opModeIsActive()) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
        }


        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }


    public void backward(double time) {
        ElapsedTime timer = new ElapsedTime();


        while (timer.seconds() <= time && opModeIsActive()) {
            leftFrontDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);
        }


        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }


    public void outake(double time) {
        ElapsedTime timer = new ElapsedTime();


        while (timer.seconds() <= time && opModeIsActive()) {
            intake.setPower(-0.5);
        }


        intake.setPower(0.0);
    }


    // true = left, false = right
    public void turn(boolean dirLeft) {
        ElapsedTime timer = new ElapsedTime();


        if (dirLeft) {
            while (timer.seconds() <= 1.1 && opModeIsActive()) {
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(-0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(-0.5);
            }
        } else {
            while (timer.seconds() <= 1.0 && opModeIsActive()) {
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
            }
        }


        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }


    public void dropPixel() {
        ElapsedTime timer0 = new ElapsedTime();


        while (timer0.seconds() <= 1.5 && opModeIsActive()) {
            armMotor.setPower(-0.8);
        }
        armMotor.setPower(0.0);


        sleep(1000);
        wristServo.setPosition(0.5);
        sleep(1000);
        clawServo.setPosition(0.9);
        sleep(1000);
        wristServo.setPosition(0.0);
        sleep(1000);


        ElapsedTime timer1 = new ElapsedTime();
        while (timer1.seconds() <= 1.5 && opModeIsActive()) {
            armMotor.setPower(0.8);
        }
        armMotor.setPower(0.0);
    }
}



