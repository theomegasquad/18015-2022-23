/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "FinalBlueLeft", group = "Concept")

public class FinalBlueLeftAutonomous extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "OmegaSquadBlue2.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "Blue1",
            "Blue2",
            "Blue3"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASHCLUD/////AAABmcwbozAzJkhfrIjyPMOyy8Fwtjdx8zHQSMfKRC7S0QO6TJ/jpW2i9eiFAc9SfzsNd7H+6vVUw1xN79pvd1WHyqBITw908iY7x4fMAy9eru4/3NH3XDll8zV/tqe4hrar8ELE3sZqq4vwCF67HtRcpAbjaSp0LVGKFrcJ7PLJw7GmgP9X1dSJ7ACsdBIRmxrMe6grj2QyGAulk3B1HYqOE+65NMz8NGzd37NtJqAT+OZj5FUdza+wuk1uCowJz2R5uBExhwiANAaZheXL6oduzAhjxKWQaG8tUYAbDXTP7+PeeCbCJzSR8L8qkITv5MyCheAupkzMTrQOmHA+Kz1JQJwkkTxKLgQOFzo1BSeEZ78c";

    public static String sideOfCone;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor linearSlide1;
    DcMotor linearSlide2;
    TouchSensor touch1;
    IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 312 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double APPROACH_SPEED = 0.25;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = -0.5;
    static final double     REVERSE_SPEED           =0.2;
    static final double     TURN_SPEED              = 0.2;
    static final int HIGH_JUNCTION_TARGET_POSITION = 3950; //Set actual value
    static final int MEDIUM_JUNCTION_TARGET_POSITION = 2760; //Set actual value
    static final int LOW_JUNCTION_TARGET_POSITION = 1560; //Set actual value

    public static double distance = 0.0;

    // public DcMotor liftArm;

    public CRServo servo1;
    public CRServo servo2;

    private final double robotSpeedCarousel = -0.75;
    private final float robotSpeedRamp = 0.36f;
    private final float robotSpeedLowGoal = 0.28f;
    private final float robotSpeedClaw = 0.15f;
    private final float robotSpeedArm = 0.14f;

    private final double SLIDE1_UP_POWER = 0.3;
    private final double SLIDE2_UP_POWER = 0.3;
    private final double SLIDE1_DOWN_POWER = 0.10;
    private final double SLIDE2_DOWN_POWER = 0.03;

    private final double SLIDE_HOLD_POWER = 0.05;

    private static final double LIFT_SYNC_KP = 0.07;               //this value needs to be tuned
    private static final double SPEED_RATE_KP = 0.0003;
    private static final double LIFT_POSITION_TOLERANCE = 25; //this value needs to be tuned

    double  MIN_POSITION = 0, MAX_POSITION = 1;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Autonomous Mode Status", "Ready to Run");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        linearSlide1 = hardwareMap.dcMotor.get("linearSlide1");
        linearSlide2 = hardwareMap.dcMotor.get("linearSlide2");

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        //dist = hardwareMap.get(DistanceSensor.class, "dist");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor);

        //touch1 = hardwareMap.get(TouchSensor.class, "touch1");
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Wheels Encoders", "Reset",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition());

        telemetry.addData("Slide Encoder", "1 & 2 ",
                linearSlide1.getCurrentPosition(), linearSlide2.getCurrentPosition());
        telemetry.update();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();

            while (runtime.seconds() < 2) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                            sideOfCone = recognition.getLabel();

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
        }

        telemetry.addData("Label Detected: ", sideOfCone);
        telemetry.update();

        //Hold the first cone
        servo1.setPower(0.5);
        servo2.setPower(-0.5);
        //Wait for the arm to close to grab the cone
        sleep(750);
        //Move the robot right while lighting the slide up
        synchronizeSlideAnyJunction(700, false, true);
        //Shuffle right
        encoderShuffle(0.3, -20, -20, 3);

        //Back against the wall, before moving forward
        encoderDrive(3, 3, 5.0);

        //Drive the robot to High Junction
        encoderDriveSynchronizeSlides(-47, -47, HIGH_JUNCTION_TARGET_POSITION, true, false, 7);
        //Turn counter-clockwise
        encoderDrive(-8, 8, 5.0);
        //Move close to High Junction
        encoderDrive(-5, -5, 5.0);
        //Release the pre-loaded cone
        servo1.setPower(-0.5);
        servo2.setPower(0.5);
        sleep(350);
        //Move away from High Junction
        encoderDrive(3, 3, 5.0);
        //Turn counter-clockwise - facing the sub-station
        encoderDrive(-9, 9, 5.0);
        //reposition
        //Reach sub-station to grab Cone
        encoderDriveSynchronizeSlides(-40, -40, 700, false, false, 7);
        //Grab the 2nd Cone
        servo1.setPower(0.2);
        servo2.setPower(-0.2);
        //Allow time to grab cone
        sleep(350);
        synchronizeSlideAnyJunction(1800, false, true);
        sleep(500);
        //Drive backward to High junction
        encoderDriveSynchronizeSlides(22, 22, 4025, true, false, 7);
        //Turn clock-wise - facing the High Junction
        encoderDrive(24, -24, 5.0);
        //drive forward to High Junction
        encoderDrive(-2, -2, 5.0);
        //Release the 2nd Cone
        servo1.setPower(-0.5);
        servo2.setPower(0.5);
        sleep(350);
        //Move away from high Junction
        encoderDrive(4.5, 4.5, 5.0);
        //Turn counter-clock wise
        encoderDrive(-6, 6, 5.0);
        //Move back to Signal sleeve position, while lowering the slide
        encoderDriveSynchronizeSlides(21, 21, 0, false, true, 5.0);

        if (sideOfCone.equals("Blue1")){
            encoderDrive(-17.5, 17.5, 5.0);
            encoderDrive(-22, -22, 5.0);
        }

        else if (sideOfCone.equals("Blue3")){
            encoderDrive(16, -16, 5.0);
            encoderDrive(-20, -20, 5.0);
        }

        else {
            telemetry.addData("Status", "Yipee!");
            telemetry.update();
        }

        //encoderDrive(-45, -45, 6);
        //encoderDrive(45, 45, 6);
        /*
        //shuffling to the side

        //Drive towards the tall junction
        encoderDrive(DRIVE_SPEED, -45, -45, 5.0);
        //Turning left
        encoderDrive(DRIVE_SPEED, -8, 8, 5.0);
        runtime.reset();
        synchronizeSlideAnyJunction(HIGH_JUNCTION_TARGET_POSITION, true, true);
        //Move closer to junction


        //Drop cone
        servo1.setPower(-0.75);
        servo2.setPower(-0.75);

        //sleep(3000);

        servo1.setPower(0);
        servo2.setPower(0);
        //Turning right
        encoderDrive(0.04, 3, 3, 5.0);

        synchronizeSlideAnyJunction(0,  false, false);

        //Drive to pick 2nd Code
        encoderDrive(DRIVE_SPEED, -9, 9, 5.0);

        //
        synchronizeSlideAnyJunction(750,  false, true);


        encoderDrive(DRIVE_SPEED, -40, -40, 5.0);

        encoderDrive(DRIVE_SPEED, 28, 28, 5.0);

        encoderDrive(DRIVE_SPEED, 24, -24, 5.0);

        synchronizeSlideAnyJunction(0,  false, false);
        //sleep(750);

        synchronizeSlideAnyJunction(HIGH_JUNCTION_TARGET_POSITION, true, true);

        encoderDrive(0.05, -5, -5, 5.0);
        */
    }

    /**
     * Initialize the Vuforia localization engine.
     */


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void synchronizeSlideAnyJunction(int slideTargetPosition, boolean resetEncoder, boolean isGoingUp) {
        int slide1TargetPosition = slideTargetPosition;
        int slide2TargetPosition = -slideTargetPosition;
        double slide1Power;
        double slide2Power;
        linearSlide1.setTargetPosition(slide1TargetPosition);
        linearSlide2.setTargetPosition(slide2TargetPosition);

        if (isGoingUp) {
            slide1Power = SLIDE1_UP_POWER;
            slide2Power = SLIDE2_UP_POWER;
        }
        else{
            slide1Power = SLIDE1_DOWN_POWER;
            slide2Power = SLIDE2_DOWN_POWER;
        }

        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean isOnTarget = false;
        while (!isOnTarget)
        {
            double differentiatePower = Math.abs (
                    Math.abs(linearSlide2.getCurrentPosition())  -
                            Math.abs(linearSlide1.getCurrentPosition())
            ) * LIFT_SYNC_KP;

            linearSlide1.setPower(Range.clip(slide1Power + differentiatePower, -1, 1));
            linearSlide2.setPower(Range.clip(slide2Power - differentiatePower, -1, 1));
            isOnTarget = Math.abs(slide1TargetPosition - linearSlide1.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                    Math.abs(slide2TargetPosition - linearSlide2.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;


            telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
            telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
            telemetry.addData("differentiatePower power: ", String.format("%f in", differentiatePower));
            telemetry.addData("Is on target: ", String.format("Linear 1: %d, Linear 2: %d", Math.abs(slide1TargetPosition - linearSlide1.getCurrentPosition()), Math.abs(slide2TargetPosition + linearSlide2.getCurrentPosition())));

            telemetry.update();
            idle();
        }

        if(slideTargetPosition == 0 || resetEncoder) {
            linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void encoderDriveSynchronizeSlides(double leftInches, double rightInches, int slideTargetPosition,
                                               boolean isGoingUp, boolean resetEncoder, double timeoutS) {
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newLeftBackTarget;
        int slide1TargetPosition = slideTargetPosition;
        int slide2TargetPosition = -slideTargetPosition;
        double slide1Power;
        double slide2Power;
        double speed;

        linearSlide1.setTargetPosition(slide1TargetPosition);
        linearSlide2.setTargetPosition(slide2TargetPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isGoingUp) {
            slide1Power = SLIDE1_UP_POWER;
            slide2Power = SLIDE2_UP_POWER;
        }
        else{
            slide1Power = SLIDE1_DOWN_POWER;
            slide2Power = SLIDE2_DOWN_POWER;
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget =leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) ||
                    (linearSlide1.isBusy() && linearSlide2.isBusy()))
            {

                int targetToCurrent = Math.abs(Math.abs(newLeftFrontTarget) - Math.abs(leftFront.getCurrentPosition())) ;
                speed = Range.clip(targetToCurrent * SPEED_RATE_KP + APPROACH_SPEED, -1.0, 1.0);

                double differentiatePower = Math.abs (Math.abs(linearSlide2.getCurrentPosition())  -
                                Math.abs(linearSlide1.getCurrentPosition())
                                ) * LIFT_SYNC_KP;

                leftFront.setPower(Math.abs(speed));
                rightFront.setPower(Math.abs(speed));
                leftBack.setPower(Math.abs(speed));
                rightBack.setPower(Math.abs(speed));

                linearSlide1.setPower(Range.clip(slide1Power + differentiatePower, -1, 1));
                linearSlide2.setPower(Range.clip(slide2Power - differentiatePower, -1, 1));

                // Display it for the driver.
                telemetry.addData("Drive Speed: %f", speed);
                telemetry.addData("Path1",  "Running to - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        newLeftFrontTarget,  newRightFrontTarget,
                        newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path1",  "Current pos - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        leftFront.getCurrentPosition(),  rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
                if(!linearSlide1.isBusy() || !linearSlide2.isBusy()) {
                    linearSlide1.setPower(0.05);
                    linearSlide2.setPower(0.05);
                }

                if(!leftFront.isBusy() || !leftBack.isBusy() || !rightFront.isBusy() || !rightBack.isBusy()) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                }
                idle();
            }

            // Stop all motion;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            linearSlide2.setPower(0.05);
            linearSlide1.setPower(0.05);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(slideTargetPosition == 0 || resetEncoder) {
                linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    private void encoderDrive(double leftInches, double rightInches,
                             double timeoutS) {
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newLeftBackTarget;
        double speed;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget =leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())) {

                int targetToCurrent = Math.abs(Math.abs(newLeftFrontTarget) - Math.abs(leftFront.getCurrentPosition())) ;
                speed = Range.clip(targetToCurrent * SPEED_RATE_KP + APPROACH_SPEED, -1.0, 1.0);

                leftFront.setPower(Math.abs(speed));
                rightFront.setPower(Math.abs(speed));
                leftBack.setPower(Math.abs(speed));
                rightBack.setPower(Math.abs(speed));

                // Display it for the driver.
                telemetry.addData("Drive Speed: %f", speed);
                telemetry.addData("Path1",  "Running to - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        newLeftFrontTarget,  newRightFrontTarget,
                        newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path1",  "Current pos - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        leftFront.getCurrentPosition(),  rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
                idle();
            }

            // Stop all motion;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    private void encoderShuffle(double speed,
                               double leftInches, double rightInches,
                               double timeoutS) {
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newLeftBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget =leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(-newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(-newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            leftFront.setPower((Math.abs(speed)));
            rightFront.setPower((Math.abs(speed)));
            leftBack.setPower((Math.abs(speed)));
            rightBack.setPower((Math.abs(speed)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Drive Speed: %f", speed);
                telemetry.addData("Path1",  "Running to - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        newLeftFrontTarget,  newRightFrontTarget,
                        newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path1",  "Current pos - LF: %7d RF :%7d, LB: %7d, RB: %7d",
                        leftFront.getCurrentPosition(),  rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    private void correctOrientationYawZero() {
        //Get the current angle
        boolean correctOrientation = true;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(correctOrientation) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yAngle =  orientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if((yAngle > 0.75) || (yAngle < -0.75))
                correctOrientation = true;
            else
                correctOrientation = false;

            if(yAngle > 0.75) {
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
                rightFront.setPower(-0.1);
                rightBack.setPower(-0.1);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Turn Right)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
            if(yAngle < -0.75) {
                leftFront.setPower(-0.1);
                leftBack.setPower(-0.1);
                rightFront.setPower(0.1);
                rightBack.setPower(0.1);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Turn Left)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

}

