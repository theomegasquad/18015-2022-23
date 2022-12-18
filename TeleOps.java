package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOps", group = "")
public class TeleOps extends LinearOpMode {

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor linearSlide1;
    public DcMotor linearSlide2;
    //private DcMotor wheel;


    //private DcMotor intake;
    //public DcMotor carousel;
    ////public DcMotor shooter;
    //public DcMotor wobble;
    DigitalChannel digitalTouch;
    DigitalChannel digitalTouch2;
    public TouchSensor touch1;
    //TouchSensor touch2;


    public CRServo servo1;
    public CRServo servo2;

    //private DistanceSensor dist;


    private final double robotSpeedCarousel = -0.75;
    private final float robotSpeedRamp = 0.36f;
    private final float robotSpeedLowGoal = 0.28f;
    private final float robotSpeedClaw = 0.15f;
    private final float robotSpeedArm = 0.14f;
    public static double distance = 0.0;

    static double SLIDEPOWER1 = 0.6;
    static double SLIDEPOWER2 = 0.68;


    private static final double LIFT_SYNC_KP = 0.07;               //this value needs to be tuned
    private static final double LIFT_POSITION_TOLERANCE = 25; //this value needs to be tuned

    double  MIN_POSITION = 0, MAX_POSITION = 1;



    void synchronizeSlideHighJunction(int slideTargetPosition, boolean encoderReset, boolean isGoingUp) {
        int slide1TargetPosition = 0;
        int slide2TargetPosition = 0;
        if (isGoingUp) {
            //Always reset encoders before going up
            //linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide1TargetPosition = slideTargetPosition;
            slide2TargetPosition = -slideTargetPosition+20;
            linearSlide1.setTargetPosition(slide1TargetPosition);
            linearSlide2.setTargetPosition(slide2TargetPosition);

            SLIDEPOWER1 = 0.3;
            SLIDEPOWER2 = 0.3;
        }
        else{
            slide1TargetPosition = 0;
            slide2TargetPosition = 0;
            linearSlide1.setTargetPosition(slide1TargetPosition);
            linearSlide2.setTargetPosition(slide2TargetPosition);

            SLIDEPOWER1 = 0.13;
            SLIDEPOWER2 = 0.03;
        }

        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean isOnTarget = false;
        while (!isOnTarget)
        {

            if (gamepad1.left_stick_y != 0) {
                leftFront.setPower(-gamepad1.left_stick_y);
                leftBack.setPower(-gamepad1.left_stick_y);
                rightFront.setPower(gamepad1.left_stick_y);
                rightBack.setPower(gamepad1.left_stick_y);
                telemetry.addData("Moving", "%f", gamepad1.left_stick_y);
                telemetry.update();
            }
            else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }

            //Dpad right makes robot shuffle left
            if (gamepad1.dpad_left) {
                leftFront.setPower(1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(1);
                telemetry.addData("Shuffling", "left");
                telemetry.update();
            }

            //Dpad left makes robot shuffle right
            else if (gamepad1.dpad_right) {
                leftFront.setPower(-1);
                leftBack.setPower(1);
                rightFront.setPower(1);
                rightBack.setPower(-1);
                telemetry.addData("Shuffling", "right");
                telemetry.update();
            }
            else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            double differentiatePower = Math.abs (
                    Math.abs(linearSlide2.getCurrentPosition())  -
                            Math.abs(linearSlide1.getCurrentPosition())
            ) * LIFT_SYNC_KP;


            linearSlide1.setPower(Range.clip(SLIDEPOWER1 + differentiatePower, -1, 1));
            linearSlide2.setPower(Range.clip(SLIDEPOWER2 - differentiatePower, -1, 1));
            isOnTarget = Math.abs(slide1TargetPosition - linearSlide1.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                    Math.abs(slide2TargetPosition - linearSlide2.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;


            telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
            telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
            telemetry.addData("differentiatePower power: ", String.format("%f in", differentiatePower));
            telemetry.addData("Is on target: ", String.format("Linear 1: %d, Linear 2: %d", Math.abs(slide1TargetPosition - linearSlide1.getCurrentPosition()), Math.abs(slide2TargetPosition + linearSlide2.getCurrentPosition())));

            telemetry.update();
            idle();
        }

        if(!isGoingUp) {
            linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float speed = 0.0f;
        double turnspeed = 0.0f;
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "touch1");
        //digitalTouch2 = hardwareMap.get(DigitalChannel.class, "touch2");
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

        waitForStart();
        if (opModeIsActive()) {
            speed = 0.0f;
            leftFront.setPower(speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
            //wheel.setPower(-speed);
            // Put run blocks here.

            while (opModeIsActive()) {

                /*distance = dist.getDistance(DistanceUnit.CM);

                telemetry.addData("deviceName",dist.getDeviceName() );
                telemetry.addData("range", String.format("%.01f mm", dist.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", dist.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f m", dist.getDistance(DistanceUnit.METER)));
                telemetry.addData("range", String.format("%.01f in", dist.getDistance(DistanceUnit.INCH)));
                telemetry.addData("Distance", distance);

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();
*/
                //Right Trigger makes robot go forward

                if (gamepad1.left_stick_y != 0) {
                    leftFront.setPower(-gamepad1.left_stick_y);
                    leftBack.setPower(-gamepad1.left_stick_y);
                    rightFront.setPower(gamepad1.left_stick_y);
                    rightBack.setPower(gamepad1.left_stick_y);
                    telemetry.addData("Moving", "%f", gamepad1.left_stick_y);
                    telemetry.update();
                }
                else {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                }

                //Dpad right makes robot shuffle left
                if (gamepad1.dpad_left) {
                    leftFront.setPower(1);
                    leftBack.setPower(-1);
                    rightFront.setPower(-1);
                    rightBack.setPower(1);
                    telemetry.addData("Shuffling", "left");
                    telemetry.update();
                }

                //Dpad left makes robot shuffle right
                else if (gamepad1.dpad_right) {
                    leftFront.setPower(-1);
                    leftBack.setPower(1);
                    rightFront.setPower(1);
                    rightBack.setPower(-1);
                    telemetry.addData("Shuffling", "right");
                    telemetry.update();
                }
                else {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                }

                //Left Joystick makes robot turn
                if (gamepad1.left_stick_x != 0) {
                    turnspeed = gamepad1.left_stick_x;
                    leftFront.setPower(-turnspeed);
                    leftBack.setPower(-turnspeed);
                    rightFront.setPower(-turnspeed);
                    rightBack.setPower(-turnspeed);
                    telemetry.addData("Turning", "left or right");
                    telemetry.update();
                }
                else {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                }

                int slideTargetPosition;
                if (gamepad2.dpad_up) {
                    //linearSlide2.setPower(-0.41);
                    //linearSlide1.setPower(0.40);
                    slideTargetPosition = 3950; //Set actual value
                    synchronizeSlideHighJunction(slideTargetPosition, true, true
                    );
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }
                else if (gamepad2.dpad_down ){
                    //linearSlide2.setPower(0.12);
                    //linearSlide1.setPower(-0.30);

                    slideTargetPosition = 350;
                    synchronizeSlideHighJunction(slideTargetPosition,  false, false);
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }

                if (gamepad2.right_bumper) {
                    //linearSlide2.setPower(-0.41);
                    //linearSlide1.setPower(0.40);
                    slideTargetPosition = 2750; //Set actual value
                    synchronizeSlideHighJunction(slideTargetPosition, true, true
                    );
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }
                else if (gamepad2.left_bumper){
                    //linearSlide2.setPower(0.12);
                    //linearSlide1.setPower(-0.30);

                    slideTargetPosition = 350;
                    synchronizeSlideHighJunction(slideTargetPosition,  false, false);
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }

                if (gamepad2.right_trigger != 0.0) {
                    //linearSlide2.setPower(-0.41);
                    //linearSlide1.setPower(0.40);
                    slideTargetPosition = 1550; //Set actual value
                    synchronizeSlideHighJunction(slideTargetPosition, true, true
                    );
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }
                else if (gamepad2.left_trigger != 0.0){
                    //linearSlide2.setPower(0.12);
                    //linearSlide1.setPower(-0.30);

                    slideTargetPosition = 350;
                    synchronizeSlideHighJunction(slideTargetPosition,  false, false);
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));
                    telemetry.update();
                }

                else {
                    linearSlide1.setPower(0.05);
                    linearSlide2.setPower(-0.05);
                }

                if(gamepad2.x) {
                    telemetry.addData("Slide 1 Current Position: ", String.format("%d in", linearSlide1.getCurrentPosition()));
                    telemetry.addData("Slide 2 Current Position: ", String.format("%d in", linearSlide2.getCurrentPosition()));

                    telemetry.update();
                }
                //Opens the arm that holds the Cone
                if (gamepad2.a) {
                    servo1.setPower(-0.2);
                    servo2.setPower(0.2);

                }

                //Arms collides
                else if (gamepad2.b){
                    servo1.setPower(0.2);
                    servo2.setPower(-0.2);
                }
                //setting the power to bring the close position
                else {
                    servo1.setPower(0);
                    servo2.setPower(0);
                }
/*
                else if (gamepad2.right_bumper){
                    linearSlide1.setPower(-0.41);
                    linearSlide2.setPower(0.40);

                }
                else if (gamepad2.left_bumper){
                    linearSlide1.setPower(0.41);
                    linearSlide2.setPower(-0.40);
                }
                else {
                    linearSlide1.setPower(0);
                    linearSlide2.setPower(0);
                }

*/


                /*

                //Grip and Lift Wobble Goal

                telemetry.update();
                if (gamepad2.dpad_up && touch2.isPressed()) {
                    //claw.setPosition(50);
                    wobble.setPower(0);
                    telemetry.addData("Moving", "wobble goal up");
                    telemetry.update();
                }
                if (gamepad2.dpad_up && !(touch2.isPressed())) {
                    wobble.setPower(0.48);
                }

                //Release and Drop Wobble Goal
                if (gamepad2.dpad_down && touch1.isPressed()) {
                    //claw.setPosition(50);
                    wobble.setPower(0);
                    telemetry.addData("Moving", "wobble goal down");
                    telemetry.update();
                }
                if (gamepad2.dpad_down && !(touch1.isPressed())) {
                    wobble.setPower(-0.48);
                }
                //Suck in ring with right bumper
                if (gamepad2.right_bumper) {
                    carousel.setPower(1);
                    intake.setPower(-0.8);
                    telemetry.addData("Moving", "intake and carousel");
                    telemetry.update();
                }

                else if(gamepad2.left_trigger > 0.9){
                    carousel.setPower(1);
                    telemetry.addData("Moving", "carousel");
                    telemetry.update();
                }

                else if(gamepad2.right_trigger > 0.9){
                    intake.setPower(-0.6);
                    telemetry.addData("Moving", "intake");
                    telemetry.update();
                }
                else if ((gamepad2.right_trigger < 0.9 && gamepad2.left_trigger < 0.9 && gamepad2.right_bumper) == false) {
                    carousel.setPower(0);
                    intake.setPower(0);
                }




                //shooter.setPower(gamepad2.right_trigger);
                //Shoot with Left Bumper
                /*if (gamepad2.left_bumper) {
                    shooter.setPower(-0.75);
                    telemetry.addData("Moving", "shooter");
                    telemetry.update();
                }
                if (!gamepad2.left_bumper) {
                    shooter.setPower(0);
                }
                //Shoot with x from dot
                if (gamepad2.x) {
                    //shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooter.setPower(0.85);

                }






                /*if (!gamepad2.x) {
                    shooter.setPower(0);
                }*/
                /*
                //Shoot with y for power shots from line
                if (gamepad2.y) {
                    shooter.setPower(0.8);
                    telemetry.addData("Moving", "shooter for power shots");
                    telemetry.update();
                }
                if (!gamepad2.y && !gamepad2.x) {
                    shooter.setPower(0);
                }
                if (gamepad2.dpad_left) {
                    claw.setPosition(0);
                }
                if (gamepad2.dpad_right) {
                    claw.setPosition(1);
                }
                /*
                if (!gamepad2.dpad_left && !gamepad2.dpad_right) {
                    claw.setPosition(0.5);
                }
                */
                /*
                if (gamepad2.a) {
                    carousel.setPower(-1);
                    intake.setPower(0.8);
                    telemetry.addData("Reversing", "intake and carousel");
                    telemetry.update();
                }


                if (gamepad2.dpad_down && !(touch1.isPressed())) {
                    wobble.setPower(-0.48);
                    telemetry.addData("Moving", "touch1");
                    telemetry.update();
                } else if ((gamepad2.dpad_down && touch1.isPressed()) || (touch1.isPressed())) {
                    wobble.setPower(0);
                }
                if (gamepad2.dpad_up && !(touch2.isPressed())) {
                    wobble.setPower(0.48);
                    telemetry.addData("Moving", "touch2");
                    telemetry.update();
                } else if ((gamepad2.dpad_down && touch2.isPressed()) || (touch2.isPressed())) {
                    wobble.setPower(0);
                }
                if(gamepad2.dpad_right) {
                    claw.setPosition(MIN_POSITION);
                    telemetry.addData("Claw", claw.getPosition()+ " " + MIN_POSITION);
                    telemetry.update();
                }
                if(gamepad2.dpad_left) {
                    claw.setPosition(MAX_POSITION);
                    telemetry.addData("Claw", claw.getPosition() + " " + MAX_POSITION);
                    telemetry.update();
                }*/

            }
        }
    }
}
