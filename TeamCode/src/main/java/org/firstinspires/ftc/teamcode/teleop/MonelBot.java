package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "Robot Main")
@Config
public class MonelBot extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;


    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public static double
            armServoPos=0.63, wristServoPos=0.28, deliveryServoPos=0.5;
    public static double levelOne = 0, levelTwo = 300, levelThree = 450;
    public static double
            gripperServoPos, intakeArmServoPos, intakeWristServoPos, crankServoPos, position2, position;
    boolean
            armToggle = false, deliveryToggleOne = false, deliveryToggleTwo = false, intakeToggle = false, crankToggle = false, driveToggle = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        while (opModeInInit()){
            Arm.SetArmPosition(0.15,0.73);
            Intake.crankServo.setPosition(0.7);
            Intake.intakeArmServo.setPosition(0.4);
            Intake.intakeWristServo.setPosition(0.65);
            Drone.initialPos();
            Hanger.hangerServo.setPosition(0.3);
            Intake.gripperServo.setPosition(1);
        }

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
//                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
//            );
//            drive.update();
//            telemetry.addData("heading", poseEstimate.getHeading());
            //--------------------------------------------------------------------------------------
//
//            //Slider
//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                telemetry.addLine("DPad_UP_Pressed");
//                Slider.IncreaseExtension(levelTwo);
//            }
//            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                telemetry.addLine("DPad_DOWN_Pressed");
//                Slider.DecreaseExtension(levelOne);
//            }

            telemetry.addData("SliderMotorOne tick count", Slider.sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", Slider.sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", Slider.sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", Slider.sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            //--------------------------------------------------------------------------------------
//
//            //Arm
//            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
//                //Place Pixel
//                armServoPos = 0.5;
//                wristServoPos = 0.1;
//                Arm.SetArmPosition(armServoPos,wristServoPos);
//            }
//            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                //Pick Pixel
//                armServoPos = 0.15;
//                wristServoPos = 0.73;
//                Arm.SetArmPosition(armServoPos, wristServoPos);
//            }
//            if (currentGamepad2.back && !previousGamepad2.back){
//                Arm.SetArmPosition(armServoPos, wristServoPos);
//            }
            //--------------------------------------------------
//
//            //Delivery
//            if(currentGamepad1.a && !previousGamepad1.a){
//                deliveryServoPos = 1;
//                Arm.DropPixel(deliveryServoPos);
//            }
//            if (currentGamepad1.b && !previousGamepad1.b){
//                deliveryServoPos = 0.45;
//                Arm.DropPixel(deliveryServoPos);
//                sleep(200);
//                Arm.armServo.setPosition(0);
//                slider.extendTo(-5, 1);
//                sleep(200);
//                slider.extendTo(0, 1);
//            }
            //--------------------------------------------------------------------------------------
//
//            //Hanger
//            if (currentGamepad1.right_bumper){
//                Hanger.LiftRobot();
//            }
//            if (currentGamepad1.left_bumper){
//                Hanger.PutDownRobot();
//            }
            telemetry.addData("HangerMotor tick count", Hanger.hangerMotor.getCurrentPosition());
            telemetry.addData("Hanger Current", Hanger.hangerMotor.getCurrent(CurrentUnit.AMPS));
            //--------------------------------------------------------------------------------------
//
//            //Intake
//            if (currentGamepad1.y && !previousGamepad1.y){
//                //Intake Pixel Position
//
//                TrajectorySequence PickBottomPixel = drive.trajectorySequenceBuilder(startPose)
//
//                        .addTemporalMarker(()->{Intake.gripperServo.setPosition(0.75);})
//                        .waitSeconds(0.3)
//                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);})//Intake.intakeArmServo.setPosition(0.4);I
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
//                        .waitSeconds(0.5)//0.6
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.crankServo.setPosition(0.7);Intake.gripperServo.setPosition(0.75);})
//                        .build();
//                TrajectorySequence PickMiddlePixel = drive.trajectorySequenceBuilder(startPose)
//                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);})
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
//                        .waitSeconds(0.5)//0.6
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.crankServo.setPosition(0.7);})
//                        .build();
//                TrajectorySequence PickTopPixel = drive.trajectorySequenceBuilder(startPose)
//                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
//                        .waitSeconds(0.4)//0.2
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
//                        .waitSeconds(0.5)//0.5//0.6
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);Intake.crankServo.setPosition(0.7);})
//                        .build();
//                drive.followTrajectorySequence(PickBottomPixel);
//            }

//            if(currentGamepad1.x && !previousGamepad1.x) {
//                //Pick Pixel from ground Position
//
//                TrajectorySequence IntakeBottomPixel = drive.trajectorySequenceBuilder(startPose)
//                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
//                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
//                        .waitSeconds(0.05)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);})//Intake.intakeWristServo.setPosition(0.65);
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.45);})
//                        .build();
//
//                TrajectorySequence IntakeTopPixel = drive.trajectorySequenceBuilder(startPose)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.62);})//Intake.intakeWristServo.setPosition(0.65);
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.62);Intake.intakeWristServo.setPosition(0.258);})
//                        .build();
//                drive.followTrajectorySequence(IntakeBottomPixel);
//            }
            if (currentGamepad2.start && !previousGamepad2.start){
                Intake.SetArmPosition(intakeArmServoPos, intakeWristServoPos);
            }
            //--------------------------------------------------------------------------------------

//            Gripper
            if (currentGamepad1.x && !previousGamepad1.x){
                gripperServoPos = 0.75;
                Intake.IntakePixel(gripperServoPos);
            }
            if (currentGamepad1.b && !previousGamepad1.b){
                gripperServoPos = 1;
                Intake.IntakePixel(gripperServoPos);
            }
            //-----------------------------------------------------

//            //Crank
//            if (currentGamepad1.back && !previousGamepad1.back){
//                crankToggle = !crankToggle;
//            }
//            if (crankToggle){
//                telemetry.addLine("Crank Extended");
//                crankServoPos = 0.38;
//                Intake.CrankPosition(crankServoPos);
//            }
//            else {
//                telemetry.addLine("Crank Retracted");
//                crankServoPos = 0.67;
//                Intake.CrankPosition(crankServoPos);
//            }
            //--------------------------------------------------------------------------------------

            //Final TeleOp
            Pose2d poseEstimate2 = drive.getPoseEstimate();
            Vector2d input2 = new Vector2d(Math.pow(Range.clip(gamepad2.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(gamepad2.left_stick_x, -1, 1), 3)).rotated(-poseEstimate2.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input2.getX() * THROTTLE, input2.getY() * TURN, -gamepad2.right_stick_x * HEADING)
            );
            drive.update();
            telemetry.addData("heading", poseEstimate2.getHeading());
            //-----------------------------------------------------------------

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                //grip and intake pixel into robot
                TrajectorySequence IntakePixel = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4); Intake.intakeWristServo.setPosition(0.45);})
                        .waitSeconds(0.2)
                        .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.IntakePixel(0.75);})
//                        .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Arm.DropPixel(1);})
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Arm.armServo.setPosition(0.3);Arm.wristServo.setPosition(0.735);})
                        .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.4);})
                        .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.intakeArmServo.setPosition(0.7);})
                        .UNSTABLE_addTemporalMarkerOffset(0.6,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.crankServo.setPosition(0.7);})
                        .UNSTABLE_addTemporalMarkerOffset(0.8,()->{Arm.wristServo.setPosition(0.735);Arm.armServo.setPosition(0.15);})
                        .UNSTABLE_addTemporalMarkerOffset(1.0,()->{Arm.DropPixel(0.45);})
                        .UNSTABLE_addTemporalMarkerOffset(1.0,()->{Arm.armServo.setPosition(0);slider.extendTo(-10, 0.8);})
                        .UNSTABLE_addTemporalMarkerOffset(1.2,()->{slider.extendTo(0, 0.8);})
                        .waitSeconds(1.5)
                        .build();

                drive.followTrajectorySequenceAsync(IntakePixel);
                drive.update();
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                //outtake pixel and bring out pixel intake
                TrajectorySequence OuttakePixel = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{Arm.DropPixel(0.45);})
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
                        .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.65);})
                        .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.65);}) //arm->0.4 for grd
                        .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Arm.armServo.setPosition(0.5);Arm.wristServo.setPosition(0.1);})
                        .waitSeconds(1)
                        .build();

                drive.followTrajectorySequenceAsync(OuttakePixel);
                drive.update();
            }

            if(currentGamepad2.b && !previousGamepad2.b){
                //drop 1st pixel
                deliveryServoPos = 0.75;
                Arm.DropPixel(deliveryServoPos);
            }
            if(currentGamepad2.a && !previousGamepad2.a){
                //drop 2nd pixel
                deliveryServoPos = 1;
                Arm.DropPixel(deliveryServoPos);
                TrajectorySequence DropPixel = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Arm.DropPixel(1);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.73);Arm.armServo.setPosition(0.15);})
                        .waitSeconds(0.1)
                        .build();
                drive.followTrajectorySequenceAsync(DropPixel);
                drive.update();
            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                telemetry.addLine("DPad_UP_Pressed");
                Slider.IncreaseExtension(levelTwo);
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                telemetry.addLine("DPad_DOWN_Pressed");
                Slider.DecreaseExtension(levelOne);
            }
            if (currentGamepad2.y && !previousGamepad2.y){
                Hanger.ExtendHanger();
            }
            if (currentGamepad2.x && !previousGamepad2.x){
                Drone.shootDrone();
            }
            if (currentGamepad2.dpad_right){
                Hanger.LiftRobot();
            }
            if (currentGamepad2.dpad_left){
                Hanger.PutDownRobot();
            }
            if(currentGamepad2.right_trigger>0.5){
                THROTTLE = 0.4;
                HEADING = 0.4;
                TURN = 0.4;
//                driveToggle = !driveToggle;
            }
            else {
                THROTTLE = 1;
                HEADING = 1;
                TURN = 1;
            }
            if (currentGamepad2.left_trigger > 0.5 && !(previousGamepad2.left_trigger > 0.3)){
                crankToggle = !crankToggle;
            }
            if (crankToggle){
                telemetry.addLine("Crank Extended");
                crankServoPos = 0.38;
                Intake.CrankPosition(crankServoPos);
                Intake.intakeArmServo.setPosition(0.4);
                Intake.intakeWristServo.setPosition(0.5);

            }
            else {
                telemetry.addLine("Crank Retracted");
                crankServoPos = 0.67;
                Intake.CrankPosition(crankServoPos);
            }


//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                Intake.intakeArmServo.setPosition(Intake.intakeArmServo.getPosition() + 0.05);
//            }
//            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
//                Intake.intakeArmServo.setPosition(Intake.intakeArmServo.getPosition() - 0.05);
//            }
//            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//                Intake.intakeWristServo.setPosition(Intake.intakeWristServo.getPosition() + 0.05);
//            }
//            if (!currentGamepad1.dpad_left && previousGamepad1.dpad_left) {
//                Intake.intakeWristServo.setPosition(Intake.intakeWristServo.getPosition() - 0.05);
//            }
//            if (currentGamepad1.a && !previousGamepad1.a) {
//                Arm.armServo.setPosition(Arm.armServo.getPosition() + 0.05);
//            }
//            if (!currentGamepad1.y && previousGamepad1.y) {
//                Arm.armServo.setPosition(Arm.armServo.getPosition() - 0.05);
//            }
//            if (currentGamepad1.x && !previousGamepad1.x) {
//                Arm.wristServo.setPosition(Arm.wristServo.getPosition() + 0.05);
//            }
//            if (!currentGamepad1.b && previousGamepad1.b) {
//                Arm.wristServo.setPosition(Arm.wristServo.getPosition() - 0.05);
//            }
//            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
//                //Second Pixel Drop Position
//                deliveryToggleTwo = !deliveryToggleTwo;
//            }
//            if (deliveryToggleTwo){
//                deliveryServoPos = 1;
//                Arm.DropPixel(deliveryServoPos);
//            }
//            else {
//                //Pixel Intake Position
//                deliveryServoPos = 0.5;
//                deliveryToggleTwo = false;
//                Arm.DropPixel(deliveryServoPos);
//            }
//            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
//                intakeToggle = !intakeToggle;
//            }
//            if (intakeToggle){
//                gripperServoPos = 0.75;
//                Intake.IntakePixel(gripperServoPos);
//            }
//            else {
//                gripperServoPos = 1;
//                Intake.IntakePixel(gripperServoPos);
//            }

            telemetry.addData("gripperServo", Intake.gripperServo.getPosition());
            telemetry.addData("intakeWristServo", Intake.intakeWristServo.getPosition());
            telemetry.addData("intakeArmServo", Intake.intakeArmServo.getPosition());
            telemetry.addData("crankServo", Intake.crankServo.getPosition());
            telemetry.addData("armServo", Arm.armServo.getPosition());
            telemetry.addData("wristServo", Arm.wristServo.getPosition());
            telemetry.addData("deliveryServo", Arm.deliveryServo.getPosition());

            telemetry.update();
            drive.update();
        }
    }
}
