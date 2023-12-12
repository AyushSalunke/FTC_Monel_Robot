package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class MonelBot2 extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    ElapsedTime inputTimer;

    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public static double
            armServoPos=0.63, wristServoPos=0.28, deliveryServoPos=0.5;
    public static double levelOne = 0, levelTwo = 300, levelThree = 450;
    public static double
            gripperServoPos, intakeArmServoPos, intakeWristServoPos, crankServoPos, position2, position;
    boolean
            armToggle = false, deliveryToggleOne = false, deliveryToggleTwo = false, intakeToggle = false, crankToggle = false, driveToggle = false;

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRIP,
        INTAKE_RETRACT,
        INTAKE_INPUT,
        INTAKE_FINAL
    };
    IntakeState inputState = IntakeState.INTAKE_START;
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

        inputTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        while (opModeInInit()){
            Arm.SetArmPosition(0.15,0.73);
            Intake.crankServo.setPosition(0.7);
            Intake.intakeArmServo.setPosition(0.5);
            Intake.intakeWristServo.setPosition(0.65);
            Drone.initialPos();
            Hanger.hangerServo.setPosition(0.3);
            Intake.gripperServo.setPosition(1);
            inputTimer.reset();
        }

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);

            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
            );
            drive.update();
            telemetry.addData("heading", poseEstimate.getHeading());
            //--------------------------------------------------------------------------------------

            switch (inputState){
                case INTAKE_START:
                    //waiting for input
                    if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                        Intake.intakeArmServo.setPosition(0.4); Intake.intakeWristServo.setPosition(0.45);
                        Intake.IntakePixel(1);
                        Arm.armServo.setPosition(0.3);Arm.wristServo.setPosition(0.735);
                        Arm.DropPixel(1);
                        inputTimer.reset();
                        inputState = IntakeState.INTAKE_EXTEND;
                    }
                    break;
                case INTAKE_EXTEND:
                    Intake.CrankPosition(0.69);
                    if (inputTimer.milliseconds() >= 200){
                        inputState = IntakeState.INTAKE_GRIP;
                    }
                    break;
                case INTAKE_GRIP:
                    if (beamBreaker.getState()){
                        Intake.IntakePixel(0.75);
                        inputTimer.reset();
                        inputState = IntakeState.INTAKE_RETRACT;
                    }
                    break;
                case INTAKE_RETRACT:
                    Intake.CrankPosition(0.375);
                    if (inputTimer.milliseconds() >= 300){
                        inputState = IntakeState.INTAKE_INPUT;
                        inputTimer.reset();
                    }
                    break;
                case INTAKE_INPUT:
                    if (inputTimer.milliseconds() >= 100){
                        Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.4);
                        if(inputTimer.milliseconds() >= 200){
                            Intake.intakeArmServo.setPosition(0.7);
                            if(inputTimer.milliseconds() >= 600){
                                Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.crankServo.setPosition(0.7);
                                inputState = IntakeState.INTAKE_FINAL;
                                inputTimer.reset();
                            }
                        }
                    }
                    break;
                case INTAKE_FINAL:
                    if (inputTimer.milliseconds() >= 200){
                        Arm.wristServo.setPosition(0.735);Arm.armServo.setPosition(0.15);
                        if (inputTimer.milliseconds() >= 400){
                            Arm.DropPixel(0.45);
                            Arm.armServo.setPosition(0);
                            slider.extendTo(-10, 0.8);
                            if (inputTimer.milliseconds() >= 600){
                                slider.extendTo(0, 0.8);Arm.armServo.setPosition(0.15);
                                inputState = IntakeState.INTAKE_START;
                            }
                        }
                    }
                    break;
            }
            if(currentGamepad1.left_trigger > 0.3 && !(previousGamepad1.left_trigger>0.3) && inputState!=IntakeState.INTAKE_START){
                inputState = IntakeState.INTAKE_START;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                //outtake pixel and bring out pixel intake
                TrajectorySequence OuttakePixel = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{Arm.DropPixel(0.45);})
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
                        .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.65);})
                        .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.65);}) //arm->0.4 for grd
                        .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Arm.armServo.setPosition(0.5);Arm.wristServo.setPosition(0.1);})
                        .waitSeconds(1)
                        .build();

                drive.followTrajectorySequenceAsync(OuttakePixel);
                drive.update();
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                //drop 1st pixel
                deliveryServoPos = 0.75;
                Arm.DropPixel(deliveryServoPos);
            }
            if(currentGamepad1.a && !previousGamepad1.a){
                //drop 2nd pixel
                deliveryServoPos = 1;
                Arm.DropPixel(deliveryServoPos);
                TrajectorySequence DropPixel = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Arm.DropPixel(1);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.73);Arm.armServo.setPosition(0.15);})
                        .addTemporalMarker(()->{Slider.DecreaseExtension(levelOne);})
                        .waitSeconds(0.1)
                        .build();
                drive.followTrajectorySequenceAsync(DropPixel);
                drive.update();
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                telemetry.addLine("DPad_UP_Pressed");
                Slider.IncreaseExtension(levelTwo);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                telemetry.addLine("DPad_DOWN_Pressed");
                Slider.DecreaseExtension(levelOne);
            }
            if (currentGamepad1.y && !previousGamepad1.y){
                Hanger.ExtendHanger();
            }
            if (currentGamepad1.x && !previousGamepad1.x){
                Drone.shootDrone();
            }
            if (currentGamepad1.dpad_right){
                Hanger.LiftRobot();
            }
            if (currentGamepad1.dpad_left){
                Hanger.PutDownRobot();
            }
            if(currentGamepad1.right_trigger>0.5){
                THROTTLE = 0.3;
                HEADING = 0.3;
                TURN = 0.3;
//                driveToggle = !driveToggle;
            }
            else {
                THROTTLE = 1;
                HEADING = 1;
                TURN = 1;
            }

            telemetry.addData("Beam Breaker State:", beamBreaker.getState());
            telemetry.addData("SliderMotorOne tick count", Slider.sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", Slider.sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", Slider.sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", Slider.sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("HangerMotor tick count", Hanger.hangerMotor.getCurrentPosition());
            telemetry.addData("Hanger Current", Hanger.hangerMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("gripperServo", Intake.gripperServo.getPosition());
            telemetry.addData("intakeWristServo", Intake.intakeWristServo.getPosition());
            telemetry.addData("intakeArmServo", Intake.intakeArmServo.getPosition());
            telemetry.addData("crankServo", Intake.crankServo.getPosition());
            telemetry.addData("armServo", Arm.armServo.getPosition());
            telemetry.addData("wristServo", Arm.wristServo.getPosition());
            telemetry.addData("deliveryServo", Arm.deliveryServo.getPosition());
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.update();
            drive.update();
        }
    }
}
