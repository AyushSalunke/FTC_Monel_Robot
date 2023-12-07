package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="OldAuto4")
@Config
public class OldRedAutoFour extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        slider.extendToHome();
        Arm.SetArmPosition(0.15,0.73);
        Intake.SetArmPosition(0.4,0.5);
        Intake.IntakePixel(0.75);
        Arm.DropPixel(0.7);
        Intake.CrankPosition(0.67);

        Pose2d  startPose=new Pose2d(15, -64, -Math.PI);
        drive.setPoseEstimate(startPose);
        telemetry.clearAll();

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30 , -36))
                //.waitSeconds(0.5)

                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    Intake.IntakePixel(1);
                })
                .waitSeconds(0.5)

                // drop pixel on backdrop
                .splineToConstantHeading(new Vector2d(48,-30),0)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(1);})

                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })

                // on the way to pick up pixel // round 1
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-49.5,-14),Math.PI)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.73);
                })
                // open crank and drop gripper to pick pixel
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(0.62);
                    Intake.intakeWristServo.setPosition(0.268);
                    Intake.CrankPosition(0.38);})
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(1.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(1)

                // pick pixel from arm
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    slider.extendTo(-10, 1);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    Intake.IntakePixel(0.85);
                    slider.extendTo(0, 1);
                })

                // head to backdrop and drop delivery
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34,-7),0)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .splineToConstantHeading(new Vector2d(48,-38),0) //46
                .waitSeconds(1)

                // place pixel on backdrop
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(0.8);})
                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .lineToConstantHeading(new Vector2d(48.5, -34))
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(1);})

                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);})//Intake.intakeWristServo.setPosition(0.65);
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                // on the way to pick up pixel // round 2
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-49.5,-14),Math.PI)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.73);
                })
                // open crank and drop gripper to pick pixel
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(0.51);
                    Intake.intakeWristServo.setPosition(0.38);
                    Intake.CrankPosition(0.38);})
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(1.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(1)

                // pick pixel from arm
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    slider.extendTo(-10, 1);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    Intake.IntakePixel(0.85);
                    slider.extendTo(0, 1);
                })

                // head to backdrop and drop delivery
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34,-7),0)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .splineToConstantHeading(new Vector2d(48,-38),0) //46
                .waitSeconds(1)

                // place pixel on backdrop
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(0.8);})
                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .lineToConstantHeading(new Vector2d(48.5, -34))
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(1);})

                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);})//Intake.intakeWristServo.setPosition(0.65);
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                // on the way to pick up pixel // round 3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .splineToConstantHeading(new Vector2d(-49.5,-14),Math.PI)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.73);
                })
                // open crank and drop gripper to pick pixel
                .addTemporalMarker(()->{
                    Intake.CrankPosition(0.38);
                    Intake.intakeArmServo.setPosition(0.4);
                    Intake.intakeWristServo.setPosition(0.45);})
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(1.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(1)

                // pick pixel from arm
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    slider.extendTo(-10, 1);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    Intake.IntakePixel(0.85);
                    slider.extendTo(0, 1);
                })

                // head to backdrop and drop delivery
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34,-7),0)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .splineToConstantHeading(new Vector2d(48,-38),0) //46
                .waitSeconds(1)

                // place pixel on backdrop
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(0.8);})
                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .lineToConstantHeading(new Vector2d(48.5, -34))
                .waitSeconds(1)
                .addTemporalMarker(()->{Arm.DropPixel(1);})

                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(200)
                .build();


        waitForStart();

        drive.followTrajectorySequence(first);
        telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
        telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
        telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
        telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));

        while (opModeIsActive()) {
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));


            drive.update();
            telemetry.update();
        }
    }
}
