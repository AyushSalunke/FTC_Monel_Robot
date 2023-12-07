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

@Autonomous(name="RedAuto2")
@Config
public class RedAutoTwo extends LinearOpMode {
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
        Intake.SetArmPosition(0.4,0.65);
        Intake.IntakePixel(0.75);
        Arm.DropPixel(0.5);
        Intake.CrankPosition(0.67);

        Pose2d  startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                //backdrop
                .lineToConstantHeading(new Vector2d(30 , -36))
                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.545);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(46.5,-30), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Arm.DropPixel(1);})
                .waitSeconds(0.55)
                .addTemporalMarker(this::telem)
                .setReversed(false)

                //pixel intake // round 1
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-29,-12), -Math.PI)
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{Intake.intakeArmServo.setPosition(0.633);Intake.intakeWristServo.setPosition(0.2515);}) //arm->0.64
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .splineToConstantHeading(new Vector2d(-51,-12), -Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.3)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.64);Intake.CrankPosition(0.7);})
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.8);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);})
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->{Arm.wristServo.setPosition(0.73);})
                .UNSTABLE_addTemporalMarkerOffset(1.05, ()->{Arm.armServo.setPosition(0.15);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(47,-38),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.55);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Arm.DropPixel(0.70);})
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(47.3, -35))
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.545);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.DropPixel(1);})
                .setReversed(false)

                //pixel intake // round 2------------------------------------------------------------
                .splineToConstantHeading(new Vector2d(18,-5),-Math.PI)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);})
                .UNSTABLE_addTemporalMarkerOffset(0.35,()->{Intake.intakeArmServo.setPosition(0.4);})
                .UNSTABLE_addTemporalMarkerOffset(0.50,()->{Intake.intakeWristServo.setPosition(0.375);Intake.intakeArmServo.setPosition(0.513);})//arm->0.52
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})

                .splineToConstantHeading(new Vector2d(-29,-12),-Math.PI)
                .splineToConstantHeading(new Vector2d(-31, -12), -Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .addTemporalMarker(this::telem)
                .lineToConstantHeading(new Vector2d(-51.7,-12))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.3)
                .setReversed(true)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.52);Intake.CrankPosition(0.7);})
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.8);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);})
                .UNSTABLE_addTemporalMarkerOffset(1.00,()->{Arm.wristServo.setPosition(0.73);})
                .UNSTABLE_addTemporalMarkerOffset(1.10, ()->{Arm.armServo.setPosition(0.15);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(47,-33),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.55);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Arm.DropPixel(0.70);})
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(47.3, -30))
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.545);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.DropPixel(1);})
                .setReversed(false)
                .waitSeconds(200)
                .build();


        waitForStart();

        drive.followTrajectorySequence(first);
        while (opModeIsActive()) {
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));


            drive.update();
            telemetry.update();
        }
    }
    public void telem(){
        telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
        telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
        telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
        telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
        telemetry.update();
    }
}
