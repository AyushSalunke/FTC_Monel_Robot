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

@Autonomous(name="RedAuto1")
@Config
public class RedAutoOne extends LinearOpMode {
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
                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.535);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Arm.DropPixel(1);})
                .waitSeconds(0.55)
                .addTemporalMarker(this::telem)
                .setReversed(false)

                //pixel intake // round 1
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})
                .splineToConstantHeading(new Vector2d(18,-10), -Math.PI)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-29,-12), -Math.PI)
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{Intake.intakeArmServo.setPosition(0.62);Intake.intakeWristServo.setPosition(0.270);})
                .UNSTABLE_addDisplacementMarkerOffset(-0.7, ()->{Intake.CrankPosition(0.38);})
                .splineToConstantHeading(new Vector2d(-50.2,-12), -Math.PI)
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.3)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->Intake.CrankPosition(0.7))
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-7),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.8);})
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);})
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->{Arm.wristServo.setPosition(0.73);})
                .UNSTABLE_addTemporalMarkerOffset(1.05, ()->{Arm.armServo.setPosition(0.15);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-7),0)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(48,-34),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(1)

                //place pixel on backdrop
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.535);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.515);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Arm.DropPixel(0.85);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.wristServo.setPosition(0.05);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.DropPixel(1);})
                .setReversed(false)

//                //pixel intake // round 2
//                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-49.5,-14),Math.PI)
//                .addTemporalMarker(this::telem)
//                .waitSeconds(1)
//                .setReversed(true)
//                //backdrop
//                .splineToConstantHeading(new Vector2d(-34,-7),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(18,-7),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(48,-34),0)
//                .addTemporalMarker(this::telem)
//                .waitSeconds(1)
//                .setReversed(false)
//
//                //pixel intake // round 3
//                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-49.5,-14),Math.PI)
//                .addTemporalMarker(this::telem)
//                .waitSeconds(1)
//                .setReversed(true)
//                //backdrop
//                .splineToConstantHeading(new Vector2d(-34,-7),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(18,-7),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(48,-34),0)
//                .addTemporalMarker(this::telem)
//                .waitSeconds(1)
//                .setReversed(false)
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
