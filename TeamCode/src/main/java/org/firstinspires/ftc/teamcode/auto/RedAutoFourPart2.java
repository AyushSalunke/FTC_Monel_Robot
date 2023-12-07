package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RedAuto4Part2")
@Config
public class RedAutoFourPart2 extends LinearOpMode {
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
        Arm.DropPixel(0.75);
        Intake.CrankPosition(0.67);

        Pose2d  startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})
                //backdrop
                .lineToConstantHeading(new Vector2d(30 , -36))
                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{Arm.wristServo.setPosition(0.1);Arm.armServo.setPosition(0.545);})
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Arm.DropPixel(1);})
                .waitSeconds(0.2)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})
                .splineToConstantHeading(new Vector2d(18,-10), -Math.PI)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-34,-10), -Math.PI)
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.252);}) //0.633-0.2515 //arm->0.64
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.265);}) //0.645-0.2595
                .waitSeconds(0.2)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.645);Intake.CrankPosition(0.7);})
                .resetConstraints()
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-10),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.8);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);})
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->{Arm.wristServo.setPosition(0.73);})
                .UNSTABLE_addTemporalMarkerOffset(1.05, ()->{Arm.armServo.setPosition(0.15);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-10),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.55);})
                .waitSeconds(0.3) //0.6
                .addTemporalMarker(()->{slider.extendTo(230, 0.8);})
                .addTemporalMarker(()->{Arm.DropPixel(0.75);})
                .waitSeconds(0.4) //0.8
//                .lineToConstantHeading(new Vector2d(47.3, -35))
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.545);})
                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{Arm.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.8);})
                .waitSeconds(0.05)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{Intake.intakeArmServo.setPosition(0.4);}) //0.35
                .UNSTABLE_addTemporalMarkerOffset(0.80,()->{Intake.intakeWristServo.setPosition(0.375);Intake.intakeArmServo.setPosition(0.513);})//0.375-0.513//arm->0.52 //0.50
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})//0.2
                .setReversed(false)

                //pixel intake // round 2------------------------------------------------------------
                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(54))
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(this::telem)
//                .UNSTABLE_addTemporalMarkerOffset(-0.7,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);}) //0.0
//                .UNSTABLE_addTemporalMarkerOffset(-0.35,()->{Intake.intakeArmServo.setPosition(0.4);}) //0.35
//                .UNSTABLE_addTemporalMarkerOffset(-0.20,()->{Intake.intakeWristServo.setPosition(0.380);Intake.intakeArmServo.setPosition(0.515);})//0.380-0.513//arm->0.52 //0.50
//                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})//0.2

                .splineToConstantHeading(new Vector2d(-34, -12.5), -Math.PI)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
//                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                .lineToConstantHeading(new Vector2d(-51.7,-12.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.2) //0.3
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.525);Intake.intakeWristServo.setPosition(0.4);})
                .waitSeconds(0.1)
                .setReversed(true)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.525);Intake.intakeWristServo.setPosition(0.4);Intake.CrankPosition(0.7);})
                .resetConstraints()
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-10),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.45,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{Intake.intakeArmServo.setPosition(0.8);}) //0.3
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);}) //0.45
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{Arm.wristServo.setPosition(0.73);}) //1.00
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{Arm.armServo.setPosition(0.15);}) //1.10
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-10),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.55);})
                .waitSeconds(0.3) //0.6
                .addTemporalMarker(()->{slider.extendTo(200, 0.9);})
                .addTemporalMarker(()->{Arm.DropPixel(0.75);})
                .waitSeconds(0.4) //0.8
//                .lineToConstantHeading(new Vector2d(47.5, -30))
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.545);})
                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{Arm.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.9);})
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{Intake.intakeArmServo.setPosition(0.4);}) //0.35
                .UNSTABLE_addTemporalMarkerOffset(0.80,()->{Intake.intakeWristServo.setPosition(0.2515);Intake.intakeArmServo.setPosition(0.633);})//0.633-0.2515//arm->0.64 //0.50
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);}) //0.2
                .setReversed(false)

                //pixel intake // round 3-----------------------------------------------------------
                .splineToConstantHeading(new Vector2d(18,-8),-Math.PI)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(54))
//                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(this::telem)
//                .UNSTABLE_addTemporalMarkerOffset(-0.7,()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);}) //0.0
//                .UNSTABLE_addTemporalMarkerOffset(-0.35,()->{Intake.intakeArmServo.setPosition(0.4);}) //0.35
//                .UNSTABLE_addTemporalMarkerOffset(-0.20,()->{Intake.intakeWristServo.setPosition(0.252);Intake.intakeArmServo.setPosition(0.636);})//0.633-0.2515//arm->0.64 //0.50
//                .UNSTABLE_addTemporalMarkerOffset(-0.20,()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);}) //0.2

                .splineToConstantHeading(new Vector2d(-31,-8),-Math.PI)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-24.2),-Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Arm.armServo.setPosition(0.30);Arm.wristServo.setPosition(0.73);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.2) //0.3
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.2595);})
                .waitSeconds(0.05)
                .setReversed(true)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.2595);Intake.CrankPosition(0.7);})
                .resetConstraints()
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-31,-8),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.45,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{Intake.intakeArmServo.setPosition(0.8);}) //0.3
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);}) //0.45
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{Arm.wristServo.setPosition(0.73);}) //1.00
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->{Arm.armServo.setPosition(0.15);}) //1.10
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-8),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Arm.DropPixel(0.45);Arm.armServo.setPosition(0);slider.extendTo(-20, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.55);})
                .waitSeconds(0.3)//0.6
                .addTemporalMarker(()->{slider.extendTo(250, 1);})
                .addTemporalMarker(()->{Arm.DropPixel(0.75);})
                .waitSeconds(0.4)//0.8
//                .lineToConstantHeading(new Vector2d(47.3, -30))
                .addTemporalMarker(()->{Arm.wristServo.setPosition(0.05);Arm.armServo.setPosition(0.545);})
                .waitSeconds(0.2)//0.4
                .addTemporalMarker(()->{Arm.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})
                .resetConstraints()
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
