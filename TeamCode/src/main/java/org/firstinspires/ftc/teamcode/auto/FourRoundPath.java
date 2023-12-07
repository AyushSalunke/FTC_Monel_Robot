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

@Autonomous(name="4Path")
@Config
public class FourRoundPath extends LinearOpMode {
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
        Arm.SetArmPosition(0.15, 0.73);
        Intake.SetArmPosition(0.4, 0.65);
        Intake.IntakePixel(0.75);
        Arm.DropPixel(0.5);
        Intake.CrankPosition(0.67);

        Pose2d startPose = new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                //backdrop
                .lineToConstantHeading(new Vector2d(30 , -36))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(18,-12.5),-Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-12.5),-Math.PI)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5),-Math.PI)
                .resetConstraints()
                .setReversed(true)
                //backdrop
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(-34,-12.5),0)
                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 2
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(18,-12.5),-Math.PI)
                .splineToConstantHeading(new Vector2d(-34, -12.5), -Math.PI)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                .resetConstraints()
                .setReversed(true)
                //backdrop
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(-34,-12.5),0)
                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 3
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                .splineToConstantHeading(new Vector2d(-31,-10),-Math.PI)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                .resetConstraints()
                .setReversed(true)
                //backdrop
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(-31,-12.5),0)
                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 4
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                .splineToConstantHeading(new Vector2d(-31,-10),-Math.PI)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                .resetConstraints()
                .setReversed(true)
                //backdrop
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(60))
                .splineToConstantHeading(new Vector2d(-31,-12.5),0)
                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                .resetConstraints()
                .setReversed(false)
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
}
