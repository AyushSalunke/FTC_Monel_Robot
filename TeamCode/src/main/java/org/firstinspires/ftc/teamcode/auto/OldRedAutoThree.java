//final code for 1 auto round
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

@Autonomous(name="OldAuto3")
@Config
public class OldRedAutoThree extends LinearOpMode {
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

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30 , -36))

                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
                    Intake.IntakePixel(1);
                })

                // drop pixel on backdrop
                .splineToConstantHeading(new Vector2d(48,-30),0)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                //0.4
                .UNSTABLE_addTemporalMarkerOffset(1,()->{Arm.DropPixel(1);})

                .waitSeconds(0.5)//0.4
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })

                // on the way to pick up pixel //round 1
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
                .waitSeconds(0.45)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)//0.2


                //Unstable
                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(0.3)

                // pick pixel from arm
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                    slider.extendTo(-10, 1);
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
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.DropPixel(0.8);})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })

                .lineToConstantHeading(new Vector2d(48.5, -34))
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Arm.DropPixel(1);})

                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.73);
                })
                .waitSeconds(200)

                .build();


        waitForStart();
        while (opModeIsActive()) {
            drive.followTrajectorySequence(first);
            drive.update();
        }
    }
}
