//final code for 1 auto round
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="REFAUTO2")
@Config
public class REFAUTO2 extends LinearOpMode {
    SampleMecanumDrive drive = null;

    double Runtime = 30;
    private ElapsedTime timer;

    //    private FtcDashboard dashboard = FtcDashboard.getInstance();
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private ElapsedTime runtime = new ElapsedTime();
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
        Arm.DropPixel(0.8);
        Intake.CrankPosition(0.67);

        Pose2d  startPose=new Pose2d(15, -64, -Math.PI);
        drive.setPoseEstimate(startPose);

//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30 , -36))
                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.05,()->{
                    Intake.IntakePixel(1);
                })
                // drop pixel on backdrop
                .splineToConstantHeading(new Vector2d(50 ,-30),0)

                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    Arm.DropPixel(1);
                })
                .waitSeconds(0.05)

                // on the way to pick up pixel //round 1
                .setReversed(false)
//
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-49,-13),Math.PI)

                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(0.62);
                    Intake.intakeWristServo.setPosition(0.268);
                    Intake.CrankPosition(0.38);})
                .waitSeconds(0.5) ///1
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.5)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    Arm.wristServo.setPosition(0.735);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.armServo.setPosition(0.15);
                })
                .setReversed(true)


////                .waitSeconds(5)
////                .waitSeconds(0.4)
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
//                    Intake.intakeWristServo.setPosition(0.65);
//                    Intake.intakeArmServo.setPosition(0.55);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(0.7);
//                })
//////                .waitSeconds(0.2)//0.6
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(1);
//                    Intake.intakeWristServo.setPosition(0.44);
//                })
////                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(-34,-10),0)

// head to backdrop and drop delivery

                .splineToConstantHeading(new Vector2d(18,-10),0)
//                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                    slider.extendTo(-20, 1);
                })
////                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{
                    Intake.IntakePixel(1);
                    slider.extendTo(0, 1);
                })

                .splineToConstantHeading(new Vector2d(49,-34),0) //50//////////////////

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })


                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.DropPixel(0.9);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.wristServo.setPosition(0.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.DropPixel(1);
                })









                .setReversed(false)
//
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.IntakePixel(0.75);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.intakeWristServo.setPosition(0.5);
                    Intake.intakeArmServo.setPosition(0.4);
                })



                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.IntakePixel(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-49,-13),Math.PI)

                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(0.49);
                    Intake.intakeWristServo.setPosition(0.40);
                    Intake.CrankPosition(0.38);})
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    Arm.wristServo.setPosition(0.735);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.armServo.setPosition(0.15);
                })
                .setReversed(true)


////                .waitSeconds(5)
////                .waitSeconds(0.4)
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
//                    Intake.intakeWristServo.setPosition(0.65);
//                    Intake.intakeArmServo.setPosition(0.55);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(0.7);
//                })
//////                .waitSeconds(0.2)//0.6
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(1);
//                    Intake.intakeWristServo.setPosition(0.44);
//                })
////                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(-34,-10),0)

// head to backdrop and drop delivery

                .splineToConstantHeading(new Vector2d(18,-10),0)
//                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                    slider.extendTo(-20, 1);
                })
////                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    Intake.IntakePixel(1);
                    slider.extendTo(0, 1);
                })

                .splineToConstantHeading(new Vector2d(50,-38),0) //46

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })


                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.DropPixel(0.9);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.wristServo.setPosition(0.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.DropPixel(1);
                })









//
//
                .setReversed(false)
//
                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.IntakePixel(0.75);
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.intakeWristServo.setPosition(0.5);
                    Intake.intakeArmServo.setPosition(0.4);
                })



                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.15);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-34,-7),-Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Intake.IntakePixel(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.armServo.setPosition(0.25);
                    Arm.wristServo.setPosition(0.735);
                })
                .splineToConstantHeading(new Vector2d(-49,-24),Math.PI)

                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(0.62);
                    Intake.intakeWristServo.setPosition(0.268);
                    Intake.CrankPosition(0.38);})
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.IntakePixel(0.75))
                .waitSeconds(0.6)

                // intake pixel into bot
                .addTemporalMarker(()->Intake.CrankPosition(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmServo.setPosition(0.55);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);})
                .waitSeconds(0.5)//0.6
                .addTemporalMarker(()->{
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.44);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    Arm.wristServo.setPosition(0.735);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    Arm.armServo.setPosition(0.15);
                })
                .setReversed(true)


////                .waitSeconds(5)
////                .waitSeconds(0.4)
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
//                    Intake.intakeWristServo.setPosition(0.65);
//                    Intake.intakeArmServo.setPosition(0.55);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(0.7);
//                })
//////                .waitSeconds(0.2)//0.6
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Intake.intakeArmServo.setPosition(1);
//                    Intake.intakeWristServo.setPosition(0.44);
//                })
////                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(-34,-10),0)

// head to backdrop and drop delivery

                .splineToConstantHeading(new Vector2d(18,-10),0)
//                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    Arm.DropPixel(0.5);
                    Arm.armServo.setPosition(0);
                    slider.extendTo(-20, 1);
                })
////                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    Intake.IntakePixel(1);
                    slider.extendTo(0, 1);
                })

                .splineToConstantHeading(new Vector2d(50,-38),0) //46

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.535);
                })


                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.wristServo.setPosition(0.1);
                    Arm.armServo.setPosition(0.515);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Arm.DropPixel(0.9);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.wristServo.setPosition(0.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    Arm.DropPixel(1);
                })

//                .UNSTABLE_addTemporalMarkerOffset(-1,() -> {
//                    Intake.intakeWristServo.setPosition(0.5);Intake.intakeArmServo.setPosition(0.4);
//                    Intake.IntakePixel(0.75);})






//                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Arm.wristServo.setPosition(0.1);
//                    Arm.armServo.setPosition(0.485);
//                    slider.extendTo(30,1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    Arm.DropPixel(1);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
//                    slider.extendTo(0,1);
//                })


//

//
//                .lineToConstantHeading(new Vector2d(48.5, -34))
//                .waitSeconds(1)
//                .addTemporalMarker(()->{})
//
//                .waitSeconds(1)
//                .addTemporalMarker(()-> {
//
//                    Arm.armServo.setPosition(0.15);
//                    Arm.wristServo.setPosition(0.73);
//                })
//                .waitSeconds(200)
//
//

                .build();


        waitForStart();

        TrajectorywithTelemetry(first);




//            while (timer.seconds() < 30){
//                telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
//                telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
//                telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
//                telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
//            }
        telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
        telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
        telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
        telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
        drive.update();
        telemetry.update();


    }

    public void TrajectorywithTelemetry(TrajectorySequence trajectory){
        drive.followTrajectorySequence(trajectory);

        while (opModeIsActive() && drive.isBusy()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            packet.put("RFrontCurrent", drive.getMotorCurrent().get(1));
            packet.put("LeftRearCurrent", drive.getMotorCurrent().get(2));
            packet.put("RightRearCurrent", drive.getMotorCurrent().get(3));

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