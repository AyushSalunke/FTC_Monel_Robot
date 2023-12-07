package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPractice {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d())
                                //.forward(40)
                                //.back(40)
                                //.strafeLeft(40)
                                //.strafeRight(40)
                                //.strafeTo(new Vector2d(40, 40))

                                //.lineTo(new Vector2d(40, 40))
                                //.lineToConstantHeading(new Vector2d(40, 40))
                                //.lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
                                //.lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)))

                                //.splineTo(new Vector2d(40, 40), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0)) //Not Recommended
                                //.splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))

                        //SequenceBuilder Specific Markers
                                //.turn(Math.toRadians(45))                                  // Turns 45 degrees counter-clockwise
                                //.waitSeconds(3)                                            // Waits 3 seconds // Don't use .wait()
                                //.UNSTABLE_addTemporalMarkerOffset(3, () -> {})             // Different from .addTemporalMarker(time, MarkerCallBack)
                                //.UNSTABLE_addDisplacementMarkerOffset(3, () -> {})

                                .addTemporalMarker(2, () -> {
                                    // This marker runs two seconds into the trajectory
                                    // Run your action in here!
                                    // Call Order does not matter
                                    //.addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback)
                                })

                                .addDisplacementMarker(() -> {
                                    // INLINE Displacement Marker
                                    // This marker runs after the path before it
                                    // Run your action in here!
                                    // Call Order matters
                                })
                                .addDisplacementMarker(20, () -> {
                                    // GLOBAL Displacement Marker
                                    // This marker runs 20 inches into the trajectory
                                    // Run your action in here!
                                    // Call Order does not matter
                                    //addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback)
                                })

                                .addSpatialMarker(new Vector2d(20, 20), () -> {
                                    // This marker runs at the point that gets
                                    // closest to the (20, 20) coordinate
                                    // Run your action in here!
                                    // Not Recommended to use
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
