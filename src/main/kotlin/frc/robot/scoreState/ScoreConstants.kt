package frc.robot.scoreState

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity

object ScoreConstants {

    val chainLocations = arrayOf(Pose2d(), Pose2d(), Pose2d()) //left, right, middle

    val AMP_ROTATION: Measure<Angle> = Units.Degrees.of(-90.0)
    val SHOOTER_TOP_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(8.5)
    val SHOOTER_BOTTOM_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(12.5)
    val CONVEYOR_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(15.0)

    val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)
}