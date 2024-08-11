package frc.robot.scoreState

import edu.wpi.first.math.geometry.Rotation2d

object ScoreConstants {
    val AMP_ROTATION = Rotation2d.fromDegrees(-90.0)
    val CONVEYOR_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(15.0)
}