package frc.robot.scoreState

import edu.wpi.first.units.*

object ScoreConstants {
    val AMP_ROTATION = Rotation2d.fromDegrees(-90.0)
    val SHOOTER_TOP_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(8.5)
    val SHOOTER_BOTTOM_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(12.5)
    val CONVEYOR_AMP_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(15.0)
}