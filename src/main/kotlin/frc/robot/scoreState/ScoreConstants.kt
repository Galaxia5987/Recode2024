package frc.robot.scoreState

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import frc.robot.lib.parseCSVToMap

object ScoreConstants {

    val AMP_ROTATION: Angle = Units.Degrees.of(-90.0)
    val HOOD_AMP_ANGLE: Angle = Units.Degrees.of(100.0)
    val SHOOTER_TOP_AMP_VELOCITY: AngularVelocity = Units.RotationsPerSecond.of(8.5)
    val SHOOTER_BOTTOM_AMP_VELOCITY: AngularVelocity = Units.RotationsPerSecond.of(12.0)
    val CONVEYOR_AMP_VELOCITY: AngularVelocity = Units.RotationsPerSecond.of(10.0)

    val HOOD_ANGLE_BY_DISTANCE: InterpolatingDoubleMap =
        parseCSVToMap(
            Filesystem.getDeployDirectory().path + "/shootData/distance-to-angle.csv"
        )

    val SHOOTER_VELOCITY_BY_DISTANCE: InterpolatingDoubleMap =
        parseCSVToMap(
            Filesystem.getDeployDirectory().path + "/shootData/distance-to-shooter-velocity.csv"
        )

    val CONVEYOR_VELOCITY_BY_DISTANCE: InterpolatingDoubleMap =
        parseCSVToMap(
            Filesystem.getDeployDirectory().path + "/shootData/distance-to-conveyor-velocity.csv"
        )
}