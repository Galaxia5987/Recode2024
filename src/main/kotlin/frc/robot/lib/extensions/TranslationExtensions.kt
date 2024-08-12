package frc.robot.lib.extensions

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

object TranslationExtensions {

    fun Translation2d.getRotationToTranslation(other: Translation2d): Rotation2d {
        return Rotation2d(
            other.x - this.x,
            other.y - this.y
        )
    }
}