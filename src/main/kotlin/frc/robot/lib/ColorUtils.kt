package frc.robot.lib

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.Constants

fun getTranslationByColor(translation: Translation2d): Translation2d {
    return if (Constants.IS_RED) {
        GeometryUtil.flipFieldPosition(translation)
    } else {
        translation
    }
}

fun getPoseByColor(pose: Pose2d): Pose2d {
    return if (Constants.IS_RED) {
        GeometryUtil.flipFieldPose(pose)
    } else {
        pose
    }
}