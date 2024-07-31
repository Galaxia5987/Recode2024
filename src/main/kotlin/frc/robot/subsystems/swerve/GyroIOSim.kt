package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.lib.math.differential.Integral

class GyroIOSim : GyroIO {
    private val yaw = Integral()

    override fun resetGyro(angle: Rotation2d) {
        yaw.override(angle.radians)
    }

    override fun updateInputs(inputs: SwerveDriveInputs) {
        yaw.update(SwerveDrive.getInstance().currentSpeeds.omegaRadiansPerSecond)
        inputs.yaw = Rotation2d.fromRadians(yaw.get())
        inputs.rawYaw = inputs.yaw
    }
}
