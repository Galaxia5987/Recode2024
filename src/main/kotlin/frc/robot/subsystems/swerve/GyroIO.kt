package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d

interface GyroIO {
    fun updateInputs(inputs: SwerveDriveInputs) {}

    fun resetGyro(angle: Rotation2d) {}
}
