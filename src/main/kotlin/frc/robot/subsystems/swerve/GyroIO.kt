package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d

interface GyroIO {
    fun updateInputs(inputs: SwerveDriveInputs) {}

    fun getAccelMagnitude(): Float { return 1.0F } // In G

    fun resetGyro(angle: Rotation2d) {}
}
