package frc.robot.subsystems.shooter

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.BaseSubsystemTest
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class ShooterTest : BaseSubsystemTest() {
    private lateinit var shooter: Shooter

    @BeforeEach
    fun initializeShooter() {
        val shooterIO = ShooterIOSim()
        Shooter.initialize(shooterIO)
        shooter = Shooter.getInstance()
    }

    @Test
    fun testSetTwoVelocities() {
        val command: Command = shooter.setVelocity(
            Units.RotationsPerSecond.of(30.0), Units.RotationsPerSecond.of(30.0)
        )

        command.execute()

        simulateTimeAndWait()

        assertTrue(shooter.atSetpoint(), "Shooter should be at setpoint.")
    }
}
