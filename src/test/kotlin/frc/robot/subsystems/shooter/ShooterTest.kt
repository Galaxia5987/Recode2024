package frc.robot.subsystems.shooter

import edu.wpi.first.hal.HAL
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

class ShooterTest {
    private lateinit var shooter: Shooter
    private lateinit var schedulerExecutor: ScheduledExecutorService

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0)) { "HAL Initialization Failed" }
        val shooterIO = ShooterIOSim()
        Shooter.initialize(shooterIO)
        shooter = Shooter.getInstance()

        schedulerExecutor = Executors.newScheduledThreadPool(1)
        schedulerExecutor.scheduleAtFixedRate(
            { CommandScheduler.getInstance().run() }, 0, 20, TimeUnit.MILLISECONDS
        )
    }

    @Test
    fun testSetTwoVelocities() {
        val command: Command = shooter.setVelocity(
            Units.RotationsPerSecond.of(30.0), Units.RotationsPerSecond.of(30.0)
        )

        command.execute()

        SimHooks.stepTiming(5.0)

        Thread.sleep(1000)

        println()
        assertTrue(shooter.atSetpoint())
    }

    @AfterEach
    fun tearDown() {
        schedulerExecutor.shutdown()
        try {
            if (!schedulerExecutor.awaitTermination(1, TimeUnit.SECONDS)) {
                schedulerExecutor.shutdownNow()
            }
        } catch (e: InterruptedException) {
            schedulerExecutor.shutdownNow()
        }
    }
}
