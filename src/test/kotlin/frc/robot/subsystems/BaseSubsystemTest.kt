package frc.robot.subsystems

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

abstract class BaseSubsystemTest {
    private lateinit var schedulerExecutor: ScheduledExecutorService

    @BeforeEach
    fun setUp() {
        assert(HAL.initialize(500, 0)) { "HAL Initialization Failed" }

        schedulerExecutor = Executors.newScheduledThreadPool(1)
        schedulerExecutor.scheduleAtFixedRate(
            { CommandScheduler.getInstance().run() }, 0, 20, TimeUnit.MILLISECONDS
        )
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

    protected fun simulateTimeAndWait(simulationTime: Double = 5.0, waitTimeMillis: Long = 1000) {
        SimHooks.stepTiming(simulationTime)
        Thread.sleep(waitTimeMillis)
    }
}
