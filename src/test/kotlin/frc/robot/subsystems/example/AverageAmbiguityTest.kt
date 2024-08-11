package frc.robot.subsystems.example

import frc.robot.lib.PoseEstimation
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class AverageAmbiguityTest {

    @Test
    fun testAverageAmbiguity() {
        val ambiguities1 = listOf(1.0, 2.0, 3.0)
        val expected1 = 3.0 / (1.0 / 1.0 + 1.0 / 2.0 + 1.0 / 3.0)
        assertEquals(expected1, PoseEstimation.averageAmbiguity(ambiguities1), 1e-10)

        val ambiguities2 = listOf(4.0, 4.0, 4.0)
        val expected2 = 4.0
        val res = PoseEstimation.averageAmbiguity(ambiguities2)
        assertEquals(expected2, res, 1e-1)
    }
}