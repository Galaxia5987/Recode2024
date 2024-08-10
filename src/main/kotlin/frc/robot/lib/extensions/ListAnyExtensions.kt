package frc.robot.lib.extensions

object ListAnyExtensions {
    fun List<Any>.toDoubleArray(): DoubleArray {
        return this.map { it as Double }.toDoubleArray()
    }

    fun List<Any>.toIntArray(): IntArray {
        return this.map { it as Int }.toIntArray()
    }

    fun List<Any>.toBooleanArray(): BooleanArray {
        return this.map { it as Boolean }.toBooleanArray()
    }
}