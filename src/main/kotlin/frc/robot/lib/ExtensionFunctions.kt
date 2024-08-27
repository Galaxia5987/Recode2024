package frc.robot.lib

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import org.littletonrobotics.junction.LogTable
import kotlin.math.hypot

fun ChassisSpeeds.getSpeed() = hypot(vxMetersPerSecond, vyMetersPerSecond)

fun List<Any>.toDoubleArray(): DoubleArray {
    return this.map { it as Double }.toDoubleArray()
}

fun List<Any>.toIntArray(): IntArray {
    return this.map { it as Int }.toIntArray()
}

fun List<Any>.toBooleanArray(): BooleanArray {
    return this.map { it as Boolean }.toBooleanArray()
}

fun LogTable.put(key: String, defaultValue: List<Any>) {
    when {
        defaultValue.all { it is Double } -> put(key, defaultValue.toDoubleArray())
        defaultValue.all { it is Int } -> put(key, defaultValue.toIntArray())
        defaultValue.all { it is Boolean } -> put(key, defaultValue.toBooleanArray())
        else -> throw IllegalArgumentException("Unsupported List type: ${defaultValue::class.simpleName}")
    }
}

inline fun <reified T : List<Any>> LogTable.get(key: String, defaultValue: T): T {
    val type = defaultValue::class

    val result: List<Any> = when {
        defaultValue.all { it is Double } -> get(key, defaultValue.toDoubleArray()).toList()
        defaultValue.all { it is Int } -> get(key, defaultValue.toIntArray()).toList()
        defaultValue.all { it is Boolean } -> get(key, defaultValue.toBooleanArray()).toList()
        else -> throw IllegalArgumentException("Unable to LogTable.get List of type: ${type.simpleName}")
    }
    return if (T::class == MutableList::class) result.toMutableList() as T else result as T
}

fun Translation2d.getRotationToTranslation(other: Translation2d): Rotation2d = (this - other).angle

fun Command.handleInterrupt(command: Command): WrapperCommand = handleInterrupt { command.schedule() }