package frc.robot.lib.extensions

import frc.robot.lib.extensions.ListAnyExtensions.toBooleanArray
import frc.robot.lib.extensions.ListAnyExtensions.toDoubleArray
import frc.robot.lib.extensions.ListAnyExtensions.toIntArray
import org.littletonrobotics.junction.LogTable

object LogTableExtensions {
    fun LogTable.put(key: String, defaultValue: List<Any>) {
        when {
            defaultValue.all { it is Double } -> put(key, defaultValue.toDoubleArray())
            defaultValue.all { it is Int } -> put(key, defaultValue.toIntArray())
            defaultValue.all { it is Boolean } -> put(key, defaultValue.toBooleanArray())
            else -> throw IllegalArgumentException("Unsupported List type: ${defaultValue::class.simpleName}")
        }
    }

    inline fun <reified T: List<Any>> LogTable.get(key: String, defaultValue: T): T {
        val type = defaultValue::class

        val result: List<Any> = when {
            defaultValue.all { it is Double } -> get(key, defaultValue.toDoubleArray()).toList()
            defaultValue.all { it is Int } -> get(key, defaultValue.toIntArray()).toList()
            defaultValue.all { it is Boolean } -> get(key, defaultValue.toBooleanArray()).toList()
            else -> throw IllegalArgumentException("Unable to LogTable.get List of type: ${type.simpleName}")
        }
        return if (T::class == MutableList::class) result.toMutableList() as T else result as T
    }
}
