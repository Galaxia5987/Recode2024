package frc.robot.lib

import com.opencsv.CSVReader
import com.opencsv.exceptions.CsvValidationException
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import java.io.FileReader
import java.io.IOException

fun parseCSVToMap(file: String): InterpolatingDoubleMap {
    val map = InterpolatingDoubleMap(100)
    try {
        CSVReader(FileReader(file)).use { reader ->
            var values: Array<String>
            reader.readNext()
            while ((reader.readNext().also { values = it }) != null) {
                map.put(values[0].toDouble(), values[1].toDouble())
            }
        }
    } catch (e: CsvValidationException) {
        throw RuntimeException(e)
    } catch (e: IOException) {
        throw RuntimeException(e)
    }
    return map
}