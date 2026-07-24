package fgc.vietnam.robot01

import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Kotlin adaptation of FIRST's Datalogging Part 4 lifecycle:
 * create a file, write one header, append one CSV record per reading, then
 * close it when the test completes or from OpMode.stop().
 *
 * A complete record is accepted as a list instead of repeated addField/newLine
 * calls, so a partial row cannot leak into the following sample.
 */
internal class Datalogger private constructor(
    val file: File,
    private val writer: BufferedWriter,
) : Closeable {
    var rowCount = 0
        private set

    var errorMessage: String? = null
        private set

    fun writeRow(values: List<Any?>): Boolean {
        if (errorMessage != null) return false

        return try {
            writer.appendLine(values.joinToString(separator = ",", transform = ::escape))
            rowCount += 1
            if (rowCount % FLUSH_INTERVAL_ROWS == 0) writer.flush()
            true
        } catch (exception: IOException) {
            errorMessage = exception.message ?: exception.javaClass.simpleName
            false
        }
    }

    override fun close() {
        try {
            writer.flush()
        } catch (exception: IOException) {
            if (errorMessage == null) {
                errorMessage =
                    exception.message ?: exception.javaClass.simpleName
            }
        }

        try {
            writer.close()
        } catch (exception: IOException) {
            if (errorMessage == null) {
                errorMessage =
                    exception.message ?: exception.javaClass.simpleName
            }
        }
    }

    private fun escape(value: Any?): String {
        val text = value?.toString().orEmpty()
        if (text.none { it == ',' || it == '"' || it == '\n' || it == '\r' }) {
            return text
        }
        return "\"${text.replace("\"", "\"\"")}\""
    }

    internal companion object {
        private const val FLUSH_INTERVAL_ROWS = 25

        fun create(prefix: String, header: List<String>): Datalogger {
            val directory = File(AppUtil.FIRST_FOLDER, "Datalogs")
            if (!directory.exists() && !directory.mkdirs()) {
                throw IOException(
                    "Unable to create datalog directory: ${directory.absolutePath}",
                )
            }

            val timestamp = SimpleDateFormat(
                "yyyyMMdd_HHmmss_SSS",
                Locale.US,
            ).format(Date())
            val file = File(directory, "${prefix}_$timestamp.csv")
            val writer = BufferedWriter(
                OutputStreamWriter(
                    FileOutputStream(file, false),
                    Charsets.UTF_8,
                ),
            )
            val logger = Datalogger(file = file, writer = writer)
            if (!logger.writeRow(header)) {
                val error = logger.errorMessage
                logger.close()
                throw IOException("Unable to write CSV header: $error")
            }
            return logger
        }
    }
}
