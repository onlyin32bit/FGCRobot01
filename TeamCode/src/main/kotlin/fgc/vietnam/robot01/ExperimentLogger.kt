package fgc.vietnam.robot01

/** The minimal envelope shared by every robot mechanism experiment. */
internal data class ExperimentRunContext(
    val scenarioId: String,
    val configurationId: String = "UNSET",
    val trial: Int = 1,
)

internal data class ExperimentSample(
    val sampleIndex: Int,
    val timeS: Double,
    val loopDtMs: Double? = null,
    val event: String = "",
    val batteryV: Double? = null,
)

internal enum class ExperimentEvent {
    LOG_START,
    START,
    COMMAND_START,
    TARGET_REACHED,
    COMMAND_STOP,
    STOPPED,
    MARK,
    END,
    ABORT,
    FAULT,
}

internal class ExperimentLogger private constructor(
    private val datalogger: Datalogger,
    private val context: ExperimentRunContext,
    private val scenarioColumns: List<String>,
) : AutoCloseable {
    val fileName: String
        get() = datalogger.file.name

    val rowCount: Int
        get() = datalogger.rowCount

    val errorMessage: String?
        get() = datalogger.errorMessage

    fun write(
        sample: ExperimentSample,
        scenarioValues: Map<String, Any?> = emptyMap(),
    ): Boolean {
        require(scenarioValues.keys == scenarioColumns.toSet()) {
            "Scenario values must match registered scenario columns"
        }
        return datalogger.writeRow(
            listOf(
                SCHEMA_VERSION,
                datalogger.file.nameWithoutExtension,
                context.scenarioId,
                context.configurationId,
                context.trial,
                sample.sampleIndex,
                sample.timeS,
                sample.loopDtMs,
                sample.event,
                sample.batteryV,
            ) + scenarioColumns.map(scenarioValues::get),
        )
    }

    override fun close() = datalogger.close()

    internal companion object {
        const val SCHEMA_VERSION = "fgc-ts-v1"

        val GLOBAL_COLUMNS = listOf(
            "schema_version",
            "run_id",
            "scenario_id",
            "configuration_id",
            "trial",
            "sample_index",
            "time_s",
            "loop_dt_ms",
            "event",
            "battery_v",
        )

        fun events(vararg events: ExperimentEvent): String =
            events.joinToString(separator = "|") { it.name }

        fun create(
            prefix: String,
            context: ExperimentRunContext,
            scenarioColumns: List<String> = emptyList(),
        ): ExperimentLogger {
            require(scenarioColumns.distinct().size == scenarioColumns.size) {
                "Scenario columns must be unique"
            }
            require(scenarioColumns.all { it.startsWith("scenario_") }) {
                "Scenario columns must use the scenario_ prefix"
            }
            return ExperimentLogger(
                datalogger = Datalogger.create(
                    prefix,
                    GLOBAL_COLUMNS + scenarioColumns,
                ),
                context = context,
                scenarioColumns = scenarioColumns,
            )
        }
    }
}
