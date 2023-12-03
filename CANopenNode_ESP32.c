#include "sdkconfig.h"

#if CONFIG_USE_CANOPENNODE

#include "esp_log.h"
#include "CANopen.h"
#include "OD.h"

#if (CONFIG_FREERTOS_HZ != 1000)
#error "FreeRTOS tick interrupt frequency must be 1000Hz"
#endif
#define CO_PERIODIC_TASK_INTERVAL_US (CONFIG_CO_PERIODIC_TASK_INTERVAL_MS * 1000)
#define CO_MAIN_TASK_INTERVAL_US (CONFIG_CO_MAIN_TASK_INTERVAL_MS * 1000)

static const char *TAG = "CO_ESP32";

/* default values for CO_CANopenInit() */
#define NMT_CONTROL (CO_NMT_control_t)(CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

static CO_t *CO = NULL;
static void *CANptr = NULL;

static StaticTask_t xCoMainTaskBuffer;
static StackType_t xCoMainStack[CONFIG_CO_MAIN_TASK_STACK_SIZE];
static TaskHandle_t xCoMainTaskHandle = NULL;
static void CO_mainTask(void *pxParam);

static StaticTask_t xCoPeriodicTaskBuffer;
static StackType_t xCoPeriodicStack[CONFIG_CO_PERIODIC_TASK_STACK_SIZE];
static TaskHandle_t xCoPeriodicTaskHandle = NULL;
static void CO_periodicTask(void *pxParam);

bool CO_ESP32_init()
{
    ESP_LOGI(TAG, "Initializing");
    xCoMainTaskHandle = xTaskCreateStaticPinnedToCore(
        CO_mainTask,
        "CO_main",
        CONFIG_CO_MAIN_TASK_STACK_SIZE,
        (void *)0,
        CONFIG_CO_MAIN_TASK_PRIORITY,
        &xCoMainStack[0],
        &xCoMainTaskBuffer,
        CONFIG_CO_TASK_CORE);
    return true;
}

static void CO_mainTask(void *pxParam)
{
    CO_ReturnError_t err;
    uint32_t errInfo = 0;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t activeNodeId = CONFIG_CO_DEFAULT_NODE_ID;
    TickType_t xLastWakeTime;

    ESP_LOGI(TAG, "main task running.");

    /* Allocate CANopen object */
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL)
    {
        ESP_LOGW(TAG, "Can't allocate memory");
    }
    else
    {
        ESP_LOGI(TAG, "Allocated %d bytes for CANopen objects", (int)heapMemoryUsed);
    }

    while (reset != CO_RESET_APP)
    {
        /* CANopen communication reset - initialize CANopen objects *******************/
        ESP_LOGI(TAG, "CANopenNode - Reset communication");

        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode(CANptr);

        /* Initialize CAN */
        err = CO_CANinit(CO, CANptr, CONFIG_CO_DEFAULT_BPS);
        if (err != CO_ERROR_NO)
        {
            ESP_LOGE(TAG, "CAN initialization failed: %d", err);
        }

        /* Initialize CANopen */
        err = CO_CANopenInit(CO,                           /* CANopen object */
                             NULL,                         /* alternate NMT */
                             NULL,                         /* alternate em */
                             OD,                           /* Object dictionary */
                             NULL,                         /* Optional OD_statusBits */
                             NMT_CONTROL,                  /* CO_NMT_control_t */
                             CONFIG_CO_FIRST_HB_TIME,      /* firstHBTime_ms */
                             CONFIG_CO_SDO_SERVER_TIMEOUT, /* SDOserverTimeoutTime_ms */
                             CONFIG_CO_SDO_CLIENT_TIMEOUT, /* SDOclientTimeoutTime_ms */
#if CONFIG_CO_SDO_CLIENT_BLOCK_TRANSFER
                             true, /* SDOclientBlockTransfer */
#else
                             false, /* SDOclientBlockTransfer */
#endif
                             activeNodeId,
                             &errInfo);
        if ((err != CO_ERROR_NO) && (err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS))
        {
            if (err == CO_ERROR_OD_PARAMETERS)
            {
                ESP_LOGE(TAG, "Object Dictionary entry 0x%lx", errInfo);
            }
            else
            {
                ESP_LOGE(TAG, "CANopen initialization failed: %d", err);
            }
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO)
        {
            if (err == CO_ERROR_OD_PARAMETERS)
            {
                ESP_LOGE(TAG, "Object Dictionary entry 0x%lx", errInfo);
            }
            else
            {
                ESP_LOGE(TAG, "PDO initialization failed: %d", err);
            }
        }

        /*
         * Create Timer Task with execution every 1 millisecond
         */
#if (CONFIG_CO_PERIODIC_TASK_PRIORITY <= CONFIG_CO_MAIN_TASK_PRIORITY)
/*
 * Note: taskCO_timer priority must be higher than taskCO_main priority
 */
#error "Invalid CANopenNode task priority"
#endif
        if (xCoPeriodicTaskHandle == NULL)
        {
            ESP_LOGI(TAG, "creating periodic task");
            xCoPeriodicTaskHandle = xTaskCreateStaticPinnedToCore(
                CO_periodicTask,
                "CO_timer",
                CONFIG_CO_PERIODIC_TASK_STACK_SIZE,
                (void *)0,
                CONFIG_CO_PERIODIC_TASK_PRIORITY,
                &xCoPeriodicStack[0],
                &xCoPeriodicTaskBuffer,
                CONFIG_CO_TASK_CORE);
            if (xCoPeriodicTaskHandle == NULL)
            {
                ESP_LOGE(TAG, "Failed to create periodic task");
            }
        }
        ESP_LOGI(TAG, "periodic task created");

#if CO_CONFIG_LEDS
        CO_LEDs_init(CO->LEDs);
#endif

        /* Start CAN */
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        ESP_LOGI(TAG, "CANopenNode is running");
        xLastWakeTime = xTaskGetTickCount();
        while (reset == CO_RESET_NOT)
        {
            vTaskDelayUntil(&xLastWakeTime, CONFIG_CO_MAIN_TASK_INTERVAL_MS);
            /* CANopen process */
            reset = CO_process(CO, false, CO_MAIN_TASK_INTERVAL_US, NULL);
#if CO_CONFIG_LEDS
            uint32_t ledState;
#if (CONFIG_CO_LED_RED_GPIO >= 0)
            ledState = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
#if CONFIG_CO_LED_RED_ACTIVE_HIGH
            gpio_set_level(CONFIG_CO_LED_RED_GPIO, ledState);
#else
            gpio_set_level(CONFIG_CO_LED_RED_GPIO, (ledState == 0) ? 1 : 0);
#endif /* CONFIG_CO_LED_RED_ACTIVE_HIGH */
#endif
#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
            ledState = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);
#if CONFIG_CO_LED_GREEN_ACTIVE_HIGH
            gpio_set_level(CONFIG_CO_LED_GREEN_GPIO, ledState);
#else
            gpio_set_level(CONFIG_CO_LED_GREEN_GPIO, (ledState == 0) ? 1 : 0);
#endif /* CONFIG_CO_LED_GREEN_ACTIVE_HIGH */
#endif
#endif /* CO_CONFIG_LEDS */
        }
    }

    CO_delete(CO);

    /* Reset */
    ESP_LOGI(TAG, "resetting");
    vTaskDelay(100); /* Put some delay to give time to dump serial log */
    esp_restart();

    /* Will not reach here */
    vTaskDelete(NULL);
}

static void CO_periodicTask(void *pxParam)
{
    ESP_LOGI(TAG, "Periodic task running");

    while (1)
    {
        vTaskDelay(1);
        if ((!CO->nodeIdUnconfigured) && (CO->CANmodule->CANnormal))
        {
            bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
        }
    }
}

#endif /* CONFIG_USE_CANOPENNODE */