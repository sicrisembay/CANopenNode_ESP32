/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster / Sicris Embay
 * @copyright   2004 - 2023 Janez Paternoster / Sicris Embay
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "301/CO_driver.h"
#include "esp_log.h"
#include "driver/twai.h"

static const char *TAG = "CO_driver";

typedef struct
{
    uint16_t kbps;
    twai_timing_config_t timing_config;
} baudrate_config_t;

static baudrate_config_t const baudrate_config[] = {
#if CONFIG_CO_BPS_25K
    {25, TWAI_TIMING_CONFIG_25KBITS()},
#endif
#if CONFIG_CO_BPS_50K
    {50, TWAI_TIMING_CONFIG_50KBITS()},
#endif
#if CONFIG_CO_BPS_100K
    {100, TWAI_TIMING_CONFIG_100KBITS()},
#endif
#if CONFIG_CO_BPS_125K
    {125, TWAI_TIMING_CONFIG_125KBITS()},
#endif
#if CONFIG_CO_BPS_250K
    {250, TWAI_TIMING_CONFIG_250KBITS()},
#endif
#if CONFIG_CO_BPS_500K
    {500, TWAI_TIMING_CONFIG_500KBITS()},
#endif
#if CONFIG_CO_BPS_1M
    {1000, TWAI_TIMING_CONFIG_1MBITS()},
#endif
#if CONFIG_CO_BPS_25K
    {25, TWAI_TIMING_CONFIG_25KBITS()},
#endif
};

static StaticTask_t xCoTxTaskBuffer;
static StackType_t xCoTxStack[CONFIG_CO_TX_TASK_STACK_SIZE];
static TaskHandle_t xCoTxTaskHandle = NULL;
static void CO_txTask(void *pxParam);

static StaticTask_t xCoRxTaskBuffer;
static StackType_t xCoRxStack[CONFIG_CO_RX_TASK_STACK_SIZE];
static TaskHandle_t xCoRxTaskHandle = NULL;
static void CO_rxTask(void *pxParam);

static bool bInstalled = false;

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */

    CANmodule->CANnormal = true;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
    CO_CANmodule_t *CANmodule,
    void *CANptr,
    CO_CANrx_t rxArray[],
    uint16_t rxSize,
    CO_CANtx_t txArray[],
    uint16_t txSize,
    uint16_t CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

#if CONFIG_CO_LED_ENABLE
    gpio_config_t io_conf;
    uint64_t pinMask = 0ULL;
#if (CONFIG_CO_LED_RED_GPIO >= 0)
    pinMask |= (1ULL << CONFIG_CO_LED_RED_GPIO);
#endif
#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
    pinMask |= (1ULL << CONFIG_CO_LED_GREEN_GPIO);
#endif
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = pinMask;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    /* Set LED off */
#if (CONFIG_CO_LED_RED_GPIO >= 0)
    gpio_set_level(CONFIG_CO_LED_RED_GPIO,
#if CONFIG_CO_LED_RED_ACTIVE_HIGH
                   0);
#else
                   1);
#endif /* CONFIG_CO_LED_RED_ACTIVE_HIGH */
#endif

#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
    gpio_set_level(CONFIG_CO_LED_GREEN_GPIO,
#if CONFIG_CO_LED_GREEN_ACTIVE_HIGH
                   0);
#else
                   1);
#endif
#endif
#endif /* CONFIG_CO_LED_ENABLE */

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (i = 0U; i < rxSize; i++)
    {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++)
    {
        txArray[i].bufferFull = false;
    }

    /* Configure CAN module registers */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CO_TWAI_TX_GPIO, CONFIG_CO_TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_timing_config_t t_config;
    for (i = 0; i < (sizeof(baudrate_config) / sizeof(baudrate_config[0])); i++)
    {
        if (CANbitRate == baudrate_config[i].kbps)
        {
            t_config = baudrate_config[i].timing_config;
            break;
        }
    }
    if (i >= (sizeof(baudrate_config) / sizeof(baudrate_config[0])))
    {
        /* Baudrate not found */
        return CO_ERROR_ILLEGAL_BAUDRATE;
    }

    /* Install TWAI driver */
    if (bInstalled != true)
    {
        /* create Mutex */
        CANmodule->xMutexCanSendHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexCanSendBuf));
        CANmodule->xMutexEmcyHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexEmcyBuf));
        CANmodule->xMutexODHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexODBuf));

        /* Install TWAI */
        ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
        ESP_LOGI(TAG, "Driver installed");
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(TAG, "Driver started");

        bInstalled = true;

        /* Create Tx tasks */
        ESP_LOGI(TAG, "Creating Tx Task");
        xCoTxTaskHandle = xTaskCreateStaticPinnedToCore(
            CO_txTask,
            "CO_tx",
            CONFIG_CO_TX_TASK_STACK_SIZE,
            (void *)CANmodule,
            CONFIG_CO_TX_TASK_PRIORITY,
            &xCoTxStack[0],
            &xCoTxTaskBuffer,
            CONFIG_CO_TASK_CORE);
        if (xCoTxTaskHandle == NULL)
        {
            ESP_LOGE(TAG, "txTask creation failed");
            return CO_ERROR_OUT_OF_MEMORY;
        }
        /* Create Rx tasks */
        ESP_LOGI(TAG, "Creating Rx Task");
        xCoRxTaskHandle = xTaskCreateStaticPinnedToCore(
            CO_rxTask,
            "CO_rx",
            CONFIG_CO_RX_TASK_STACK_SIZE,
            (void *)CANmodule,
            CONFIG_CO_RX_TASK_PRIORITY,
            &xCoRxStack[0],
            &xCoRxTaskBuffer,
            CONFIG_CO_TASK_CORE);
        if (xCoRxTaskHandle == NULL)
        {
            ESP_LOGE(TAG, "rxTask creation failed");
            return CO_ERROR_OUT_OF_MEMORY;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Driver already installed");
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL)
    {
        /* Take all mutex before deleting it */
        xSemaphoreTakeRecursive(CANmodule->xMutexCanSendHdl, portMAX_DELAY);
        xSemaphoreTakeRecursive(CANmodule->xMutexEmcyHdl, portMAX_DELAY);
        xSemaphoreTakeRecursive(CANmodule->xMutexODHdl, portMAX_DELAY);

        /* Delete Tx and Rx Tasks */
        vTaskDelete(xCoTxTaskHandle);
        xCoTxTaskHandle = NULL;
        vTaskDelete(xCoRxTaskHandle);
        xCoRxTaskHandle = NULL;
        ESP_LOGI(TAG, "tx and rx tasks deleted");

        /* As holder of mutex, it is safe to delete it */
        vSemaphoreDelete(CANmodule->xMutexCanSendHdl);
        vSemaphoreDelete(CANmodule->xMutexEmcyHdl);
        vSemaphoreDelete(CANmodule->xMutexODHdl);
        CANmodule->xMutexCanSendHdl = NULL;
        CANmodule->xMutexEmcyHdl = NULL;
        CANmodule->xMutexODHdl = NULL;
        ESP_LOGI(TAG, "mutex deleted");

        /* Uninstall TWAI */
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(TAG, "Driver stopped");
        ESP_ERROR_CHECK(twai_driver_uninstall());
        ESP_LOGI(TAG, "Driver uninstalled");

        bInstalled = false;
    }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    uint16_t mask,
    bool_t rtr,
    void *object,
    void (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize))
    {
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if (rtr)
        {
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters)
        {
        }
    }
    else
    {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    bool_t rtr,
    uint8_t noOfBytes,
    bool_t syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize))
    {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];
        buffer->ident = (uint32_t)ident & 0x07FFU;
        if (rtr)
        {
            buffer->ident |= 0x0800U;
        }
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        if (!CANmodule->firstCANtxMessage)
        {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

#if CONFIG_CO_DEBUG_DRIVER_CAN_SEND
    ESP_LOGI(TAG, "CANTX id: 0x%lx, dlc: %d, data: [%d %d %d %d %d %d %d %d]",
             buffer->ident,
             buffer->DLC,
             buffer->data[0],
             buffer->data[1],
             buffer->data[2],
             buffer->data[3],
             buffer->data[4],
             buffer->data[5],
             buffer->data[6],
             buffer->data[7]);
#endif

    CO_LOCK_CAN_SEND(CANmodule);
    buffer->bufferFull = true;
    CANmodule->CANtxCount++;
    xTaskNotify(xCoTxTaskHandle, 0, eNoAction);
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag)
    {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U)
    {
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--)
        {
            if (buffer->bufferFull)
            {
                if (buffer->syncFlag)
                {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err;
    twai_status_info_t statusInfo;
    esp_err_t espRet = ESP_OK;

    espRet = twai_get_status_info(&statusInfo);
    if (espRet != ESP_OK)
    {
        ESP_LOGW(TAG, "twai_get_status_info returns %d", espRet);
        return;
    }

    txErrors = (uint16_t)(statusInfo.tx_error_counter);
    rxErrors = (uint16_t)(statusInfo.rx_error_counter);
    overflow = (uint16_t)(statusInfo.rx_overrun_count);

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err)
    {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U)
        {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else
        {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128)
            {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            }
            else if (rxErrors >= 96)
            {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128)
            {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            }
            else if (rxErrors >= 96)
            {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0)
            {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0)
        {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}

/******************************************************************************/
static void CO_txTask(void *pxParam)
{
    uint32_t notificationValue;
    twai_message_t tx_msg;
    CO_CANtx_t *pCanTx;
    esp_err_t espRet;
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)pxParam;
    ESP_LOGI(TAG, "tx task running");

    while (1)
    {
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &notificationValue, portMAX_DELAY);

        CO_LOCK_CAN_SEND(CANmodule);
        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        while (CANmodule->CANtxCount > 0U)
        {
            /* search through whole array of pointers to transmit message buffers. */
            for (int i = 0; i < CANmodule->txSize; i++)
            {
                pCanTx = &(CANmodule->txArray[i]);
                if (pCanTx->bufferFull)
                {
                    memset(&tx_msg, 0, sizeof(tx_msg));
                    tx_msg.identifier = pCanTx->ident;
                    tx_msg.data_length_code = pCanTx->DLC;
                    memcpy(tx_msg.data, pCanTx->data, TWAI_FRAME_MAX_DLC);

                    espRet = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
                    if (ESP_OK == espRet)
                    {
                        pCanTx->bufferFull = false;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed Tx. id:%d err:0x%x", i, espRet);
                    }
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = pCanTx->syncFlag;
                    break; /* exit for loop */
                }
            }
        }
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

static void CO_rxTask(void *pxParam)
{
    twai_message_t rx_msg;
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)pxParam;
    ESP_LOGI(TAG, "rx task running");

    while (1)
    {
        twai_message_t *rcvMsg;    /* pointer to received message in CAN module */
        uint16_t index;            /* index of received message */
        uint32_t rcvMsgIdent;      /* identifier of the received message */
        CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        twai_receive(&rx_msg, portMAX_DELAY);

#if CONFIG_CO_DEBUG_DRIVER_CAN_RECEIVE
        ESP_LOGI(TAG, "CANRX id: 0x%lx, dlc: %d, data: [%d %d %d %d %d %d %d %d]",
                 rx_msg.identifier,
                 rx_msg.data_length_code,
                 rx_msg.data[0],
                 rx_msg.data[1],
                 rx_msg.data[2],
                 rx_msg.data[3],
                 rx_msg.data[4],
                 rx_msg.data[5],
                 rx_msg.data[6],
                 rx_msg.data[7]);
#endif /* CONFIG_CO_DEBUG_DRIVER_CAN_RECEIVE */

        rcvMsg = &rx_msg;
        rcvMsgIdent = rx_msg.identifier;
        /* CAN module filters are not used, message with any standard 11-bit identifier */
        /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
        buffer = &CANmodule->rxArray[0];
        for (index = CANmodule->rxSize; index > 0U; index--)
        {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                msgMatched = true;
                break;
            }
            buffer++;
        }

        /* Call specific function, which will process the message */
        if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL))
        {
            buffer->CANrx_callback(buffer->object, (void *)rcvMsg);
        }
    }
}