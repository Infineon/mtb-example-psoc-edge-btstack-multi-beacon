/*******************************************************************************
* File Name:   main.c
*
* Description: This source file contains the main routine for non-secure
*              application in the CM33 CPU for BLE Multi Beacon Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp_bt_config.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_stack.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "stdio.h"
#include "beacon_utils.h"
#include "beacon_utils.h"
#include "wiced_bt_ble.h"
#include "retarget_io_init.h"
#include "cy_time.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/
/* Allocate the multi-advertising instance numbers */
#define BEACON_EDDYSTONE_URL              (1U)
#define BEACON_IBEACON_URL                (2U)

/* This one byte will insert .com at the end of a URL in a URL frame. */
#define DOT_COM                           (0x07U)

/* Minimum and maximum ADV interval */
#define ADVERT_INTERVAL_MIN               (0xA0U)
#define ADVERT_INTERVAL_MAX               (BTM_BLE_ADVERT_INTERVAL_MAX)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC          (10U)

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles
 * to come into effect. This wait time value will depend on the actual CLK_LF
 * frequency set by the BSP */
#define LPTIMER_0_WAIT_TIME_USEC          (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority */
#define APP_LPTIMER_INTERRUPT_PRIORITY    (1U)

/* User defined UUID for iBeacon */
#define UUID_IBEACON      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,\
                          0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f \

/*BDA address initial value */
#define BDA_INIT_VALUE                    (0U)
#define ADV_LEN_INIT_VALUE                (0U)
#define BD_ADDR_LEN_RESET_VAL             (0U)

/* Advertisement URL Data */
#define URL_DATA_0                        ('i')
#define URL_DATA_1                        ('n')
#define URL_DATA_2                        ('f')
#define URL_DATA_3                        ('i')
#define URL_DATA_4                        ('n')
#define URL_DATA_5                        ('e')
#define URL_DATA_6                        ('o')
#define URL_DATA_7                        ('n')
#define URAL_DATA_EOD                     (0x0U)
#define RESET_VALUE                       (0U)


/*******************************************************************************
* Variable Definitions
*******************************************************************************/
/* RTC HAL object */
mtb_hal_rtc_t rtc_obj;

/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;

/* Multi beacon advertisement parameters */
wiced_bt_ble_multi_adv_params_t adv_parameters =
{
    .adv_int_min = ADVERT_INTERVAL_MIN,
    .adv_int_max = ADVERT_INTERVAL_MAX,
    .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38
                 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN,
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX_INDEX,
    .peer_bd_addr = {BDA_INIT_VALUE },
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .own_bd_addr = {BDA_INIT_VALUE },
    .own_addr_type = BLE_ADDR_PUBLIC
};

/*******************************************************************************
*Function Prototypes
*******************************************************************************/

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t
       app_bt_management_callback (wiced_bt_management_evt_t event,
                                   wiced_bt_management_evt_data_t *p_event_data);

/*******************************************************************************
*Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: ble_app_set_advertisement_data
********************************************************************************
* Summary:
*  This function configures the advertisement packet data.
*******************************************************************************/
static void ble_app_set_advertisement_data(void)
{
    uint8_t packet_len;

    /* Eddystone URL advertising packet */
    uint8_t url_packet[BEACON_ADV_DATA_MAX];

    /* Set sample values for iBeacon */
    uint8_t adv_data_ibeacon[BEACON_ADV_DATA_MAX];
    uint8_t adv_len_ibeacon = ADV_LEN_INIT_VALUE;
    uint8_t ibeacon_uuid[LEN_UUID_128] = { UUID_IBEACON };

    /* Eddystone URL data */
    eddystone_url_t url_data = {adv_parameters.adv_tx_power,
                                EDDYSTONE_URL_SCHEME_0,{URL_DATA_0, URL_DATA_1,
                                URL_DATA_2, URL_DATA_3, URL_DATA_4, URL_DATA_5,
                                URL_DATA_6, URL_DATA_7,DOT_COM, URAL_DATA_EOD}};

    /* Set up a URL packet with max power, and implicit "http://www." prefix */
    eddystone_set_data_for_url(url_data, url_packet, &packet_len);

   /* The multi ADV APIs will return pending status now and will
    * give the success/failure status in the BTM_MULTI_ADVERT_RESP_EVENT
    * callback event
    */
    if(WICED_BT_PENDING != wiced_set_multi_advertisement_data(url_packet,
            packet_len, BEACON_EDDYSTONE_URL))
    {
        printf("Set data for URL ADV failed\n");
        handle_app_error();
    }

    if(WICED_BT_PENDING != wiced_set_multi_advertisement_params
            (BEACON_EDDYSTONE_URL, &adv_parameters))
    {
        printf("Set params for URL ADV failed\n");
        handle_app_error();
    }

    if(WICED_BT_PENDING != wiced_start_multi_advertisements
            (MULTI_ADVERT_START, BEACON_EDDYSTONE_URL))
    {
        printf("Start ADV for URL ADV failed\n");
        handle_app_error();
    }

    /* Set up a IBEACON packet  */
   ibeacon_set_adv_data(ibeacon_uuid, IBEACON_MAJOR_NUMER,
                IBEACON_MINOR_NUMER, TX_POWER_LEVEL,
                adv_data_ibeacon, &adv_len_ibeacon);
    if(WICED_BT_PENDING != wiced_set_multi_advertisement_data
            (adv_data_ibeacon, adv_len_ibeacon, BEACON_IBEACON_URL))
    {
        printf("Set data for iBeacon ADV failed\n");
        handle_app_error();
    }

    /* Set advertisment parameters */
    if(WICED_BT_PENDING != wiced_set_multi_advertisement_params
            (BEACON_IBEACON_URL, &adv_parameters))
    {
        printf("Set params for IBEACON ADV failed\n");
        handle_app_error();
    }

    /* Start Multi advertisements */
    if(WICED_BT_PENDING != wiced_start_multi_advertisements
            (MULTI_ADVERT_START, BEACON_IBEACON_URL))
    {
        printf("Start ADV for IBEACON ADV failed\n");
        handle_app_error();
    }

    printf("Multiple ADV started.\n"
            "Use a scanner to scan for ADV packets.\n");
}

/*******************************************************************************
* Function Name: ble_address_print
********************************************************************************
* Summary:
*  This is the utility function that prints the address of the Bluetooth device
*
* Parameters:
*  wiced_bt_device_address_t bdadr : Bluetooth address
*
* Return:
*  void
*
*******************************************************************************/
static void ble_address_print(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=BD_ADDR_LEN_RESET_VAL;i<BD_ADDR_LEN-1;i++)
    {
        printf("%2X:",bdadr[i]);
    }

    printf("%2X\n",bdadr[BD_ADDR_LEN-1]);
}

/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt handler function for LPTimer instance.
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
* 1. This function first configures and initializes an interrupt for LPTimer.
* 2. Then it initializes the LPTimer HAL object to be used in the RTOS
*    tickless idle mode implementation to allow the device enter deep sleep
*    when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
* 3. It then passes the LPTimer object to abstraction RTOS library that
*    implements tickless idle mode.
*******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status =
         Cy_SysInt_Init(&lptimer_intr_cfg,
         lptimer_interrupt_handler);

    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status =
         Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW,
                       &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }

    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
                    CY_MCWDT_CTR_Msk,
                    LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj,
                                             &CYBSP_CM33_LPTIMER_0_hal_config);

    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements
     * tickless idle mode
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_clib_support
********************************************************************************
* Summary:
*    1. This function configures and initializes the Real-Time Clock (RTC).
*    2. It then initializes the RTC HAL object to enable CLIB support library
*       to work with the provided Real-Time Clock (RTC) module.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_clib_support(void)
{
    /* RTC Initialization */
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    /* Initialize the ModusToolbox CLIB support library */
    mtb_clib_support_init(&rtc_obj);
}

/*******************************************************************************
* Function Name : main
* ******************************************************************************
* Summary :
*  Entry point to the application. Set device configuration and start BT
*  freeRTOS tasks and initialization.
*
* Parameters:
*  None
*
* Return:
*  Int
*******************************************************************************/
int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    if(CY_RSLT_SUCCESS != cybsp_init())
    {
        handle_app_error();
    }

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* Setup CLIB support library. */
    setup_clib_support();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    /* Clear UART Terminal Window*/
    printf("\x1b[2J\x1b[;H");

    printf("******************* PSOC Edge MCU: Bluetooth "
            "LE Multi Beacon ******************* \n\n");

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback,
            &cy_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!! \n");
        handle_app_error();
    }

    /* Enable CM55. CM55_APP_BOOT_ADDR must be updated if
     *  CM55 memory layout is changed. */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR,
            CM55_BOOT_WAIT_TIME_USEC);

    /* Enable global interrupts */
    __enable_irq();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    handle_app_error();
}

/*******************************************************************************
* Function Name: app_bt_management_callback
********************************************************************************
* Summary:
*  This is a Bluetooth stack event handler function to receive management events
*  from the BLE stack and process as per the application.
*
* Parameters:
*  wiced_bt_management_evt_t event : BLE event code of one byte length
*  wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event
*                                                structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {BDA_INIT_VALUE};
    wiced_bt_multi_adv_opcodes_t multi_adv_resp_opcode;
    uint8_t multi_adv_resp_status = RESET_VALUE;
    switch (event)
    {
    case BTM_ENABLED_EVT:
        if( WICED_BT_SUCCESS == p_event_data->enabled.status )
        {
            printf("Bluetooth Enabled\r\n");

            wiced_bt_dev_read_local_addr(bda);
                    printf("Local Bluetooth Address: ");
                    ble_address_print(bda);

            /* Create the packet and begin advertising */
            ble_app_set_advertisement_data();
        }
        break;

    case BTM_MULTI_ADVERT_RESP_EVENT:

        /* Multi ADV Response */
        multi_adv_resp_opcode = p_event_data->ble_multi_adv_response_event.opcode;
        multi_adv_resp_status = p_event_data->ble_multi_adv_response_event.status;

        if (SET_ADVT_PARAM_MULTI == multi_adv_resp_opcode)
        {
            if(WICED_SUCCESS == multi_adv_resp_status)
            {
                printf("Multi ADV Set Param Event Status: SUCCESS\n");
            }
            else
            {
                printf("Multi ADV Set Param Event Status: FAILED\n");
            }
        }
        else if (SET_ADVT_DATA_MULTI == multi_adv_resp_opcode)
        {
            if(WICED_SUCCESS == multi_adv_resp_status)
            {
                printf("Multi ADV Set Data Event Status: SUCCESS\n");
            }
            else
            {
                printf("Multi ADV Set Data Event Status: FAILED\n");
            }
        }
        else if (SET_ADVT_ENABLE_MULTI == multi_adv_resp_opcode)
        {
            if(WICED_SUCCESS == multi_adv_resp_status)
            {
                printf("Multi ADV Start Event Status: SUCCESS\n");
            }
            else
            {
                printf("Multi ADV Start Event Status: FAILED\n");
            }
        }
        break;

    default:
        break;
    }

    return status;
}


/* [] END OF FILE */