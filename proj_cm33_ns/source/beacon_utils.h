/*******************************************************************************
* File Name: beacon_utils.c
*
* Description:This is the source code for the eddystone utility
* functions
*
********************************************************************************
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#ifndef __BEACON_UTILS_H__
#define __BEACON_UTILS_H__

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"


/*******************************************************************************
 *Constants
 ******************************************************************************/
/* Type of eddystone frame
   https://github.com/google/eddystone/blob/master/protocol-specification.md */
#define EDDYSTONE_FRAME_TYPE_UID          (0x00U)
#define EDDYSTONE_FRAME_TYPE_URL          (0x10U)
#define EDDYSTONE_FRAME_TYPE_TLM          (0x20U)
#define EDDYSTONE_FRAME_TYPE_EID          (0x30U)

/* Definitions for UID frame format */
#define EDDYSTONE_UID_FRAME_LEN           (20U)
#define EDDYSTONE_UID_NAMESPACE_LEN       (10U)
#define EDDYSTONE_UID_INSTANCE_ID_LEN     (6U)

/*******************************************************************************
* URL Scheme Prefix for Google Eddystone
* Decimal Hex   Expansion
*   0      0x00   http://www.
*   1      0x01   https://www.
*   2      0x02   http://
*   3      0x03   https://
*
*******************************************************************************/
#define EDDYSTONE_URL_SCHEME_0           (0x00U)
#define EDDYSTONE_URL_SCHEME_1           (0x01U)
#define EDDYSTONE_URL_SCHEME_2           (0x02U)
#define EDDYSTONE_URL_SCHEME_3           (0x03U)

/* Definitions for URL frame format */
#define EDDYSTONE_URL_FRAME_LEN          (20U)
#define EDDYSTONE_URL_VALUE_MAX_LEN      (17U)

/* Eddystone UUID*/
#define EDDYSTONE_UUID16                 (0xFEAAU)

/* Number of advertisement elements for Eddystone */
#define EDDYSTONE_NUM_ELEM_URL           (3U)

/* Max ADV data length */
#define BEACON_ADV_DATA_MAX              (31U)
#define EDDYSTONE_ADV_INDEX0             (0U)
#define EDDYSTONE_ADV_INDEX1             (1U)
#define EDDYSTONE_ADV_INDEX2             (2U)
#define EDDYSTONE_ADV_DATA_INDEX0        (0U)
#define EDDYSTONE_ADV_DATA_INDEX1        (1U)
#define EDDYSTONE_ADV_DATA_INDEX2        (2U)
#define EDDYSTONE_ADV_DATA_INDEX3        (3U)
#define EDDYSTONE_UUID_INDEX0            (0U)
#define EDDYSTONE_UUID_INDEX1            (1U)
#define EDDYSTONE_URL_COM_LENGTH         (3U)
#define EDDYSTONE_SERVICE_DATA_LENGTH    (3U)
#define ADV_PKT_FLAG_LENGTH              (2U)
#define ADV_PKT_16SRV_LENGTH             (3U)
#define UUID_LENGTH                      (2U)

/* iBeacon type */
#define IBEACON_TYPE                     (0x01U)
#define IBEACON_PROXIMITY                (0x02U),(0x15U)
/* iBeacon company ID */
#define IBEACON_COMPANY_ID_APPLE         (0x4cU),(0x00U)
/* iBeacon data length */
#define IBEACON_DATA_LENGTH              (0x19U)
/* iBeacon adv packet length */
#define IBEACON_ADV_PKT_LENGTH           (IBEACON_DATA_LENGTH + 1)

/* Number of elements in advertisement */
#define IBEACON_ELEM_NUM                 (2U)
#define IBEACON_MAJOR_NUMER              (0x01U)
#define IBEACON_MINOR_NUMER              (0x02U)
#define TX_POWER_LEVEL                   (0xB3U)
#define IBEACON_ADV_INDEX0               (0U)
#define IBEACON_ADV_INDEX1               (1U)
#define IBEACON_DATA_INDEX2              (2U)
#define IBEACON_DATA_INDEX3              (3U)
#define IBEACON_DATA_INDEX4              (4U)
#define IBEACON_DATA_INDEX20             (20U)
#define IBEACON_DATA_INDEX21             (21U)
#define IBEACON_DATA_INDEX22             (22U)
#define IBEACON_DATA_INDEX23             (23U)
#define IBEACON_TX_POWER_INDEX           (24U)
#define IBEACON_DATA_COMPANY_ID_INDEX0   (0U)
#define IBEACON_DATA_COMPANY_ID_INDEX1   (1U)
#define IBEACON_DATA_TYPE_INDEX0         (0U)
#define IBEACON_DATA_TYPE_INDEX1         (1U)
#define IBEACON_ADV_DATA0                (0U)
#define IBEACON_DATA_INDEX0              (0U)
#define IBEACON_DATA_INDEX1              (1U)


/*******************************************************************************
* Structures
*******************************************************************************/
/* Structure to hold advertisement element data */
typedef struct
{
    uint8_t data[BEACON_ADV_DATA_MAX];      /* Advertisement  data */
    uint8_t len;                            /* Advertisement length */
    wiced_bt_ble_advert_type_t advert_type; /* Advertisement data type */
}beacon_ble_advert_elem_t;

/* Structure to hold eddystone URL parameters */
typedef struct __attribute__((packed, aligned(1)))
{
    uint8_t tx_power;                                    /* ADV Tx Power */
    uint8_t urlscheme;                                   /* URL Scheme */
    uint8_t encoded_url[EDDYSTONE_URL_VALUE_MAX_LEN];    /* URL */
}eddystone_url_t;

/* Structure to hold eddystone UID parameters */
typedef struct __attribute__((packed, aligned(1)))
{
    uint8_t eddystone_ranging_data;                            /* Calibrated TX power */
    uint8_t eddystone_namespace[EDDYSTONE_UID_NAMESPACE_LEN];  /* UID namespace */
    uint8_t eddystone_instance[EDDYSTONE_UID_INSTANCE_ID_LEN]; /* Instance */
}eddystone_uid_t;

/*******************************************************************************
* Function Declarations
*******************************************************************************/

void eddystone_set_data_for_url  (eddystone_url_t url_data,
                                  uint8_t adv_data[BEACON_ADV_DATA_MAX],
                                  uint8_t *adv_len);

void eddystone_set_data_common   (beacon_ble_advert_elem_t *eddystone_adv_elem,
                                  uint8_t frame_type, uint8_t frame_len);

void beacon_set_adv_data         (beacon_ble_advert_elem_t *beacon_adv_elem,
                                  uint8_t num_elem,
                                  uint8_t adv_data[BEACON_ADV_DATA_MAX],
                                  uint8_t *adv_len);

void ibeacon_set_adv_data        (uint8_t ibeacon_uuid[LEN_UUID_128],
                                   uint16_t ibeacon_major_number,
                                   uint16_t ibeacon_minor_number,
                                   uint8_t tx_power_lcl,
                                   uint8_t adv_data[BEACON_ADV_DATA_MAX],
                                   uint8_t *adv_len);
void beacon_bt_set_adv_data     (beacon_ble_advert_elem_t *beacon_adv_elem,
                                 uint8_t num_elem,
                                 uint8_t adv_data[BEACON_ADV_DATA_MAX],
                                 uint8_t *adv_len);
#endif

/* __BEACON_UTILS_H__ */

/* [] END OF FILE */
