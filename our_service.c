/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
extern uint8_t omid[5];
uint8_t new_hex_status[6];
uint8_t * p_data;
/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    // OUR_JOB: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table     
    ble_uuid_t        service_uuid;
		ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
		service_uuid.uuid = BLE_UUID_OUR_SERVICE;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
		APP_ERROR_CHECK(err_code);
		
    // OUR_JOB: Add our service
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                    &service_uuid,
                                    &p_our_service->service_handle);
		APP_ERROR_CHECK(err_code);
	our_char_add(p_our_service);
    // Print messages to Segger Real Time Terminal
    // UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
//    SEGGER_RTT_WriteString(0, "Exectuing our_service_init().\n"); // Print message to RTT to the application flow
//    SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
//    SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
//    SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_our_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}

void our_char_add(ble_os_t * p_our_service)
{
	uint32_t            err_code;
	//define two char 
	ble_uuid_t          char_uuid;
	ble_uuid_t          char_uuid2;
	//set base uuid
	ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
	//set each char a uuid
	char_uuid.uuid      = BLE_UUID_OUR_CHARACTERISTC_UUID;
	char_uuid2.uuid      = BLE_UUID_OUR_CHARACTERISTC_UUID2;
	//add char to service
	err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid2.type);
	APP_ERROR_CHECK(err_code);
	//set perm
	ble_gatts_char_md_t char_md;
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	
	ble_gatts_attr_md_t cccd_md;
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
	char_md.p_cccd_md           = &cccd_md;
	char_md.char_props.notify   = 1;
	
	ble_gatts_attr_md_t attr_md;
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
	//set defailt value for char 1
	ble_gatts_attr_t    attr_char_value;
	memset(&attr_char_value, 0, sizeof(attr_char_value));    
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;
	attr_char_value.max_len = 8 ;
	attr_char_value.init_len = 5 ;
	uint8_t value[5]            = {"navid"};
	attr_char_value.p_value     = value;
	
	//set defailt value for char 2
	ble_gatts_attr_t    attr_char_value2;
	memset(&attr_char_value2, 0, sizeof(attr_char_value2));    
	attr_char_value2.p_uuid      = &char_uuid2;
	attr_char_value2.p_attr_md   = &attr_md;
	attr_char_value2.max_len = 8 ;
	attr_char_value2.init_len = 5 ;
	attr_char_value2.p_value     = value;
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
	err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
	err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value2,
                                   &p_our_service->char_handles2);
	APP_ERROR_CHECK(err_code);
	
}

void ble_our_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt)
{
		switch (p_ble_evt->header.evt_id)
	{
			case BLE_GAP_EVT_CONNECTED:
					p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
					break;
			case BLE_GAP_EVT_DISCONNECTED:
					p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
					break;
			case BLE_GATTS_EVT_WRITE:
					p_data = p_ble_evt->evt.gatts_evt.params.write.data;
          uint16_t length = p_ble_evt->evt.gatts_evt.params.write.len;

            new_hex_status[0] = p_data[0];
						new_hex_status[1] = p_data[1];
						new_hex_status[2] = p_data[2];
						new_hex_status[3] = p_data[3];
						new_hex_status[4] = p_data[4];
						new_hex_status[5] = p_data[5];

					break;
			default:
					// No implementation needed.
					break;
	}
}

 uint32_t our_termperature_characteristic_update(ble_os_t * p_our_service, uint8_t* temperature_value)
{
		//if any device connected the change the char
//		if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
//		{
				uint16_t               len = 5;
				temperature_value = omid;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));
				
				hvx_params.handle = p_our_service->char_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = temperature_value;  
				sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
				return 1;
//		}
//		return 0;
}

uint32_t our_termperature_characteristic_update2(ble_os_t * p_our_service, uint8_t* temperature_value)
{
		//if any device connected the change the char
//		if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
//		{
				uint16_t               len = 5;
				temperature_value = omid;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));
				
				hvx_params.handle = p_our_service->char_handles2.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = temperature_value;  
				sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
				return 1;
//		}
//		return 0;
}
