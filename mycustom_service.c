
#include "mycustom_service.h"
#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

static uint16_t                 service_handle;
static ble_gatts_char_handles_t char1_handles;
static ble_gatts_char_handles_t char2_handles;

uint32_t mycustom_service_init(void)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_srv_security_mode_t        dis_attr_md;
    uint8_t i=0;

    /*create custom service*/
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    //*create custom characteristics*/
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    uint8_t initial_char_values[CHARACTERISTIC_SIZE];
    for(i=0;i<CHARACTERISTIC_SIZE;i++) initial_char_values[i] = 0x00;

    /*char1 will be for writing*/
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write_wo_resp    = 1;
    char_md.char_props.write    = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_CHAR1);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.write_perm);
    attr_md.read_perm  = dis_attr_md.read_perm;
    attr_md.write_perm = dis_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHARACTERISTIC_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHARACTERISTIC_SIZE;
    attr_char_value.p_value   = initial_char_values;

    err_code =  sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, &char1_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /*Char2 will be for reading only*/
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    //char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CUSTOM_CHAR2);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_attr_md.write_perm);
    attr_md.read_perm  = dis_attr_md.read_perm;
    attr_md.write_perm = dis_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHARACTERISTIC_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHARACTERISTIC_SIZE;
    attr_char_value.p_value   = initial_char_values;

    err_code =  sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, &char2_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t mycustom_service_update_data(uint16_t conn_handle,uint8_t *new_data)
{   

    uint16_t len = CHARACTERISTIC_SIZE;
    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = CHARACTERISTIC_SIZE;
    gatts_value.offset  = 0;
    gatts_value.p_value = new_data;

    if(conn_handle!=BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gatts_value_set(conn_handle,
                                          char2_handles.value_handle,
                                          &gatts_value);   
    }
    return err_code;
}
