#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include "config_thread.h"

#include "max40109_hal.h"

#include <stdlib.h>
#if defined(RT_USING_FINSH) || defined(RT_USING_MSH)
#include <finsh.h>
#endif

typedef struct
{
    const char *name;
    config_update_name update_name;
    void *ptr;
    size_t size;
} config_entry_t;


#define MAX_CONFIG_ITEMS 32
static config_entry_t config_registry[MAX_CONFIG_ITEMS];
static int config_registry_count = 0;
rt_mq_t config_update_notify = RT_NULL;

app_config_t app_config = {
        .adc_gain = 1,
        .sample_rate = 1,
        .adc_enable_channel = 0,
        .strain_S1 = 1.0,
        .epsilon_0 = 0.0,
        .acceleration_S2 = 1.0,
        .a_0 = 0.0,
        .adc_data_type = 1,
        .enable_filter = 0,
        .outlier_detection_method = OUTLIER_DETECT_NONE,
        .filter_type = FILTER_TYPE_NONE,
        .outlier_max = 1500000,
        .outlier_min = -1500000,
        .gradient_threshold = 400000,
        .n_sigma = 3.0,
        .low_pass_alpha = 0.1,
};

/* Registry implementation */
int config_register(const char *name, config_update_name update_name, void *ptr, size_t size)
{
    if (!name || !ptr) return -RT_ERROR;
    if (config_registry_count >= MAX_CONFIG_ITEMS) return -RT_ERROR;

    config_registry[config_registry_count].name = name;
    config_registry[config_registry_count].update_name = update_name;
    config_registry[config_registry_count].ptr = ptr;
    config_registry[config_registry_count].size = size;
    config_registry_count++;
    return RT_EOK;
}

static config_entry_t *find_entry(const char *name)
{
    for (int i = 0; i < config_registry_count; i++)
    {
        if (strcmp(config_registry[i].name, name) == 0)
            return &config_registry[i];
    }
    return RT_NULL;
}

static void config_send_update_message(config_update_name msg_name)
{
    config_update_msg_t msg = {
        .msg_name = msg_name
    };

    rt_err_t result = rt_mq_send(config_update_notify, &msg, sizeof(msg));
    if (result != RT_EOK) {
        rt_kprintf("Failed to send config message: %d\n", msg_name);
    }
}


int config_set(const char *name, const char *value)
{
    config_entry_t *e = find_entry(name);
    if (!e) return -RT_ERROR;

    switch (e->update_name){

    case CONFIG_ADC_GAIN:{
        rt_uint16_t v = (rt_uint16_t)atoi(value);
        if(v != 1 && v != 2 && v != 4 && v != 8 && v != 16 && v != 32 && v != 64 && v != 128){
            return -RT_ERROR;
        }
        memcpy(e->ptr, &v, sizeof(rt_uint16_t));
        break;
    }
    case CONFIG_ADC_ENABLE_CHANNEL:{
        size_t len = strlen(value);
        if (len == 0 || len > 8) {
            return -RT_ERROR;
        }
        rt_uint8_t v = 0;
        for (size_t i = 0; i < len; i++) {
            v <<= 1;

            if (value[i] == '1') {
                v |= 1;
            } else if (value[i] != '0') {
                return -RT_ERROR;
            }
        }
        memcpy(e->ptr, &v, sizeof(rt_uint8_t));
        break;
    }
    case CONFIG_SAMPLE_RATE:{
        rt_uint8_t v = (rt_uint8_t)atoi(value);
        if(v != 1 && v != 10 && v != 20 && v != 50 && v != 100 && v != 200){
            return -RT_ERROR;
        }
        memcpy(e->ptr, &v, sizeof(rt_uint8_t));
        break;
    }
    case CONFIG_ADC_DATA_TYPE:{
        rt_uint8_t v = (rt_uint8_t)atoi(value);
        if(v != 1 && v != 2 && v != 3){
            return -RT_ERROR;
        }
        memcpy(e->ptr, &v, sizeof(rt_uint8_t));
        break;
    }
    case CONFIG_ENABLE_FILTER:{
        rt_uint8_t v = (rt_uint8_t)atoi(value);
        if(v != 1 && v != 0){
            return -RT_ERROR;
        }
        memcpy(e->ptr, &v, sizeof(rt_uint8_t));
        break;
    }
    case CONFIG_OUTLIER_DETECTION_METHOD:{
        int v = atoi(value);
        if(v < 0 || v > OUTLIER_DETECT_ALL_SEQUENTIAL){
            return -RT_ERROR;
        }

        OutlierDetectionMethod method = (OutlierDetectionMethod)v;
        memcpy(e->ptr, &method, sizeof(OutlierDetectionMethod));
        break;
    }
    case CONFIG_FILTER_TYPE:{
        int v = atoi(value);
        if(v < 0 || v > FILTER_TYPE_LOW_PASS){
            return -RT_ERROR;
        }

        FilterType method = (FilterType)v;
        memcpy(e->ptr, &method, sizeof(FilterType));
        break;
    }

    case CONFIG_OUTLIER_MAX:
    case CONFIG_OUTLIER_MIN:
    case CONFIG_GRADIENT_THRESHLOD:{
        rt_int32_t v = (rt_int32_t)atoi(value);
        memcpy(e->ptr, &v, sizeof(rt_int32_t));
        break;
    }
    case CONFIG_N_SIGMA:
    case CONFIG_LOW_PASS_ALPHA:
    case CONFIG_STRAIN_S1:
    case CONFIG_ACCELERATION_S2:
    case CONFIG_EPSILON_0:
    case CONFIG_A_0:{
        float v = (float)atof(value);
        memcpy(e->ptr, &v, sizeof(float));
        break;
    }
    default:
        return -RT_ERROR;
        break;
    }
    config_send_update_message(e->update_name);
    return RT_EOK;
}

int config_get(const char *name, char *out, size_t out_size)
{
    config_entry_t *e = find_entry(name);
    if (!e || !out) return -RT_ERROR;

    switch (e->update_name)
    {
    case CONFIG_OUTLIER_MAX:
    case CONFIG_OUTLIER_MIN:
    case CONFIG_GRADIENT_THRESHLOD:{
        rt_int32_t v;
        memcpy(&v, e->ptr, sizeof(rt_int32_t));
        rt_snprintf(out, out_size, "%d", v);
        break;
    }
    case CONFIG_ADC_GAIN:{
        rt_uint16_t v;
        memcpy(&v, e->ptr, sizeof(rt_uint16_t));
        rt_snprintf(out, out_size, "%d", v);
        break;
    }
    case CONFIG_ADC_ENABLE_CHANNEL:
    {
        rt_uint8_t v;
        memcpy(&v, e->ptr, sizeof(rt_uint8_t));
        rt_snprintf(out, out_size, "%d%d%d%d%d%d%d%d",
                (v >> 7) & 1,
                (v >> 6) & 1,
                (v >> 5) & 1,
                (v >> 4) & 1,
                (v >> 3) & 1,
                (v >> 2) & 1,
                (v >> 1) & 1,
                v & 1);
        break;
    }
    case CONFIG_ADC_DATA_TYPE:
    {
        rt_uint8_t v;
        memcpy(&v, e->ptr, sizeof(rt_uint8_t));
        switch (v) {
            case 1:
                rt_snprintf(out, out_size, "%s", "1(voltage)");
                break;
            case 2:
                rt_snprintf(out, out_size, "%s", "2(strain)");
                break;
            case 3:
                rt_snprintf(out, out_size, "%s", "3(acceleration)");
                break;
            default:
                rt_snprintf(out, out_size, "error value: %d", v);
                break;
        }
        break;
    }
    case CONFIG_OUTLIER_DETECTION_METHOD:
    {
        OutlierDetectionMethod v;
        memcpy(&v, e->ptr, sizeof(OutlierDetectionMethod));
        switch (v) {
        case OUTLIER_DETECT_NONE:
            rt_snprintf(out, out_size, "%s", "0(OUTLIER_DETECT_NONE)");
            break;
        case OUTLIER_DETECT_LIMIT:
            rt_snprintf(out, out_size, "%s", "1(OUTLIER_DETECT_LIMIT)");
            break;
        case OUTLIER_DETECT_GRADIENT:
            rt_snprintf(out, out_size, "%s", "2(OUTLIER_DETECT_GRADIENT)");
            break;
        case OUTLIER_DETECT_3SIGMA:
            rt_snprintf(out, out_size, "%s", "3(OUTLIER_DETECT_3SIGMA)");
            break;
        case OUTLIER_DETECT_ALL_SEQUENTIAL:
            rt_snprintf(out, out_size, "%s", "4(OUTLIER_DETECT_ALL_SEQUENTIAL)");
            break;
        default:
            rt_snprintf(out, out_size, "error value: %d", v);
            break;
        }
        break;
    }

    case CONFIG_FILTER_TYPE:
    {
        FilterType v;
        memcpy(&v, e->ptr, sizeof(FilterType));
        switch (v) {
        case FILTER_TYPE_NONE:
            rt_snprintf(out, out_size, "%s", "0(FILTER_TYPE_NONE)");
            break;
        case FILTER_TYPE_MOVING_AVERAGE:
            rt_snprintf(out, out_size, "%s", "1(FILTER_TYPE_MOVING_AVERAGE)");
            break;
        case FILTER_TYPE_MEDIAN:
            rt_snprintf(out, out_size, "%s", "2(FILTER_TYPE_MEDIAN)");
            break;
        case FILTER_TYPE_LOW_PASS:
            rt_snprintf(out, out_size, "%s", "3(FILTER_TYPE_LOW_PASS)");
            break;
        default:
            rt_snprintf(out, out_size, "error value: %d", v);
            break;
        }
        break;

    }
    case CONFIG_SAMPLE_RATE:
    case CONFIG_ENABLE_FILTER:
    {
        rt_uint8_t v;
        memcpy(&v, e->ptr, sizeof(rt_uint8_t));
        rt_snprintf(out, out_size, "%d", v);
        break;
    }
    case CONFIG_N_SIGMA:
    case CONFIG_LOW_PASS_ALPHA:
    case CONFIG_STRAIN_S1:
    case CONFIG_ACCELERATION_S2:
    case CONFIG_EPSILON_0:
    case CONFIG_A_0:{
        float v;
        memcpy(&v, e->ptr, sizeof(float));
        int integer_part = (int)v;
        int decimal_part = (int)((v - integer_part) * 100000);
        if (decimal_part < 0) decimal_part = -decimal_part;
        rt_snprintf(out, out_size, "%d.%05d", integer_part, decimal_part);
        break;
    }
    default:
        return -RT_ERROR;
    }
    return RT_EOK;

}

void config_print_all(void)
{
    char buf[64];
    rt_kprintf("Current Configuration:\n");
    for (int i = 0; i < config_registry_count; i++)
    {
        if (config_get(config_registry[i].name, buf, sizeof(buf)) == RT_EOK)
        {
            rt_kprintf("  %s: %s\n", config_registry[i].name, buf);
        }
        else
        {
            rt_kprintf("  %s: <error>\n", config_registry[i].name);
        }
    }
    rt_kprintf("-----------------------\n");
}

static rt_err_t write_max_chip(const char* id_str, const char* addr_str, const char* val_str,rt_uint8_t len)
{
    rt_uint8_t reg_addr;
    rt_uint16_t reg_val;
    rt_err_t res = RT_EOK;
    reg_addr = (rt_uint8_t)strtol(addr_str, NULL, 0);
    reg_val = (rt_uint16_t)strtol(val_str, NULL, 0);
    if (strcmp(id_str, "all") == 0)
    {
        rt_kprintf("Writing 0x%04X to Reg 0x%02X on ALL chips...\n", reg_val, reg_addr);
        for (rt_uint8_t i = 0; i < 8; i++)
        {
            if (global_max40109_write_reg(i, reg_addr, reg_val,len) != RT_EOK)
            {
                rt_kprintf("Chip %d: Fail\n", i);
                res = -RT_EIO;
            }
        }
    }
    else
    {
        rt_uint8_t chip_idx = (rt_uint8_t)atoi(id_str);
        if (chip_idx >= 8)
        {
            rt_kprintf("Invalid Chip ID: %d (0-7 or 'all')\n", chip_idx);
            return -RT_ERROR;
        }
        res = global_max40109_write_reg(chip_idx, reg_addr, reg_val,len);
    }
    return res;
}

static rt_err_t read_max_chip(const char* id_str, const char* addr_str,rt_uint8_t len)
{
    rt_uint8_t reg_addr;
    rt_err_t res = RT_EOK;
    reg_addr = (rt_uint8_t)strtol(addr_str, NULL, 0);
    rt_uint8_t chip_idx = (rt_uint8_t)atoi(id_str);

    if (chip_idx >= 8)
    {
        rt_kprintf("Invalid Chip ID: %d (0-7)\n", chip_idx);
        return -RT_ERROR;
    }
    rt_uint16_t val;

    res = global_max40109_read_reg(chip_idx, reg_addr,&val,len);

    if (res == RT_EOK) rt_kprintf("current value: Chip[%d] [0x%X] = Value[0x%X]\n",chip_idx, reg_addr,val);

    return res;
}


/**
 * @brief Entry point for the configuration thread.
 *
 * This function provides a simple shell for modifying the application configuration.
 * @param parameter Unused.
 */
/* Thread entry is unused when running via finsh/msh only. If you later
 * want to re-enable a CLI thread, you can restore the implementation.
 */
/*
static void config_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
}
*/

#if defined(RT_USING_FINSH) || defined(RT_USING_MSH)
/* Finsh command wrapper - integrates registry with system shell.
 * Usage:
 *   config list
 *   config get <name>
 *   config set <name> <value>
 */
static int finsh_cmd_config(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: config list|get|set|write_max_chip ...\n");
        return -RT_ERROR;
    }

    if (strcmp(argv[1], "list") == 0)
    {
        config_print_all();
        return RT_EOK;
    }
    else if (strcmp(argv[1], "get") == 0)
    {
        if (argc != 3) { rt_kprintf("usage: config get <name>\n"); return -RT_ERROR; }
        char buf[64];
        if (config_get(argv[2], buf, sizeof(buf)) == RT_EOK) 
            rt_kprintf("%s\n", buf);
        else 
            rt_kprintf("not found\n");
        return RT_EOK;
    }
    else if (strcmp(argv[1], "set") == 0)
    {
        if (argc != 4) {
             rt_kprintf("usage: config set <name> <value>\n");
             return -RT_ERROR; 
        }

        if (config_set(argv[2], argv[3]) == RT_EOK){
            rt_kprintf("ok\n");
        }
        else{
            rt_kprintf("fail\n");
        }
        return RT_EOK;
    }
    else if (strcmp(argv[1], "write_max_chip") == 0)
    {
        //config write_max_chip <id> <reg> <value> <len>
        if (argc != 6)
        {
            rt_kprintf("Usage: config write_max_chip <id(0-7|all)> <reg> <value> <len>\n");
            rt_kprintf("E.g. : config write_max_chip 0 0x0E 0x0200 2\n");
            rt_kprintf("       config write_max_chip all 0x14 0x0001 1\n");
            return -RT_ERROR;
        }

        if(strcmp(argv[5], "1") == 0){
            if(write_max_chip(argv[2], argv[3], argv[4], 1) != RT_EOK){
                return -RT_ERROR;
            }
        }
        else if(strcmp(argv[5], "2") == 0){
            if(write_max_chip(argv[2], argv[3], argv[4], 2) != RT_EOK){
                return -RT_ERROR;
            }
        }

        return RT_EOK;
    }
    else if (strcmp(argv[1], "read_max_chip") == 0)
    {
        //config read_max_chip <id> <reg> <len>
        if (argc != 5)
        {
            rt_kprintf("Usage: config read_max_chip <id(0-7)> <reg> <len>\n");
            rt_kprintf("E.g. : config read_max_chip 0 0x0E 2\n");
            return -RT_ERROR;
        }
        if(strcmp(argv[4], "1") == 0){
            if(read_max_chip(argv[2], argv[3],1) != RT_EOK){
                return -RT_ERROR;
            }
        }
        else if(strcmp(argv[4], "2") == 0){
            if(read_max_chip(argv[2], argv[3],2) != RT_EOK){
                return -RT_ERROR;
            }
        }

        return RT_EOK;
    }

    rt_kprintf("unknown subcommand\n");
    return -RT_ERROR;
}

MSH_CMD_EXPORT_ALIAS(finsh_cmd_config, config, configuration manager: config list|get|set|write_max_chip);
#endif


int config_thread_init(void)
{
    config_register("sample_rate",CONFIG_SAMPLE_RATE, &app_config.sample_rate, sizeof(app_config.sample_rate));
    config_register("adc_gain",CONFIG_ADC_GAIN, &app_config.adc_gain, sizeof(app_config.adc_gain));
    config_register("adc_enable_channel",CONFIG_ADC_ENABLE_CHANNEL, &app_config.adc_enable_channel, sizeof(app_config.adc_enable_channel));

    config_register("strain_S1",CONFIG_STRAIN_S1, &app_config.strain_S1, sizeof(app_config.strain_S1));
    config_register("epsilon_0",CONFIG_EPSILON_0, &app_config.epsilon_0, sizeof(app_config.epsilon_0));
    config_register("acceleration_S2",CONFIG_ACCELERATION_S2, &app_config.acceleration_S2, sizeof(app_config.acceleration_S2));
    config_register("a_0",CONFIG_A_0, &app_config.a_0, sizeof(app_config.a_0));
    config_register("adc_data_type",CONFIG_ADC_DATA_TYPE, &app_config.adc_data_type, sizeof(app_config.adc_data_type));

    config_register("enable_filter",CONFIG_ENABLE_FILTER, &app_config.enable_filter, sizeof(app_config.enable_filter));
    config_register("outlier_detection_method",CONFIG_OUTLIER_DETECTION_METHOD, &app_config.outlier_detection_method, sizeof(app_config.outlier_detection_method));
    config_register("filter_type",CONFIG_FILTER_TYPE, &app_config.filter_type, sizeof(app_config.filter_type));
    config_register("outlier_max",CONFIG_OUTLIER_MAX, &app_config.outlier_max, sizeof(app_config.outlier_max));
    config_register("outlier_min",CONFIG_OUTLIER_MIN, &app_config.outlier_min, sizeof(app_config.outlier_min));
    config_register("gradient_threshold",CONFIG_GRADIENT_THRESHLOD, &app_config.gradient_threshold, sizeof(app_config.gradient_threshold));
    config_register("n_sigma",CONFIG_N_SIGMA, &app_config.n_sigma, sizeof(app_config.n_sigma));
    config_register("low_pass_alpha",CONFIG_LOW_PASS_ALPHA, &app_config.low_pass_alpha, sizeof(app_config.low_pass_alpha));

    config_update_notify = rt_mq_create("config_update_notify", sizeof(config_update_msg_t), 10, RT_IPC_FLAG_FIFO);
    if (config_update_notify == RT_NULL) {
            rt_kprintf("Error: Failed to create mq config_update_notify.\n");
            return -RT_ERROR;
        }

    rt_kprintf("Configuration initialized (finsh commands active).\n");
    return RT_EOK;
}
