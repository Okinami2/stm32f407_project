#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ads131m08_hal.h"

#include "drv_spi.h"
#define DBG_TAG "ads131m08.hal"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static struct rt_spi_device *spi_dev;
rt_sem_t drdy_sem;

rt_uint8_t word_length = 24;

static int rt_hw_ads131m08_spi_init(void)
{
    rt_pin_mode(BSP_nADC_CS_PIN,PIN_MODE_OUTPUT);
    rt_err_t result = rt_hw_spi_device_attach("spi2", "spi20", BSP_nADC_CS_PORT, BSP_nADC_CS_PIN);
    if (result != RT_EOK) {
        return result;
    }
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_ads131m08_spi_init);

static void adc_drdy_isr(void *args)
{
    if (drdy_sem != RT_NULL) rt_sem_release(drdy_sem);
}

/**
 * @brief Write data to ADS131M08 register
 * @param addr 6-bit register address
 * @param data 16-bit data to write
 * @param word_length Transfer bit width (16/24/32)
 * @return RT_TRUE on success, RT_FALSE on failure
 */
rt_bool_t adcRegisterWrite(rt_uint8_t addr, rt_uint16_t data,rt_uint8_t word_length)
{
    rt_uint16_t command_word;
    rt_uint8_t tx_buffer[ADC_FRAME_WORD_COUNT * 4] = {0};
    rt_uint8_t rx_buffer[ADC_FRAME_WORD_COUNT * 4] = {0}; /* Response buffer */
    rt_uint8_t count = 3;

    /* Construct 16-bit WREG command word */
    /* Format: 011a aaaa a000 0000 (single register write, nnnnnn = 0) */
    command_word = (0b011 << 13) | ((addr & 0x3F) << 7)|0x00;

    if(word_length == 16){
        count = 2;
    }
    else if(word_length == 24){
        count = 3;
    }
    else if(word_length == 32){
        count = 4;
    }
    else{
        LOG_D("UNEXCEPTED WORD LENGTH");
    }

    tx_buffer[0] = (command_word >> 8) & 0xFF; /* Command high byte */
    tx_buffer[1] = command_word & 0xFF;        /* Command low byte */
    tx_buffer[count] = (data >> 8) & 0xFF;         /* Data high byte */
    tx_buffer[count + 1] = data & 0xFF;            /* Data low byte */

    /* Execute SPI transfer */
    if (rt_spi_transfer(spi_dev, tx_buffer, rx_buffer, ADC_FRAME_WORD_COUNT * count) != ADC_FRAME_WORD_COUNT * count) {
        LOG_E("SPI write transfer to reg 0x%02X failed!", addr);
        return RT_FALSE;
    }
    /*
    // Verify chip response - disabled to avoid issues during word length switching
    if (rt_spi_transfer(spi_dev, tx_buffer, rx_buffer, ADC_FRAME_WORD_COUNT * count) != ADC_FRAME_WORD_COUNT * count) {
            LOG_E("SPI write transfer to reg 0x%02X failed!", addr);
            return RT_FALSE;
    }

    rt_uint16_t expected_response_mask = (0b010 << 13) | ((addr & 0x3F) << 7);
    rt_uint16_t actual_response = (rx_buffer[0] << 8) | rx_buffer[1];

    if ((actual_response & 0xFFC0) != expected_response_mask) {
         LOG_W("Unexpected response 0x%04X for WREG to addr 0x%02X", actual_response, addr);
    }
    else{
        LOG_D("success response 0x%04X for WREG to addr 0x%02X", actual_response, addr);

    }
*/

    /* Clock sync */
    rt_pin_write(BSP_nADC_SYNC_PIN, PIN_LOW);
    rt_hw_us_delay(10);
    rt_pin_write(BSP_nADC_SYNC_PIN, PIN_HIGH);
    return RT_TRUE;
}


/**
 * @brief Read data from ADS131M08 register
 * @param addr 6-bit register address
 * @param data Pointer to store 16-bit data
 * @param word_length Transfer bit width (16/24/32)
 * @return RT_TRUE on success, RT_FALSE on failure
 */
rt_bool_t adcRegisterRead(rt_uint8_t addr, rt_uint16_t *data,rt_uint8_t word_length)
{
    rt_uint16_t command_word;
    rt_uint8_t tx_buffer[ADC_FRAME_WORD_COUNT * 4] = {0};
    rt_uint8_t rx_buffer[ADC_FRAME_WORD_COUNT * 4] = {0};
    rt_uint8_t count = 3;

    command_word = (0b101 << 13) | ((addr & 0x3F) << 7)|0x00;

    switch (word_length) {
            case 16: count = 2;break;
            case 24: count = 3;break;
            case 32: count = 4;break;
            default:LOG_E("UNEXCEPTED WORD LENGTH");break;
        }

    tx_buffer[0] = (command_word >> 8) & 0xFF;
    tx_buffer[1] = command_word & 0xFF;

    if (rt_spi_transfer(spi_dev, tx_buffer, rx_buffer, ADC_FRAME_WORD_COUNT * count) != ADC_FRAME_WORD_COUNT * count) {
        LOG_E("SPI read transfer to reg 0x%02X failed!", addr);
        return RT_FALSE;
    }
    rt_memset(tx_buffer, 0, sizeof(tx_buffer));
    if (rt_spi_transfer(spi_dev, tx_buffer, rx_buffer, ADC_FRAME_WORD_COUNT * count) != ADC_FRAME_WORD_COUNT * count) {
            LOG_E("SPI read transfer to reg 0x%02X failed!", addr);
            return RT_FALSE;
        }
    *data = (rx_buffer[0] << 8) | rx_buffer[1];

    return RT_TRUE;
}



/**
 * @brief Set ADC gain
 * @param gain Gain value (1, 2, 4, 8, 16, 32, 64, 128)
 * @return RT_EOK on success, error code on failure
 */
rt_err_t ads131m08_set_gain(rt_uint16_t gain)
{
    rt_uint16_t gain_code;
    rt_uint16_t register_value = 0x0000;
    switch (gain)
    {
        case 1: gain_code = 0b0000; break;
        case 2:   gain_code = 0b0001; break;
        case 4:   gain_code = 0b0010; break;
        case 8:   gain_code = 0b0011; break;
        case 16:   gain_code = 0b0100; break;
        case 32:  gain_code = 0b0101; break;
        case 64:  gain_code = 0b0110; break;
        case 128:  gain_code = 0b0111; break;
        default:
            rt_kprintf("[ADC] Error: Invalid gain value %d.\n", gain);
            return -RT_EINVAL;
    }
    register_value |= gain_code <<12 | gain_code << 8 | gain_code << 4 | gain_code;
    if (adcRegisterWrite(GAIN1_ADDR, register_value, word_length) != RT_TRUE) {
        return ADC_ERR_SET_GAIN;
    }
    if (adcRegisterWrite(GAIN2_ADDR, register_value, word_length) != RT_TRUE) {
        return ADC_ERR_SET_GAIN;
    }



    rt_kprintf("[ADC] success set gain = %d.\n", gain);
    return RT_EOK;
}

rt_err_t ads131m08_enable_cannel(rt_uint8_t channel_code)
{
    rt_uint16_t register_value = 0x0000;
    adcRegisterRead(CLOCK_ADDR, &register_value, word_length);
    register_value &= 0x00FF;
    register_value |= (channel_code<<8);
    if (adcRegisterWrite(CLOCK_ADDR, register_value, word_length) != RT_TRUE) {
        return RT_ERROR;
    }
    rt_kprintf("success set register_value %b\n", register_value);
    return RT_EOK;
}


/**
 * @brief Initialize ADS131M08 chip
 * @return RT_EOK on success, error code on failure
 */
rt_err_t ads131m08_init(void)
{
    /* 1. Hardware and SPI initialization */
    spi_dev = (struct rt_spi_device *)rt_device_find(ADC_SPI_DEVICE_NAME);
    if (!spi_dev) return ADC_ERR_DEVICE_FIND;
    if (rt_device_open((rt_device_t)spi_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) return ADC_ERR_DEVICE_OPEN;
    struct rt_spi_configuration cfg;
    cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.data_width = 8;
    cfg.max_hz = 25 * 1000 * 1000;
    if (rt_spi_configure(spi_dev, &cfg) != RT_EOK) return -RT_ERROR;


    drdy_sem = rt_sem_create("drdy_sem", 0, RT_IPC_FLAG_FIFO);
    if (drdy_sem == RT_NULL) {
        rt_kprintf("Error: Failed to create semaphore drdy_sem.\n");
        return -RT_ERROR;
    }

    /* 2. Clock, reset, DRDY setup */
    rt_pin_mode(BSP_ADCCLK_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_ADCCLK_BIT0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_ADCCLK_BIT1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_ADC_POWER_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_nADC_SYNC_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(BSP_ADCCLK_EN_PIN, PIN_LOW);
    rt_pin_write(BSP_ADC_POWER_EN_PIN, PIN_HIGH);
    #if (ADC_CLK_SELECTION == CLK_SEL_4_096_MHZ)
    rt_pin_write(BSP_ADCCLK_BIT0_PIN, PIN_LOW);
    rt_pin_write(BSP_ADCCLK_BIT1_PIN, PIN_HIGH);
    #elif (ADC_CLK_SELECTION == CLK_SEL_8_192_MHZ)
    rt_pin_write(BSP_ADCCLK_BIT0_PIN, PIN_HIGH);
    rt_pin_write(BSP_ADCCLK_BIT1_PIN, PIN_LOW);
    #elif (ADC_CLK_SELECTION == CLK_SEL_2_048_MHZ)
    rt_pin_write(BSP_ADCCLK_BIT0_PIN, PIN_HIGH);
    rt_pin_write(BSP_ADCCLK_BIT1_PIN, PIN_HIGH);
    #endif
    rt_hw_us_delay(5);
    rt_pin_write(BSP_nADC_SYNC_PIN, PIN_HIGH);

    /* Clock sync */
    rt_pin_write(BSP_nADC_SYNC_PIN, PIN_LOW);
    rt_hw_us_delay(10);
    rt_pin_write(BSP_nADC_SYNC_PIN, PIN_HIGH);
    /* Dummy read */
    if (adcRegisterWrite(STATUS_ADDR, 0x0000, word_length) != RT_TRUE) {
       return ADC_ERR_REG_WRITE_MODE;
    }

    /* Configure CLOCK register: Set OSR and power mode, 4096 for 1ksps at F_clkin=8196kHz */
    if (adcRegisterWrite(CLOCK_ADDR, 0xFF16, word_length) != RT_TRUE) {
        return ADC_ERR_REG_WRITE_CLK;
    }
    /* Configure gain registers: Set default gain to 1 */
    if (adcRegisterWrite(GAIN1_ADDR, 0x0000, word_length) != RT_TRUE) {
        return ADC_ERR_SET_GAIN;
    }
    if (adcRegisterWrite(GAIN2_ADDR, 0x0000, word_length) != RT_TRUE) {
        return ADC_ERR_SET_GAIN;
    }
    /* Clear reset bit, set DRDY low pulse mode */
    if (adcRegisterWrite(MODE_ADDR, 0X0115, word_length) != RT_TRUE) {
       return ADC_ERR_REG_WRITE_MODE;
    }

    rt_pin_mode(BSP_nADC_DRDY_PIN,PIN_MODE_INPUT);

    rt_pin_attach_irq(BSP_nADC_DRDY_PIN, PIN_IRQ_MODE_FALLING, adc_drdy_isr, RT_NULL);
    /* Disable IRQ until data read starts */
    rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_DISABLE);

    LOG_I("ADS131M08 initialized successfully. Ready for data streaming.");
    return ADC_EOK;
}



rt_err_t ads131m08_read_data_frame(rt_int32_t *buffer, rt_bool_t is_first_read)
{
    rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_DISABLE);
    rt_uint8_t dummy_tx[ADC_FRAME_WORD_COUNT * 4] = {0};
    rt_uint8_t rx_buf[ADC_FRAME_WORD_COUNT * 4] = {0};

    rt_size_t transfer_len = ADC_FRAME_WORD_COUNT * (word_length / 8);

    if (is_first_read) {
        if (rt_spi_transfer(spi_dev, dummy_tx, rx_buf, transfer_len) != transfer_len) return -RT_ERROR;
        if (rt_spi_transfer(spi_dev, dummy_tx, rx_buf, transfer_len) != transfer_len) return -RT_ERROR;
    } else {
        if (rt_spi_transfer(spi_dev, dummy_tx, rx_buf, transfer_len) != transfer_len) return -RT_ERROR;
    }
    // 如果时间精度不够可以把数据处理放到外面
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        int offset = (word_length / 8) * (i + 1);

        buffer[i] = (rt_int32_t)(((rx_buf[offset] << 16) | (rx_buf[offset + 1] << 8) | rx_buf[offset + 2]) << 8) >> 8; /* Sign extend */
    }

    rt_pin_irq_enable(BSP_nADC_DRDY_PIN, PIN_IRQ_ENABLE);
    return RT_EOK;
}
