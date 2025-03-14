#include "ads1220.h"



void ads_spi_xfer(Ads1220_t* adc, uint8_t* data, int len){
    if (adc->hspi->Init.CLKPhase != SPI_PHASE_2EDGE){
        adc->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
        HAL_SPI_Init(adc->hspi);
    }
    adc->set_cs(0);
    HAL_SPI_TransmitReceive(adc->hspi, data, data, len, 25);
    adc->set_cs(1);
}


void ads_reset(Ads1220_t* adc){
    uint8_t data = ADS1220_CMD_RESET;
    ads_spi_xfer(adc, &data, 1);
}


void ads_begin(Ads1220_t* adc, uint32_t config){
    adc->init_config = config;

    uint8_t data[5];
    data[0] = ADS1220_CMD_WREG | 3; // starts at register 0
    data[1] = (adc->init_config >> 24) & 0xFF;
    data[2] = (adc->init_config >> 16) & 0xFF;
    data[3] = (adc->init_config >> 8) & 0xFF;
    data[4] = adc->init_config & 0xFF;

    ads_spi_xfer(adc, data, 5);
}


void ads_start_sync(Ads1220_t* adc){
    uint8_t data = ADS1220_CMD_START_SYNC;
    ads_spi_xfer(adc, &data, 1);
}


void ads_set_input_mux(Ads1220_t* adc, uint8_t setting){
    uint8_t buf[2];
    buf[0] = ADS1220_CMD_RREG;
    buf[1] = 0;

    ads_spi_xfer(adc, buf, 2);

    adc->mux_setting = setting;

    buf[1] &= 0x0F; // clear mux bits [7:4]
    buf[1] |= adc->mux_setting & 0xF0;
    buf[0] = ADS1220_CMD_WREG; // rr = 0, nn = 0

    ads_spi_xfer(adc, buf, 2);
}


float ads_get_voltage(Ads1220_t* adc){
    uint8_t buf[3];

    buf[0] = ADS1220_CMD_RDATA;

    ads_spi_xfer(adc, buf, 4);

    int32_t conv = (buf[1] << 16) | (buf[2] << 8) | buf[3];

    if (conv >= 0){
        return (adc->vref*conv)/ADS1220_FS_COUNTS_POS;
    } else {
        return (adc->vref*conv)/ADS1220_FS_COUNTS_NEG;
    }
}