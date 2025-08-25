#include "vl53l5cx_platform.h"
#include <stdio.h>
#include <string.h>

/* I2C 타임아웃(ms) 및 대용량 전송 청크 크기 */
#define L5_I2C_TIMEOUT   1000U
#define L5_FW_CHUNK_SIZE 240U   /* HAL I2C NBYTES 한계(<=255) 고려 */

/* ---- 저수준 I2C ---- */
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *p_data, uint32_t size)
{
    uint32_t offset = 0;
    const uint8_t addr = (uint8_t)p->address;  /* 8-bit */

    while (offset < size) {
        uint32_t chunk = (size - offset > L5_FW_CHUNK_SIZE) ? L5_FW_CHUNK_SIZE : (size - offset);
        uint16_t cur = reg + (uint16_t)offset;
        uint8_t buf[L5_FW_CHUNK_SIZE + 2];

        buf[0] = (uint8_t)((cur >> 8) & 0xFF);
        buf[1] = (uint8_t)( cur       & 0xFF);
        memcpy(&buf[2], &p_data[offset], chunk);

        if (HAL_I2C_Master_Transmit(p->hi2c, addr, buf, (uint16_t)(chunk + 2), L5_I2C_TIMEOUT) != HAL_OK) {
            printf("[L5][I2C ERR][WrMulti] Addr:0x%02X Reg:0x%04X Off:%lu Len:%lu\r\n",
                   addr, cur, (unsigned long)offset, (unsigned long)chunk);
            return 255;
        }
        offset += chunk;
    }
    return 0;
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *p_data, uint32_t size)
{
    const uint8_t addr = (uint8_t)p->address;
    uint8_t regbuf[2] = { (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF) };

    if (HAL_I2C_Master_Transmit(p->hi2c, addr, regbuf, 2, L5_I2C_TIMEOUT) != HAL_OK) {
        printf("[L5][I2C ERR][RdMulti-Addr] Addr:0x%02X Reg:0x%04X\r\n", addr, reg);
        return 255;
    }
    if (HAL_I2C_Master_Receive(p->hi2c, addr, p_data, (uint16_t)size, L5_I2C_TIMEOUT) != HAL_OK) {
        printf("[L5][I2C ERR][RdMulti-Data] Addr:0x%02X Reg:0x%04X Len:%lu\r\n",
               addr, reg, (unsigned long)size);
        return 255;
    }
    return 0;
}

/* ---- 편의 래퍼 ---- */
uint8_t VL53L5CX_WrByte (VL53L5CX_Platform *p, uint16_t reg, uint8_t data)  { return VL53L5CX_WrMulti(p, reg, &data, 1); }

uint8_t VL53L5CX_WrWord (VL53L5CX_Platform *p, uint16_t reg, uint16_t data) {
    uint8_t b[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L5CX_WrMulti(p, reg, b, 2);
}

uint8_t VL53L5CX_WrDWord(VL53L5CX_Platform *p, uint16_t reg, uint32_t data)  {
    uint8_t b[4] = { (uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data };
    return VL53L5CX_WrMulti(p, reg, b, 4);
}

uint8_t VL53L5CX_RdByte (VL53L5CX_Platform *p, uint16_t reg, uint8_t *v)   { return VL53L5CX_RdMulti(p, reg, v, 1); }

uint8_t VL53L5CX_RdWord (VL53L5CX_Platform *p, uint16_t reg, uint16_t *v)  {
    uint8_t b[2];
    if (VL53L5CX_RdMulti(p, reg, b, 2)) return 255;
    *v = (uint16_t)((b[0] << 8) | b[1]);
    return 0;
}

uint8_t VL53L5CX_RdDWord(VL53L5CX_Platform *p, uint16_t reg, uint32_t *v)  {
    uint8_t b[4];
    if (VL53L5CX_RdMulti(p, reg, b, 4)) return 255;
    *v = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
    return 0;
}

/* ---- 공용 유틸 ---- */
uint8_t VL53L5CX_SwapBuffer(uint8_t *buf, uint16_t size)
{
    for (uint16_t i = 0; i < size; i += 4) {
        uint8_t t = buf[i];       buf[i] = buf[i+3]; buf[i+3] = t;
        t = buf[i+1];             buf[i+1] = buf[i+2]; buf[i+2] = t;
    }
    return 0;
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p, uint32_t ms) { (void)p; HAL_Delay(ms); return 0; }

uint8_t VL53L5CX_WaitUs(VL53L5CX_Platform *p, uint32_t us)
{
    (void)p;
    const uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    const uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks) { /* busy wait */ }
    return 0;
}

uint8_t VL53L5CX_GetTick(VL53L5CX_Platform *p, uint32_t *p_ms) { (void)p; *p_ms = HAL_GetTick(); return 0; }

/* LPN(저전력) 핀 토글로 리셋 */
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p)
{
    (void)p;
    HAL_GPIO_WritePin(VL53L5CX_LPN_GPIO_Port, VL53L5CX_LPN_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(VL53L5CX_LPN_GPIO_Port, VL53L5CX_LPN_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    return 0;
}

/* 선택: 초기화/해제 (필요 시 확장) */
uint8_t VL53L5CX_Platform_Init  (VL53L5CX_Platform *p){ (void)p; return 0; }
uint8_t VL53L5CX_Platform_Deinit(VL53L5CX_Platform *p){ (void)p; return 0; }
