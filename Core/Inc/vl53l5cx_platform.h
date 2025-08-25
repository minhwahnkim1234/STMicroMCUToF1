#ifndef VL53L5CX_PLATFORM_H_
#define VL53L5CX_PLATFORM_H_

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* 기본 8-bit I2C 주소 (ST 7-bit 0x29 → 8-bit 0x52) */
#ifndef VL53L5CX_I2C_ADDR
#define VL53L5CX_I2C_ADDR  (0x52U)
#endif

/* 리셋 핀(저전력/LPN) 정의: 보드에 맞게 필요 시 프로젝트 전역에서 재정의 */
#ifndef VL53L5CX_LPN_GPIO_Port
#define VL53L5CX_LPN_GPIO_Port GPIOA
#endif
#ifndef VL53L5CX_LPN_Pin
#define VL53L5CX_LPN_Pin      GPIO_PIN_1
#endif

typedef struct {
    I2C_HandleTypeDef *hi2c;   /* 사용 I2C 핸들 */
    uint16_t address;          /* 8-bit I2C 주소 (예: 0x52) */
} VL53L5CX_Platform;

/* 저수준 I2C */
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *p_data, uint32_t size);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *p_data, uint32_t size);

/* 편의 래퍼 */
uint8_t VL53L5CX_WrByte (VL53L5CX_Platform *p, uint16_t reg, uint8_t data);
uint8_t VL53L5CX_WrWord (VL53L5CX_Platform *p, uint16_t reg, uint16_t data);
uint8_t VL53L5CX_WrDWord(VL53L5CX_Platform *p, uint16_t reg, uint32_t data);

uint8_t VL53L5CX_RdByte (VL53L5CX_Platform *p, uint16_t reg, uint8_t  *p_value);
uint8_t VL53L5CX_RdWord (VL53L5CX_Platform *p, uint16_t reg, uint16_t *p_value);
uint8_t VL53L5CX_RdDWord(VL53L5CX_Platform *p, uint16_t reg, uint32_t *p_value);

/* 공용 유틸 */
uint8_t VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L5CX_WaitMs    (VL53L5CX_Platform *p, uint32_t time_ms);
uint8_t VL53L5CX_WaitUs    (VL53L5CX_Platform *p, uint32_t time_us);
uint8_t VL53L5CX_GetTick   (VL53L5CX_Platform *p, uint32_t *p_time_ms);
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p);

/* 선택: 플래폼 초기화/해제 (필요 시 사용) */
uint8_t VL53L5CX_Platform_Init  (VL53L5CX_Platform *p);
uint8_t VL53L5CX_Platform_Deinit(VL53L5CX_Platform *p);

#endif /* VL53L5CX_PLATFORM_H_ */
