/* ============================================================================
 * STM32F401CCU6 + VL53L5CX (Polling, zone-major, 8x8 center-pooling upsample)
 * - Res: 4x4 or 8x8 (macro toggle)
 * - 8x8: 2x2 center-pooling(유효값 SNR 가중 평균: sps)로 4x4 SNR 확보 후 8x8로 복제
 * - Ranging: 10 Hz
 * - Logging: tidy(9열) N프레임마다, 그 외는 레거시 한 줄(거리맵)
 * - 주의: vl53l5cx_buffers.h 를 어떤 .c에도 include하지 말 것(링크 충돌 방지)
 * ========================================================================== */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "vl53l5cx_platform.h"
#include "vl53l5cx_api.h"

/* ========= 사용자 설정 ========= */
/* 해상도: 0=4x4, 1=8x8 */
#define USE_RES_8X8            1

/* I2C 속도: 0=400kHz(권장), 1=1MHz(Fm+ 보드/배선 검증 시) */
#define USE_I2C_FMP_1MHZ       0

/* 프레임 주파수(Hz) — 요구사항 10 Hz */
#define RANGING_HZ             10

/* 로그 정책 */
#define PRINT_TIDY_EVERY       0   /* tidy 출력 간격(프레임). 0이면 tidy 비활성화 */
#define PRINT_LEGACY_ON_OTHERS 1    /* 나머지 프레임은 레거시 한 줄 출력 */

/* 품질 필터(장거리 완화) */
#define SIGMA_MAX_MM_SHORT     20
#define SIGMA_MAX_MM_LONG      35
#define SPS_MIN_RAW_BASE       1024 /* ≈0.5 kcps/spad (Q9.7 raw 단위) */
#define STATUS_OK_1            5
#define STATUS_OK_2            6
#define STATUS_OK_3            9
#define STATUS_OK_4            12

/* 폴링 대기 한계(10Hz ≈ 100ms/프레임) */
#define POLL_WAIT_TIMEOUT_MS   110

/* 8x8 모드 전용: center-pooling 사용 */
#define USE_CENTER_POOLING_8X8 1

/* 핀맵: LPn(XSHUT)=PA1, UART1=PA9/PA10, I2C1=PB6/PB7 */
#define TOF_LPN_GPIO_Port      GPIOA
#define TOF_LPN_Pin            GPIO_PIN_1

/* 8-bit I2C 주소(0x29 << 1) */
#define VL53L5CX_I2C_ADDR_8BIT 0x52

/* ========= HAL 핸들 ========= */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* ========= 전방 선언 ========= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* ========= 유틸 ========= */
static inline bool status_is_ok(uint8_t st) {
  return (st==STATUS_OK_1 || st==STATUS_OK_2 || st==STATUS_OK_3 || st==STATUS_OK_4);
}

int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

static void I2C_Scan(I2C_HandleTypeDef *hi2c) {
  printf("I2C scan...");
  int found = 0;
  for (uint8_t a7 = 0x08; a7 <= 0x77; a7++) {
    uint16_t a8 = (uint16_t)(a7 << 1);
    if (HAL_I2C_IsDeviceReady(hi2c, a8, 1, 5) == HAL_OK) {
      printf(" 0x%02X", a8);
      found = 1;
    }
  }
  if (!found) printf(" (none)");
  printf("\r\n");
}

static void TOF_Reset(void) {
  HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_SET);
  HAL_Delay(150);
}

/* 레거시 한 줄(거리만) 출력(유효성 필터 적용, idx=zone*NB_TARGET + t) */
static void print_legacy_line(const VL53L5CX_ResultsData *r, uint16_t nb_zones) {
  const uint8_t N = VL53L5CX_NB_TARGET_PER_ZONE;
  for (uint16_t z = 0; z < nb_zones; z++) {
    uint32_t idx = (uint32_t)z * N + 0; /* t=0 대표 (zone-major) */
    uint16_t dist  = r->distance_mm[idx];
    uint32_t sps   = r->signal_per_spad[idx];
    uint16_t sigma = r->range_sigma_mm[idx];
    uint8_t  st    = r->target_status[idx];

    uint32_t ambient = r->ambient_per_spad[z];
    uint32_t sps_thr = (ambient > 0) ? (ambient*2) : SPS_MIN_RAW_BASE;
    bool good = status_is_ok(st)
                && (sigma <= ((dist>1000)?SIGMA_MAX_MM_LONG:SIGMA_MAX_MM_SHORT))
                && (sps >= sps_thr)
                && (dist > 0);
    if (!good) dist = 0;

    printf("%u%s", (unsigned)dist, (z == nb_zones - 1) ? "\r\n" : ", ");
  }
}

/* 8x8 → 4x4(2x2 pooling, sps 가중 평균) → 8x8(2x2 복제) */
static void pooled_legacy_8x8(const VL53L5CX_ResultsData *r, uint16_t out64[64]) {
#if USE_CENTER_POOLING_8X8
  const uint8_t N = VL53L5CX_NB_TARGET_PER_ZONE;
  uint16_t coarse4x4[16] = {0};

  for (uint8_t R4=0; R4<4; R4++) {
    for (uint8_t C4=0; C4<4; C4++) {
      uint32_t wsum = 0, dsum = 0;
      for (uint8_t dr=0; dr<2; dr++) {
        for (uint8_t dc=0; dc<2; dc++) {
          uint8_t r8 = (uint8_t)(R4*2 + dr);
          uint8_t c8 = (uint8_t)(C4*2 + dc);
          uint16_t z8 = r8*8 + c8;
          uint32_t idx = (uint32_t)z8 * N + 0; /* t=0, zone-major */

          uint16_t dist  = r->distance_mm[idx];
          uint32_t sps   = r->signal_per_spad[idx];
          uint16_t sigma = r->range_sigma_mm[idx];
          uint8_t  st    = r->target_status[idx];
          uint32_t ambient = r->ambient_per_spad[z8];
          uint32_t sps_thr = (ambient > 0) ? (ambient*2) : SPS_MIN_RAW_BASE;

          bool good = status_is_ok(st)
                      && (sigma <= ((dist>1000)?SIGMA_MAX_MM_LONG:SIGMA_MAX_MM_SHORT))
                      && (sps >= sps_thr)
                      && (dist > 0);
          if (good) {
            uint32_t w = (sps > 0) ? sps : 1;
            dsum += (uint32_t)dist * w;
            wsum += w;
          }
        }
      }
      coarse4x4[R4*4 + C4] = (wsum>0) ? (uint16_t)(dsum / wsum) : 0;
    }
  }

  /* 4x4 값을 8x8로 2x2 복제 */
  for (uint8_t R4=0; R4<4; R4++) {
    for (uint8_t C4=0; C4<4; C4++) {
      uint16_t v = coarse4x4[R4*4 + C4];
      for (uint8_t dr=0; dr<2; dr++) {
        for (uint8_t dc=0; dc<2; dc++) {
          uint8_t r8 = (uint8_t)(R4*2 + dr);
          uint8_t c8 = (uint8_t)(C4*2 + dc);
          out64[r8*8 + c8] = v;
        }
      }
    }
  }
#else
  const uint8_t N = VL53L5CX_NB_TARGET_PER_ZONE;
  for (uint16_t z=0; z<64; z++) {
    uint32_t idx = (uint32_t)z * N + 0;
    out64[z] = r->distance_mm[idx];
  }
#endif
}

/* ========= 메인 ========= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* UART 버퍼링 해제(즉시 전송) */
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n=== VL53L5CX start (Polling, %s, %uHz) ===\r\n",
         USE_RES_8X8 ? "8x8(pool-then-upsample)" : "4x4",
         (unsigned)RANGING_HZ);

  TOF_Reset();
  I2C_Scan(&hi2c1);

  VL53L5CX_Configuration dev;
  memset(&dev, 0, sizeof(dev));
  dev.platform.hi2c    = &hi2c1;
  dev.platform.address = VL53L5CX_I2C_ADDR_8BIT;

  /* alive */
  uint8_t alive = 0;
  for (int i=0; i<50; i++) {
    if (vl53l5cx_is_alive(&dev, &alive) == 0 && alive) break;
    HAL_Delay(10);
  }
  if (!alive) { printf("ERR: is_alive=0\r\n"); Error_Handler(); }
  printf("OK : is_alive\r\n");

  /* init(재시도 포함) */
  for (int i=0; i<3; i++) {
    uint8_t st = vl53l5cx_init(&dev);
    if (st == 0) { printf("OK : init\r\n"); break; }
    printf("WARN: init fail st=%u, reset & retry\r\n", st);
    TOF_Reset();
    if (i==2) { printf("ERR: init final fail\r\n"); Error_Handler(); }
  }

  /* 해상도/주파수 */
  vl53l5cx_set_resolution(&dev, USE_RES_8X8 ? VL53L5CX_RESOLUTION_8X8
                                            : VL53L5CX_RESOLUTION_4X4);
  vl53l5cx_set_ranging_frequency_hz(&dev, RANGING_HZ);

  /* 튜닝 */
  vl53l5cx_set_target_order(&dev, VL53L5CX_TARGET_ORDER_STRONGEST);
  vl53l5cx_set_sharpener_percent(&dev, 10);
  vl53l5cx_set_VHV_repeat_count(&dev, 50);

  /* 시작 */
  {
    uint8_t st = vl53l5cx_start_ranging(&dev);
    printf("start st=%u\r\n", st);
    if (st) Error_Handler();
  }

  /* 파라미터 */
  uint8_t  res = 0;
  (void)vl53l5cx_get_resolution(&dev, &res);
  uint16_t nb_zones = (res==VL53L5CX_RESOLUTION_4X4) ? 16 : 64;
  const uint8_t N   = VL53L5CX_NB_TARGET_PER_ZONE;
  printf("res=%u nb_zones=%u N=%u\r\n", res, nb_zones, N);
  printf("=== ranging loop ===\r\n");

  /* 결과 버퍼 */
  VL53L5CX_ResultsData rbuf;
  VL53L5CX_ResultsData *r = &rbuf;
  uint32_t frame = 0;

  while (1) {
    uint8_t ready = 0;
    uint32_t t0 = HAL_GetTick();
    do {
      if (vl53l5cx_check_data_ready(&dev, &ready) == 0 && ready) break;
      if ((HAL_GetTick() - t0) > POLL_WAIT_TIMEOUT_MS) break;
      HAL_Delay(1);
    } while (!ready);
    if (!ready) {
      /* 디버그용 하트비트(필요 없으면 주석) */
      // printf("beat (not ready) t=%lu ms\r\n", (unsigned long)HAL_GetTick());
      continue;
    }

    if (vl53l5cx_get_ranging_data(&dev, r) != 0) continue;
    frame++;

    /* tidy 활성화 여부(0이면 tidy 완전 비활성) */
    bool do_tidy = (PRINT_TIDY_EVERY > 0) && ((frame % PRINT_TIDY_EVERY) == 0);

    if (do_tidy) {
      /* tidy: (frame, zone, t, dist, sps, sigma, ambient, spads, status) */
      for (uint16_t z = 0; z < nb_zones; z++) {
        for (uint8_t t = 0; t < N; t++) {
          uint32_t idx   = (uint32_t)z * N + t; /* zone-major */
          uint16_t dist  = r->distance_mm[idx];
          uint32_t sps   = r->signal_per_spad[idx];
          uint16_t sigma = r->range_sigma_mm[idx];
          uint16_t amb   = r->ambient_per_spad[z];
          uint16_t spads = r->nb_spads_enabled[z];
          uint8_t  stat  = r->target_status[idx];
          printf("%lu,%u,%u,%u,%lu,%u,%u,%u,%u\r\n",
                 (unsigned long)frame,
                 (unsigned)z, (unsigned)t,
                 (unsigned)dist,
                 (unsigned long)sps,
                 (unsigned)sigma,
                 (unsigned)amb,
                 (unsigned)spads,
                 (unsigned)stat);
        }
      }
    } else {
      /* do_tidy가 아니면 항상 레거시 한 줄을 출력(PRINT_LEGACY_ON_OTHERS=0이더라도 최소 한 줄 보장) */
      if (USE_RES_8X8) {
        uint16_t map64[64];
        pooled_legacy_8x8(r, map64);
        for (int i=0;i<64;i++) {
          printf("%u%s", (unsigned)map64[i], (i==63) ? "\r\n" : ", ");
        }
      } else {
        print_legacy_line(r, nb_zones);
      }
    }
  }
}

/* ========= 클럭/I2C/UART/GPIO ========= */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void) {
  __HAL_RCC_I2C1_CLK_ENABLE();

#if USE_I2C_FMP_1MHZ
  #ifdef I2C_FASTMODEPLUS_I2C1
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  #endif
  #ifdef I2C_FASTMODEPLUS_PB6
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB6);
  #endif
  #ifdef I2C_FASTMODEPLUS_PB7
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB7);
  #endif
#endif

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = (USE_I2C_FMP_1MHZ ? 1000000 : 400000);
  hi2c1.Init.DutyCycle  = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* LPn(XSHUT) */
  GPIO_InitStruct.Pin = TOF_LPN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_LPN_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  /* USART1: TX=PA9, RX=PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* I2C1: SCL=PB6, SDA=PB7 (외부 풀업 권장) */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* DRDY/INT 미사용(폴링) */
}

void Error_Handler(void) {
  __disable_irq();
  while (1) { }
}
