#include "triple_jump_config.h"

#include <stddef.h>

#define TRIPLE_JUMP_CONFIG_MAGIC        (0x4A554D50UL)
#define TRIPLE_JUMP_CONFIG_VERSION      (1UL)

void TripleJumpConfig_SetDefaults(TripleJumpConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }
    config->x1_m = 0.50f;
    config->x2_m = 0.15f;
    config->x3_m = 0.15f;
    config->speed = 0.0f;
}

uint32_t TripleJumpConfig_CalculateCrc(const TripleJumpConfig_t *config)
{
    const uint8_t *bytes = (const uint8_t *)config;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;

    if (config == NULL)
    {
        return 0U;
    }
    for (i = 0U; i < (uint32_t)sizeof(*config); ++i)
    {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    return ~crc;
}

#ifndef TRIPLE_JUMP_CONFIG_HOST_TEST

#include "zf_common_headfile.h"

#define TRIPLE_JUMP_FLASH_SECTION      (0U)
#define TRIPLE_JUMP_FLASH_PAGE         (94U)
#define TRIPLE_JUMP_FLASH_WORDS        (7U)

uint8_t TripleJumpConfig_Load(TripleJumpConfig_t *config)
{
    TripleJumpConfig_t loaded;
    uint32_t stored_crc;

    if (config == NULL)
    {
        return 0U;
    }

    flash_read_page_to_buffer(TRIPLE_JUMP_FLASH_SECTION,
                              TRIPLE_JUMP_FLASH_PAGE,
                              TRIPLE_JUMP_FLASH_WORDS);
    loaded.x1_m = flash_union_buffer[2].float_type;
    loaded.x2_m = flash_union_buffer[3].float_type;
    loaded.x3_m = flash_union_buffer[4].float_type;
    loaded.speed = flash_union_buffer[5].float_type;
    stored_crc = flash_union_buffer[6].uint32_type;

    if (flash_union_buffer[0].uint32_type != TRIPLE_JUMP_CONFIG_MAGIC ||
        flash_union_buffer[1].uint32_type != TRIPLE_JUMP_CONFIG_VERSION ||
        !TripleJump_ConfigIsValid(&loaded) ||
        stored_crc != TripleJumpConfig_CalculateCrc(&loaded))
    {
        TripleJumpConfig_SetDefaults(config);
        return 0U;
    }

    *config = loaded;
    return 1U;
}

uint8_t TripleJumpConfig_Save(const TripleJumpConfig_t *config)
{
    if (!TripleJump_ConfigIsValid(config))
    {
        return 0U;
    }

    __disable_irq();
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = TRIPLE_JUMP_CONFIG_MAGIC;
    flash_union_buffer[1].uint32_type = TRIPLE_JUMP_CONFIG_VERSION;
    flash_union_buffer[2].float_type = config->x1_m;
    flash_union_buffer[3].float_type = config->x2_m;
    flash_union_buffer[4].float_type = config->x3_m;
    flash_union_buffer[5].float_type = config->speed;
    flash_union_buffer[6].uint32_type = TripleJumpConfig_CalculateCrc(config);

    if (flash_check(TRIPLE_JUMP_FLASH_SECTION, TRIPLE_JUMP_FLASH_PAGE))
    {
        flash_erase_page(TRIPLE_JUMP_FLASH_SECTION, TRIPLE_JUMP_FLASH_PAGE);
    }
    flash_write_page_from_buffer(TRIPLE_JUMP_FLASH_SECTION,
                                 TRIPLE_JUMP_FLASH_PAGE,
                                 TRIPLE_JUMP_FLASH_WORDS);
    __enable_irq();
    return 1U;
}

#endif
