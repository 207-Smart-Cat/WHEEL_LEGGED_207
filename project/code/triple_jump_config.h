#ifndef TRIPLE_JUMP_CONFIG_H
#define TRIPLE_JUMP_CONFIG_H

#include <stdint.h>

#include "triple_jump.h"

void TripleJumpConfig_SetDefaults(TripleJumpConfig_t *config);
uint32_t TripleJumpConfig_CalculateCrc(const TripleJumpConfig_t *config);
uint8_t TripleJumpConfig_Load(TripleJumpConfig_t *config);
uint8_t TripleJumpConfig_Save(const TripleJumpConfig_t *config);

#endif
