#pragma once

// reg_helpers.h - STM32 register bit manipulation helpers
// Add any macros or inline functions needed for register access here.

// Example: Set a bit
#ifndef SET_BIT
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#endif

// Example: Clear a bit
#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#endif

// Example: Read a bit
#ifndef READ_BIT
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#endif

// Example: Write a value to a register
#ifndef WRITE_REG
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#endif

// Example: Modify a register
#ifndef MODIFY_REG
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
    WRITE_REG((REG), (((READ_BIT((REG), ~(CLEARMASK))) | (SETMASK))))
#endif

// Example: Set a register field (bitfield)
#ifndef SET_REG
#define SET_REG(REG, MASK, VAL) ((REG) = ((REG) & ~(MASK)) | ((VAL) & (MASK)))
#endif
