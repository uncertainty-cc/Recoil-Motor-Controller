/*
 * rath_core.h
 *
 *  Created on: Jan 31, 2023
 *      Author: TK
 */

#ifndef INC_RATH_CORE_H_
#define INC_RATH_CORE_H_


#define SET_BITS(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BITS(REG, BIT)                   ((REG) & (BIT))
#define WRITE_BITS(REG, CLEARMASK, SETMASK)   ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))


#endif /* INC_RATH_CORE_H_ */
