/*
 * dmac_settings.h
 *
 *  Created on: Jan 23, 2024
 *      Author: a5138119
 */

#include "hal_data.h"
#include "stdint.h"

#ifndef DMAC_SETTINGS_H_
#define DMAC_SETTINGS_H_

//For SRC_MATRIX_X_SIZE
//  10 is 790ns
//  20 is 1.56us
//  640 is 48us

#define SRAM    1
#define SDRAM   2
#define SRC_LOC SRAM
#define DST_LOC SDRAM

//VGA Resolution
#define SRC_MATRIX_X_SIZE 640
#define SRC_MATRIX_Y_SIZE 480


#define DST_MATRIX_X_SIZE SRC_MATRIX_Y_SIZE
#define DST_MATRIX_Y_SIZE SRC_MATRIX_X_SIZE

#define INTERMEDIATE_MATRIX_BUFFER_SIZE 2
#define INTERMEDIATE_MATRIX_ROW_SIZE 48  //WARNING: This has to an even divisor of SRC_MATRIX_Y_SIZE


#define MATRIX_ELEMENT_SIZE TRANSFER_SIZE_2_BYTE //adjust to match the size of the src_matrix

void g_transfer0_cb( dmac_callback_args_t * p_args );


#endif /* DMAC_SETTINGS_H_ */
