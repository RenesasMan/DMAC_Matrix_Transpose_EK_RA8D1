/*
 * dmac_settings.c
 *
 *  Created on: Jan 23, 2024
 *      Author: a5138119
 */

#include "dmac_settings.h"
#include "hal_data.h"
#include <stdint.h>

bool g_is_transfer_complete = false;
uint32_t scan_count = 0U;
uint8_t intermediate_buffer_index = 0;

// Check caching on SDRAM
// Probe SDRAM signals CAS and RAS check conditions against destination and location of memory
// Can I grab each data frame from the camera and directly transpose it

#if (SRC_LOC == SDRAM)
uint16_t src_matrix[SRC_MATRIX_X_SIZE][SRC_MATRIX_Y_SIZE] BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(8);
#else
uint16_t src_matrix[SRC_MATRIX_X_SIZE][SRC_MATRIX_Y_SIZE];
#endif

//Intermediate matrix location is always located in SRAM. It shares the X size with the src matrix
uint16_t intermediate_matrix[INTERMEDIATE_MATRIX_BUFFER_SIZE][SRC_MATRIX_X_SIZE][INTERMEDIATE_MATRIX_ROW_SIZE];


#if (DST_LOC == SDRAM)
uint16_t dst_matrix[DST_MATRIX_X_SIZE][DST_MATRIX_Y_SIZE] BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(8);
#else
uint16_t dst_matrix[DST_MATRIX_X_SIZE][DST_MATRIX_Y_SIZE];
#endif



#ifndef APP_ERR_TRAP
#define APP_ERR_TRAP(err)        ({\
                                    if((err)) {\
                                         __asm("BKPT #0\n");} /* trap upon the error  */\
                                    })
#endif

dmac_instance_ctrl_t g_transfer_new_ctrl;
transfer_info_t p_info_new =
{
  .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,    // DMAMD.DM is increment
  .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE,          // repeat area is source
  .transfer_settings_word_b.irq = TRANSFER_IRQ_END,                             // IRQ on transfer complete
  .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,          //
  .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_OFFSET,          // DMAMD.SM is offset
  .transfer_settings_word_b.size = MATRIX_ELEMENT_SIZE,//TRANSFER_SIZE_4_BYTE,                        // transfer size is 32-bit/4byte
  .transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT,//TRANSFER_MODE_REPEAT_BLOCK,
  .p_dest = (void*) intermediate_matrix,
  .p_src = (void const*) src_matrix,
  .num_blocks = 1,
  .length = SRC_MATRIX_X_SIZE, };

const dmac_extended_cfg_t p_extended_new =
{
 .offset = (SRC_MATRIX_Y_SIZE)*sizeof(src_matrix[0][0]), .src_buffer_size = 1,
 #if defined(VECTOR_NUMBER_DMAC0_INT)
     .irq                 = VECTOR_NUMBER_DMAC0_INT,
 #else
   .irq = FSP_INVALID_VECTOR,
 #endif
   .ipl = (7),
   .channel = 0, .p_callback = g_transfer0_cb, .p_context = NULL, .activation_source = ELC_EVENT_NONE,
};

const transfer_cfg_t g_transfer_new_cfg =
{ .p_info = &p_info_new, .p_extend = &p_extended_new, };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer_new =
{ .p_ctrl = &g_transfer_new_ctrl, .p_cfg = &g_transfer_new_cfg, .p_api = &g_transfer_on_dmac };

void g_transfer0_cb( dmac_callback_args_t * p_args ) //args and events
{
    FSP_PARAMETER_NOT_USED(p_args);
    fsp_err_t err;
    R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED1_BLUE, BSP_IO_LEVEL_HIGH );
    //increment scan count
    scan_count++;

    //Close current DMA session
    err = R_DMAC_Close(&g_transfer_new_ctrl);
    APP_ERR_TRAP(err);

    //else if the scan is the very end of the transfer
    if( 0 == (scan_count % INTERMEDIATE_MATRIX_ROW_SIZE) && (scan_count != 0 ) )
    {
        //if transpose is complete
        if(scan_count == SRC_MATRIX_Y_SIZE)
        {

            R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED2_GREEN, BSP_IO_LEVEL_HIGH );

            //Mark transfer as complete, reset scan count, src, and dest
            g_is_transfer_complete = true;
            scan_count = 0U;
            g_transfer_new_cfg.p_info->p_src = &src_matrix[0][scan_count];
            g_transfer_new_cfg.p_info->p_dest = &dst_matrix[scan_count][0];

            //re-open DMAC
            err = R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
            APP_ERR_TRAP(err);
            err = R_DMAC_Enable(&g_transfer_new_ctrl);
            APP_ERR_TRAP(err);

            R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED2_GREEN, BSP_IO_LEVEL_LOW );
        }
        else //else if a block is complete
        {
            //cycle the intermediate buffer index
            if(0 == intermediate_buffer_index)
            {
                intermediate_buffer_index = 1;
            }
            else
            {
                intermediate_buffer_index = 0;
            }

            R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED3_RED, BSP_IO_LEVEL_HIGH );
            //increment source matrix index (src's column, destination's row)
                //source matrix increments linearly
            g_transfer_new_cfg.p_info->p_src = &src_matrix[0][scan_count];
                //destination matrix targets current buffer, corresponding row in buffer
            g_transfer_new_cfg.p_info->p_dest = &intermediate_matrix[intermediate_buffer_index][scan_count%INTERMEDIATE_MATRIX_ROW_SIZE][0];

            //re-open DMAC
            err = R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
            APP_ERR_TRAP(err);
            err = R_DMAC_Enable(&g_transfer_new_ctrl);
            APP_ERR_TRAP(err);

            err = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);
            APP_ERR_TRAP(err);
            R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED3_RED, BSP_IO_LEVEL_LOW );
        }
    //if the scan is the end of the block
    }
    else
    {
        //increment source matrix index (src's column, destination's row)
        g_transfer_new_cfg.p_info->p_src = &src_matrix[0][scan_count];
        g_transfer_new_cfg.p_info->p_dest = &intermediate_matrix[intermediate_buffer_index][scan_count%INTERMEDIATE_MATRIX_ROW_SIZE][0];

        //re-open DMAC
        err = R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
        APP_ERR_TRAP(err);
        err = R_DMAC_Enable(&g_transfer_new_ctrl);
        APP_ERR_TRAP(err);

        err = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);
        APP_ERR_TRAP(err);
    }

    R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED1_BLUE, BSP_IO_LEVEL_LOW );

}
