#include "hal_data.h"
#include "stdint.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#define SRC_MATRIX_X_SIZE 128
#define SRC_MATRIX_Y_SIZE 300

#define DST_MATRIX_X_SIZE SRC_MATRIX_Y_SIZE
#define DST_MATRIX_Y_SIZE SRC_MATRIX_X_SIZE


//define as UINT16_t
volatile uint16_t src_matrix[SRC_MATRIX_X_SIZE][SRC_MATRIX_Y_SIZE];
volatile uint16_t dst_matrix[DST_MATRIX_X_SIZE][DST_MATRIX_Y_SIZE];

#define MATRIX_ELEMENT_SIZE TRANSFER_SIZE_2_BYTE //adjust to match the size of the src_matrix


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
  .p_dest = (void*) dst_matrix,
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


bool g_is_transfer_complete = false;
uint32_t scan_count = 0U;

void fill_src_matrix(void);

void g_transfer0_cb( dmac_callback_args_t * p_args ) //args and events
{
    fsp_err_t status;

//    if(scan_count < SRC_MATRIX_Y_SIZE)
//    {
        //increment scan count
        scan_count++;

        //Close current DMA session
        R_DMAC_Close(&g_transfer_new_ctrl);



        if(scan_count == SRC_MATRIX_Y_SIZE)
        {
            g_is_transfer_complete = true;
        }
        else
        {
            //increment source matrix index (src's column, destination's row)
            g_transfer_new_cfg.p_info->p_src = &src_matrix[0][scan_count];//(uint16_t *) src_matrix[0][scan_count];
            g_transfer_new_cfg.p_info->p_dest = &dst_matrix[scan_count][0];//(uint16_t *) dst_matrix[scan_count][0];

            //re-open DMAC
            R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
            R_DMAC_Enable(&g_transfer_new_ctrl);

            status = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);
        }
//    }
//    else
//    {
//
//    }


}




/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */

    volatile fsp_err_t status;

    fill_src_matrix();

    status = R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
    R_DMAC_Enable(&g_transfer_new_ctrl);

    status = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);

    while( g_is_transfer_complete == false)
    {
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);;
    }

    while(1)
    {
        R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    }


#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        IOPORT_CFG_OPEN (&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);
    }
}

void fill_src_matrix(void)
{
    for( uint32_t loop_index_x = 0; loop_index_x < (SRC_MATRIX_X_SIZE); loop_index_x++)
    {
        for( uint32_t loop_index_y = 0; loop_index_y < (SRC_MATRIX_Y_SIZE); loop_index_y++)
       {
            src_matrix[loop_index_x][loop_index_y] = loop_index_x + loop_index_y*SRC_MATRIX_X_SIZE;
       }
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif