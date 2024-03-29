#include "hal_data.h"
#include "stdint.h"
#include "dmac_settings.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER


extern bool g_is_transfer_complete;

extern dmac_instance_ctrl_t g_transfer_new_ctrl;
extern const transfer_cfg_t g_transfer_new_cfg;


//define as UINT16_t
extern uint16_t src_matrix[SRC_MATRIX_X_SIZE][SRC_MATRIX_Y_SIZE];
extern uint16_t dst_matrix[DST_MATRIX_X_SIZE][DST_MATRIX_Y_SIZE];

void fill_src_matrix(void);

#define APP_ERR_TRAP(err)        ({\
                                    if((err)) {\
                                         __asm("BKPT #0\n");} /* trap upon the error  */\
                                    })




/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    volatile fsp_err_t status;

    //fill the source matrix
    fill_src_matrix();
    //clear the destination matrix
    memset(dst_matrix,0,DST_MATRIX_X_SIZE*DST_MATRIX_Y_SIZE);

    // Open the DMAC with the new settings in dmac_settings.c
    status = R_DMAC_Open(&g_transfer_new_ctrl, &g_transfer_new_cfg);
    APP_ERR_TRAP(status);
    status = R_DMAC_Enable(&g_transfer_new_ctrl);
    APP_ERR_TRAP(status);

    status = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);
    APP_ERR_TRAP(status);

    // loop forever
    while(1)
    {
        status = R_DMAC_SoftwareStart(&g_transfer_new_ctrl, TRANSFER_START_MODE_REPEAT);
//        R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED3_RED, BSP_IO_LEVEL_HIGH );

        // stay here until the DMAC is over
        while( g_is_transfer_complete == false)
        {
            __asm("NOP\n");
        }
        g_is_transfer_complete = false;
//        R_IOPORT_PinWrite(&g_ioport_ctrl, USER_LED3_RED, BSP_IO_LEVEL_LOW );

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

        /* Initialize SDRAM */
        bsp_sdram_init();
    }
}

//TODO: account for 16-bit overrun if SRC_MATRIX_X_SIZE * ...Y_SIZE > 65535
/*******************************************************************************************************************//**
 * Populate src_matrix with data to transpose
 *
 **********************************************************************************************************************/
void fill_src_matrix(void)
{
    for( uint16_t loop_index_x = 0; loop_index_x < (SRC_MATRIX_X_SIZE); loop_index_x++)
    {
        for( uint16_t loop_index_y = 0; loop_index_y < (SRC_MATRIX_Y_SIZE); loop_index_y++)
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
