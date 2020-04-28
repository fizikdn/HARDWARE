/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      scif_main
 *
 * @brief   initialization and task creation of sensorcontoller
 *
 * @param   none
 *
 * @return  none
 */

extern int scif_main(void);
extern void RS_Flow_SCIF_performPeriodicTask(void);


#define LCD_ON             	0


#if LCD_ON == 1
#define LCD_DISPLAY_ON
#endif


