 #ifndef __MB_DATA_NEXUS_H__
 #define __MB_DATA_NEXUS_H__

void mb_distribute_error_frames(void);
void mb_test_ui_board1(void);
void mb_test_ui_board2(void);

unsigned short labview_error_push(DATA_FRAME * frameptr);
unsigned short lcd_error_push(DATA_FRAME * frameptr);
unsigned short labview_error_pop(DATA_FRAME * frameptr);
unsigned short lcd_error_pop(DATA_FRAME * frameptr);
























 #endif //__MB_DATA_NEXUS_H__
