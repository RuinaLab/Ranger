#ifndef __BOARD_ID_H__
#define __BOARD_ID_H__

//Specific board IDs - Naming: BOARD_<type>_<specific>
typedef enum board_vals{
	BOARD_PC = 0,
	BOARD_MB,
	BOARD_CSR,
  BOARD_MCH,
	BOARD_MCFO, 
	BOARD_MCFI, 
  BOARD_MCSO, 
  BOARD_MCSI, 
	BOARD_UI,
	BOARD_CAN_SNIFFER,
  BOARD_MC_INVERTATRON
} BOARD_ID;

#endif //__BOARD_ID_H__
