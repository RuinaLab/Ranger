#ifndef __MOD_FILT_H__
#define __MOD_FILT_H__

typedef void* FILT_DATA_S16;

typedef void(*FILT_PUSH_INT16)(FILT_DATA_S16*,signed short);
typedef signed short(*FILT_POP_INT16)(FILT_DATA_S16*);

typedef struct filter_interface_s16 {
        FILT_PUSH_INT16  push;
        FILT_POP_INT16   pop;
		FILT_POP_INT16   most_recent;
        FILT_DATA_S16  * data;
} FILT_I_S16;

/****************** NONE definitions ***********/
typedef struct filter_none_data_s16 {
        signed short value;
} FILT_NONE_DATA_S16;


void         filt_none_s16_init(FILT_I_S16 * iface, FILT_NONE_DATA_S16 * data);
void         filt_none_s16_push(FILT_DATA_S16 * data_generic, signed short value);
signed short filt_none_s16_pop (FILT_DATA_S16 * data_generic);
/**************************************************/

/****************** BOXCAR definitions ***********/
typedef struct filter_boxcar_data_s16 {
        int len;
        int idx;
        signed short * buf;
        int sum;
} FILT_BOXCAR_DATA_S16;

void         filt_boxcar_s16_init(FILT_I_S16 * iface, FILT_BOXCAR_DATA_S16 * data, signed short * buf, int buf_len);
void         filt_boxcar_s16_push(FILT_DATA_S16 * data_generic, signed short value);
signed short filt_boxcar_s16_pop (FILT_DATA_S16 * data_generic);
signed short filt_boxcar_s16_most_recent(FILT_DATA_S16 * data_generic);
/**************************************************/

#endif  //__MOD_FILT_H__


