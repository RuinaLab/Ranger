#include <includes.h>

//As of yet, nothing in this file has actually been tested.

void filt_boxcar_s16_init(FILT_I_S16 * iface, FILT_BOXCAR_DATA_S16 * data, signed short * buf, int buf_len){
	int i;
	
	iface->data = (FILT_DATA_S16*)data;
	iface->push = filt_boxcar_s16_push;
	iface->pop  = filt_boxcar_s16_pop;
	iface->most_recent = filt_boxcar_s16_most_recent;
	
	data->sum = 0;
	data->idx = 0;
	data->buf = buf;
	data->len = buf_len;
	
	for (i = 0;i < buf_len;i++) {
		buf[i] = 0;
	}
}

void filt_boxcar_s16_push(FILT_DATA_S16 * data_generic, signed short value) {
	FILT_BOXCAR_DATA_S16 * datab = (FILT_BOXCAR_DATA_S16*)data_generic;
	datab->idx++;
	datab->sum -= datab->buf[datab->idx];
	datab->buf[datab->idx] = value;
	datab->sum += value;
}

signed short filt_boxcar_s16_pop(FILT_DATA_S16 * data_generic) {
	FILT_BOXCAR_DATA_S16 * datab = (FILT_BOXCAR_DATA_S16*)data_generic;
	short result;
	if(datab->sum == 0 ){ //avoid divide by zero
		result = 0;
	} else {
		result = datab->sum / datab->len;
	}
	return result;
}

signed short filt_boxcar_s16_most_recent(FILT_DATA_S16 * data_generic){
	FILT_BOXCAR_DATA_S16 * datab = (FILT_BOXCAR_DATA_S16*)data_generic;
	return datab->buf[datab->idx];
}
