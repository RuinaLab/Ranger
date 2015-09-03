#include <includes.h>

void filt_none_s16_init(FILT_I_S16 * iface, FILT_NONE_DATA_S16 * data){
	iface->data = (FILT_DATA_S16*)data;
	iface->push = filt_none_s16_push;
	iface->pop  = filt_none_s16_pop;
	iface->most_recent = filt_none_s16_pop;
	
	data->value = 0;
}

void filt_none_s16_push(FILT_DATA_S16 * data_generic, signed short value) {
	FILT_NONE_DATA_S16 * data = (FILT_NONE_DATA_S16*)data_generic;
	data->value = value;
}

signed short filt_none_s16_pop(FILT_DATA_S16 * data_generic) {
	FILT_NONE_DATA_S16 * data = (FILT_NONE_DATA_S16*)data_generic;
	return data->value;
}
