#ifndef __DATA_NEXUS_H__
#define __DATA_NEXUS_H__

void route_frames(void);
void router_data_nexus_init(void);
void router_error_transmit(void);
void router_timestamp_transmit(void);
void csr_send_battery_voltage(void);
void csr_send_battery_current(void);
void csr_send_battery_power(void);
void router_update_can_errors(void);
void router_update_can_loading(void);
void router_can1_load_transmit(void);
void router_can2_load_transmit(void);
void router_can3_load_transmit(void);
void router_can4_load_transmit(void);

#endif /* __DATA_NEXUS_H__ */
