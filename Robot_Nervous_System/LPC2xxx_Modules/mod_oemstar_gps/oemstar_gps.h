/*
	oemstar_gps.h
	
  Jason Cortell - March 2011
	Nicolas Williamson - Dec. 2009
*/

#ifndef __H_OEMSTAR_GPS__
#define __H_OEMSTAR_GPS__

/**
  Possible commands to send to the GPS.
*/
typedef __packed struct gps_messages
{
    //Header (common format)
  __packed struct header{
    char sync1;
    char sync2;
    char sync3;
    unsigned char header_length;
    unsigned short message_id;
    char message_type;
    unsigned char port_address;
    unsigned short message_length;
    unsigned short sequence;
    unsigned char idle_time;
    unsigned char time_status;
    unsigned short week;
    unsigned long milliseconds;
    unsigned long receiver_status;
    unsigned short reserved;
    unsigned short receiver_software_version;
  } HEADER;
    //Commands
  __packed struct antennapower{
    unsigned long flag;
  } ANTENNAPOWER;

  __packed struct clockadjust{
    unsigned long enable;
  } CLOCKADJUST;

  __packed struct com{
    unsigned long port;
    unsigned long baud;
    unsigned long parity;
    unsigned long databits;
    unsigned long stopbits;
    unsigned long handshake;
    unsigned long echo;
    unsigned long break_enable;
  } COM;

  __packed struct ecutoff{
    float angle;
  } ECUTOFF;

  __packed struct fix{
    unsigned long type;
    double param1;
    double param2;
    double param3;
  } FIX;

  __packed struct freset{
    unsigned long target;
  } FRESET;

  __packed struct log{
    unsigned long port;
    unsigned short message_id;
    char message_type;
    char reserved;
    unsigned long trigger;
    double period;
    double offset;
    unsigned long hold;
  } LOG;

  __packed struct pdpfilter{
    unsigned long control;
  } PDPFILTER;

  __packed struct pdpmode{
    unsigned long mode;
    unsigned long dynamics;
  } PDPMODE;

  __packed struct psrvelocitytype{
    unsigned long source;
  } PSRVELOCITYTYPE;

  __packed struct reset{
    unsigned long delay;
  } RESET;

  __packed struct sbascontrol{
    unsigned long enable;
    unsigned long system;
    unsigned long prn;
    unsigned long testmode;
  } SBASCONTROL;

  __packed struct setionotype{
    unsigned long model;
  } SETIONOTYPE;

    //Logs
  __packed struct pdpvel{
    unsigned long sol_status;
    unsigned long vel_type;
    float latency;
    float age;
    double hor_speed;
    double trk_ground;
    double height;
    float reserved;
  } PDPVEL;

  __packed struct pdpxyz{
    unsigned long p_sol_status;
    unsigned long pos_type;
    double p_x;
    double p_y;
    double p_z;
    float p_x_sig;
    float p_y_sig;
    float p_z_sig;
    unsigned long v_sol_status;
    unsigned long vel_type;
    double v_x;
    double v_y;
    double v_z;
    float v_x_sig;
    float v_y_sig;
    float v_z_sig;
    char stn_id[4];
    float v_latency;
    float diff_age;
    float sol_age;
    unsigned char num_sats;
    unsigned char num_sats_sol;
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    unsigned char reserved5;
    unsigned char reserved6;
  } PDPXYZ;
   
} GPS_MESSAGES;

// ******** Public Functions ******** //
void gps_update(void);
void gps_init(MSIMU_COMMAND command);
float gps_get_data_float(MSIMU_DATA index);
int gps_get_data_int(MSIMU_DATA index);

// ******** Private Functions ******** //
void gps_send_all(unsigned char *bytes, unsigned int length);
void gps_send_byte(unsigned char byte);
void gps_send_polled(MSIMU_COMMAND command);
void gps_set_continuous(MSIMU_COMMAND command);
void gps_isr(void) __irq;
int gps_get_length(MSIMU_COMMAND command);
void gps_parse_buffer(void);

#endif /* __H_OEMSTAR_GPS__ */

