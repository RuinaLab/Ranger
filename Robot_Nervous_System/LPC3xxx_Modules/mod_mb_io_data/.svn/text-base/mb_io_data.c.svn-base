#include <mb_includes.h>
 
//IO_DATA_POINT mb_io_data[MB_IO_DATA_ARRAY_SIZE];

//Define array holding all data and parameters at the main brain level; size is equal
//to the maximum ID in use, given by enum value ID_LAST + 1.
DATA_FRAME mb_io_data[ID_LAST + 1];


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Gets a pointer to the data frame at address data_id.
DATA_FRAME * mb_io_get_pointer(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  {
    return &mb_io_data[data_id];
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return NULL;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Flag DATA_FRAME data_id as already read by the subscribing process identified by subscriber_id
void mb_io_mark_as_read(unsigned short data_id, unsigned short subscriber_id)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].data_read |= (1 << subscriber_id);
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Flag DATA_FRAME data_id as not yet read by the subscribing process identified by subscriber_id
void mb_io_mark_as_unread(unsigned short data_id, unsigned short subscriber_id)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].data_read &= ~(1 << subscriber_id);
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Flag DATA_FRAME data_id as not yet read by any process
void mb_io_mark_as_unread_by_all(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].data_read = 0;
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Return 1 if DATA_FRAME data_id has already been read by the subscribing process identified by subscriber_id;
// else return 0, data is new.
unsigned short mb_io_data_was_read(unsigned short data_id, unsigned short subscriber_id)
{
  if (data_id <= ID_LAST)
    {
    if ((mb_io_data[data_id].data_read & (1 << subscriber_id)) == 0)
    {
      return 0; // Data has not yet been read by subscriber
   }
    else
    {
      return 1; // Data has been read by subscriber
    }
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
float mb_io_get_float(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  { 
    return mb_io_data[data_id].payload.ful.f;
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return 0.0f;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long mb_io_get_ul(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  {
    return mb_io_data[data_id].payload.ulul.ul1;
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
long int mb_io_get_sl(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  {
    return mb_io_data[data_id].payload.slul.sl;
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long mb_io_get_time(unsigned short data_id)
{
  if (data_id <= ID_LAST)
  {
    return mb_io_data[data_id].payload.ful.ul;
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
    return 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
void mb_io_set_float(unsigned short data_id, float value)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].payload.ful.f = value;
    mb_io_data[data_id].payload.ful.ul = mb_io_get_time(ID_TIMESTAMP);
    mb_io_data[data_id].data_read = 0;
  } 
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
void mb_io_set_ul(unsigned short data_id, unsigned long value)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].payload.ulul.ul1 = value;
    mb_io_data[data_id].payload.ulul.ul2 = mb_io_get_time(ID_TIMESTAMP);
    mb_io_data[data_id].data_read = 0; 
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  } 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
void mb_io_set_sl(unsigned short data_id, signed long value)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].payload.slul.sl = value;
    mb_io_data[data_id].payload.slul.ul = mb_io_get_time(ID_TIMESTAMP);   
    mb_io_data[data_id].data_read = 0; 
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// This function is only intended for updating the time value of data_id 0, the current elapsed time
// Other time stamps should be entered automatically by functions that set the IO data point values.
void mb_io_set_time(unsigned short data_id, unsigned long time)
{
  if (data_id <= ID_LAST)
  {
    mb_io_data[data_id].payload.ful.ul = time;
    mb_io_data[data_id].data_read = 0; 
  }
  else
  {
    mb_error_occurred(ERROR_MB_IO_ID_OOR);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
void mb_io_init(void)
{
  unsigned short i;
  
  for (i = 0; i <= ID_LAST; ++i)
  {
    mb_io_data[i].payload.ful.f = 0.0;
    mb_io_data[i].payload.ful.ul = 0;
    mb_io_data[i].id = i;
    mb_io_data[i].data_read = 0xFFFFFFFF;  // Mark as read, pending initialization
  }

  //Parameter initialization statements
  #include <io_data.c>
  
  
  mb_io_data[ID_LAST].payload.ful.f = ID_LAST;

  // Set data_read to zero after initialization, to cause initial values to be sent to data display, etc.
  // Actually, this should be done at the time of initialization, only for the parameters actually initialized.
  for (i = 0; i <= ID_LAST; ++i)
  {
    mb_io_data[i].data_read = 0x0;    // Mark as unread
  }
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
