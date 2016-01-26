#include <mb_includes.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read in errors from all error-related DATA_IDs, once each time they are processed.
// Buffer and send out to LabView/PC and robot LCD display as needed.
// This guarantees (mostly) that all error messages coming in from satellites and the main brain
// will be recorded and displayed. The caveats: first, if errors on a particular channel come in at a faster rate
// than the SSP parse/read function is running, later-arriving error data packets from a board will overwrite the earlier ones.
// So, for example, if the read/parse is running every two milliseconds, errors should be sent from the satellites no faster than
// once every two milliseconds.
// Second, of course, they won't be received properly if the CAN bus is not working properly.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set up ring buffers for error frame storage
static const unsigned short lcd_error_buffer_len = 16;
static DATA_FRAME lcd_error_buffer[lcd_error_buffer_len];
static unsigned short lcd_error_write_index = 0;
static unsigned short lcd_error_read_index = 0;

static const unsigned short labview_error_buffer_len = 64;
static DATA_FRAME labview_error_buffer[labview_error_buffer_len];
static unsigned short labview_error_write_index = 0;
static unsigned short labview_error_read_index = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mb_test_ui_board1(void)
{
  //mb_io_set_ul(ID_UI_SET_LED_1,  0x00000080);

  //mb_io_set_ul(ID_UI_SET_LCD_QUAD_2,  ('e' << 24) | ('f' << 16) | ('g' << 8) | ('h') );
  
  //mb_io_set_ul(ID_UI_SET_BUZZER_FREQ,  0xF0);
  //mb_io_set_ul(ID_UI_SET_BUZZER_AMPL,  0x11);
}

void mb_distribute_error_frames(void)
{
  
  // Write new main brain error, if any, to ID_MB_ERROR
  if (!mb_error_get_frame(mb_io_get_pointer(ID_ERROR_MB)))
  {
    mb_io_mark_as_unread_by_all(ID_ERROR_MB);
  }
  
  // Write new value, if any in LabView error buffer, to ID_ERROR_LABVIEW
  if (mb_io_data_was_read(ID_ERROR_LABVIEW, LABVIEW)) // Check whether previous data has been sent to LabView yet
  {
    if (labview_error_pop(mb_io_get_pointer(ID_ERROR_LABVIEW))) // write new error data if buffer not empty 
    {
      mb_io_mark_as_unread_by_all(ID_ERROR_LABVIEW);
    }
  }

  // Write new value, if any in LCD error buffer, to ID_ERROR_LCD
  if (mb_io_data_was_read(ID_ERROR_LCD, LCD)) // Check whether previous data has been sent to lcd yet
  {
    if (lcd_error_pop(mb_io_get_pointer(ID_ERROR_LCD))) // write new error data if buffer not empty 
    {
      mb_io_mark_as_unread_by_all(ID_ERROR_LCD);
    }
  }

  // Read new value, if any, from ID_ERROR_MB, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MB, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MB))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MB))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MB, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_MCH, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MCH, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MCH))) // write new error data if buffer not full
     
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MCH))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MCH, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_MCFO, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MCFO, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MCFO))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MCFO))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MCFO, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_MCFI, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MCFI, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MCFI))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MCFI))) // write new error data if buffer not full
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MCFI, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_MCSO, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MCSO, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MCSO))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MCSO))) // write new error data if buffer not full
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MCSO, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_MCSI, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_MCSI, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_MCSI))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_MCSI))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_MCSI, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_UI, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_UI, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_UI))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_UI))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_UI, MB_ERR_DIST);
  }

  // Read new value, if any, from ID_ERROR_CSR, and push onto error buffers if possible
  if (!mb_io_data_was_read(ID_ERROR_CSR, MB_ERR_DIST)) // Check for new error data
  {
    if (lcd_error_push(mb_io_get_pointer(ID_ERROR_CSR))) // write new error data if buffer not full 
    {
      //error - LCD display error buffer full
    }

    if (labview_error_push(mb_io_get_pointer(ID_ERROR_CSR))) // write new error data if buffer not full 
    {
      //error - LabView display error buffer full
    }
      mb_io_mark_as_read(ID_ERROR_CSR, MB_ERR_DIST);
  }                    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Push an error frame onto the LabView error buffer
// Check first whether that error is already in the buffer;
// If so, add in the new error frequency, else push normally to ring buffer
// Return zero if operation successful, one if buffer is full
unsigned short labview_error_push(DATA_FRAME * frameptr)
{
  unsigned short temp_index;

  const unsigned long frequency_bits_mask = 0xFFC00000; // Top ten bits are dedicated to error occurrence frequency (count)

  temp_index = labview_error_read_index;  // Initialize temp_index to point to oldest element of buffer

  //Search buffer for error matching incoming frame (both board ID and error ID, but ignore frequency)
  while ((frameptr->payload.ulul.ul1 & ~frequency_bits_mask) != (labview_error_buffer[temp_index].payload.ulul.ul1 & ~frequency_bits_mask))
  {
    if (++temp_index >= labview_error_buffer_len) // Move to next buffer location
    {
      temp_index = 0;
    }
    if (temp_index == labview_error_write_index)  //checked all buffer elements, add new one
    {
      labview_error_buffer[temp_index].payload.ulul.ul1 = frameptr->payload.ulul.ul1;   // Copy error data
      labview_error_buffer[temp_index].payload.ulul.ul2 = frameptr->payload.ulul.ul2;   // Copy timestamp data
      if (++temp_index == labview_error_buffer_len) // Move to next buffer location
      {
        temp_index = 0;
      }
      if (temp_index == labview_error_read_index)
      {
        //error - buffer full; don't change write index; next error may overwrite current error
        return 1; // buffer full, operation unsuccessful
      }
      else
      {
        labview_error_write_index = temp_index; // Advance write index
        return 0; // New error successfully pushed onto buffer
      }
    }
  }

  // Found matching error in buffer
  labview_error_buffer[temp_index].payload.ulul.ul1 += frameptr->payload.ulul.ul1 & frequency_bits_mask; // Update occurrence frequency data
  labview_error_buffer[temp_index].payload.ulul.ul2 = frameptr->payload.ulul.ul2;   // Update timestamp data

  return 0; // Frequency successfully updated for incoming error
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Push an error frame onto the lcd error buffer
// Check first whether that error is already in the buffer;
// If so, add in the new error frequency, else push normally to ring buffer
// Return zero if operation successful, one if buffer is full
unsigned short lcd_error_push(DATA_FRAME * frameptr)
{
  unsigned short temp_index;

  const unsigned long frequency_bits_mask = 0xFFC00000; // Top ten bits are dedicated to error occurrence frequency (count)

  temp_index = lcd_error_read_index;  // Initialize temp_index to point to oldest element of buffer

  //Search buffer for error matching incoming frame (both board ID and error ID, but ignore frequency)
  while ((frameptr->payload.ulul.ul1 & ~frequency_bits_mask) != (lcd_error_buffer[temp_index].payload.ulul.ul1 & ~frequency_bits_mask))
  {
    if (++temp_index >= lcd_error_buffer_len) // Move to next buffer location
    {
      temp_index = 0;
    }
    if (temp_index == lcd_error_write_index)  //checked all buffer elements, add new one
    {
      lcd_error_buffer[temp_index].payload.ulul.ul1 = frameptr->payload.ulul.ul1;   // Copy error data
      lcd_error_buffer[temp_index].payload.ulul.ul2 = frameptr->payload.ulul.ul2;   // Copy timestamp data
      if (++temp_index == lcd_error_buffer_len) // Move to next buffer location
      {
        temp_index = 0;
      }
      if (temp_index == lcd_error_read_index)
      {
        //error - buffer full; don't change write index; next error may overwrite current error
        return 1; // buffer full, operation unsuccessful
      }
      else
      {
        lcd_error_write_index = temp_index; // Advance write index
        return 0; // New error successfully pushed onto buffer
      }
    }
  }

  // Found matching error in buffer
  lcd_error_buffer[temp_index].payload.ulul.ul1 += frameptr->payload.ulul.ul1 & frequency_bits_mask; // Update occurrence frequency data
  lcd_error_buffer[temp_index].payload.ulul.ul2 = frameptr->payload.ulul.ul2;   // Update timestamp data

  return 0; // Frequency successfully updated for incoming error
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short labview_error_pop(DATA_FRAME * frameptr)
{
  if (labview_error_read_index != labview_error_write_index)
  {
    frameptr->payload.ulul.ul1 = labview_error_buffer[labview_error_read_index].payload.ulul.ul1;   // Copy error data
    frameptr->payload.ulul.ul2 = labview_error_buffer[labview_error_read_index].payload.ulul.ul2;   // Copy timestamp data
    
    // Advance to next read buffer location (should be ISR-safe with this construction)
    if (labview_error_read_index >= (labview_error_buffer_len - 1))
    {
      labview_error_read_index = 0;
    }
    else
    {
      ++labview_error_read_index;
    }
    return 0; // Error successfully popped from ring buffer
  }
  else
  {
    return 1; // Error buffer is empty
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short lcd_error_pop(DATA_FRAME * frameptr)
{
  if (lcd_error_read_index != lcd_error_write_index)
  {
    frameptr->payload.ulul.ul1 = lcd_error_buffer[lcd_error_read_index].payload.ulul.ul1;   // Copy error data
    frameptr->payload.ulul.ul2 = lcd_error_buffer[lcd_error_read_index].payload.ulul.ul2;   // Copy timestamp data
    
    // Advance to next read buffer location (should be ISR-safe with this construction)
    if (lcd_error_read_index >= (lcd_error_buffer_len - 1))
    {
      lcd_error_read_index = 0;
    }
    else
    {
      ++lcd_error_read_index;
    }
    return 0; // Error successfully popped from ring buffer
  }
  else
  {
    return 1; // Error buffer is empty
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

