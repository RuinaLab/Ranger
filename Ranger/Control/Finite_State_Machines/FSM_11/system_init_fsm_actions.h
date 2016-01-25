///////////////////////////////////////////////////////////////////////////////////////////////////
//Action functions
int ACTION_SI_system_start(void)
{
  set_io_float(ID_MB_STATUS, 1);
  return 1;
}

int ACTION_SI_system_run(void)
{
  set_io_float(ID_MB_STATUS, 2);
  return 1;
}

int ACTION_SI_stop(void)
{
  return 1;
}


