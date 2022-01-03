#pragma once

class CO2_sensor
{

private:
  SoftwareSerial *m_serial;
  HardwareSerial *m_monitor;
  bool m_verbose{false};
  int readCO2UART();

  byte getCheckSum(char *packet);

public:
  void begin(SoftwareSerial &i_serial, HardwareSerial &i_monitor, bool i_verbose );
    int read();
};