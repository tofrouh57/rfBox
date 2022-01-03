#include<SoftwareSerial.h>
#include<CO2.h>



  int CO2_sensor::readCO2UART()
  {
    byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    char response[9];
    if (m_verbose)
    {
      m_monitor->println("Sending CO2 request...");
    }
    m_serial->write(cmd, 9); // request PPM CO2

    // clear the buffer
    memset(response, 0, 9);
    int i = 0;
    while (m_serial->available() == 0)
    {
      if (m_verbose)
      {
        m_monitor->print("Waiting for response ");
        m_monitor->print(i);
        m_monitor->println(" s");
      }
      delay(1000);
      i++;
    }
    if (m_serial->available() > 0)
    {
      m_serial->readBytes(response, 9);
    }
    // print out the response in hexa
    for (int i = 0; i < 9; i++)
    {
      if (m_verbose)
      {
        m_monitor->print(String(response[i], HEX));
        m_monitor->print("   ");
      }
    }
    if (m_verbose)
    {
      m_monitor->println("");
    }
    // checksum
    byte check = getCheckSum(response);
    if (response[8] != check)
    {
      if (m_verbose)
      {
        m_monitor->println("Checksum not OK!");
        m_monitor->print("Received: ");
        m_monitor->println(response[8]);
        m_monitor->print("Should be: ");
        m_monitor->println(check);
      }
    }
    // ppm
    int ppm_uart = 256 * (int)response[2] + response[3];
    if (m_verbose)
    {
      m_monitor->print("UART CO2 PPM: ");
      m_monitor->println(ppm_uart);
    }
    // temp
    byte temp = response[4] - 40;
    if (m_verbose)
    {
      m_monitor->print("Sensor Temperature: ");
      m_monitor->println(temp);
    }
    // status
    byte status = response[5];
    if (m_verbose)
    {
      m_monitor->print("Status: ");
      m_monitor->println(status);
    }
    if (status == 0x40)
    {
      if (m_verbose)
      {
        m_monitor->println("Status OK");
      }
    }
    return ppm_uart;
  }

  byte CO2_sensor::getCheckSum(char *packet)
  {
    byte i;
    unsigned char checksum = 0;
    for (i = 1; i < 8; i++)
    {
      checksum += packet[i];
    }
    checksum = 0xff - checksum;
    checksum += 1;
    return checksum;
  }

  void CO2_sensor::begin(SoftwareSerial &i_serial, HardwareSerial &i_monitor, bool i_verbose )
  {
    m_serial = &i_serial;
    m_monitor = &i_monitor;
    m_verbose = i_verbose;
  }
  int CO2_sensor::read()
  {
    return readCO2UART();
  }

