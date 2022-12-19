#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "micro_ros_transport_serial.h"
extern "C"
{

  bool platformio_transport_open_serial(struct uxrCustomTransport *transport)
  {
    return true;
  }

  bool platformio_transport_close_serial(struct uxrCustomTransport *transport)
  {
    return true;
  }

  size_t platformio_transport_write_serial(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;

    Stream *stream = (Stream *)transport->args;
    size_t sent = stream->write(buf, len);
    return sent;
  }

  size_t platformio_transport_read_serial(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;

    Stream *stream = (Stream *)transport->args;
    stream->setTimeout(timeout);
    return stream->readBytes((char *)buf, len);
  }
}