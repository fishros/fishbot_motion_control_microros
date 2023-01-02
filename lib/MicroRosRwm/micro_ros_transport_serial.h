

extern "C"
{
	bool platformio_transport_open_serial(struct uxrCustomTransport *transport);
	bool platformio_transport_close_serial(struct uxrCustomTransport *transport);
	size_t platformio_transport_write_serial(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
	size_t platformio_transport_read_serial(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
}

static bool set_microros_serial_transports(Stream &stream)
{
	rmw_uros_set_custom_transport(
		true,
		&stream,
		platformio_transport_open_serial,
		platformio_transport_close_serial,
		platformio_transport_write_serial,
		platformio_transport_read_serial);
	return true;
}