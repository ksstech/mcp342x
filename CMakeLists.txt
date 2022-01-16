idf_component_register(
	SRCS "mcp342x.c"
	INCLUDE_DIRS "."
	PRIV_INCLUDE_DIRS
	REQUIRES endpoints hal_esp32
	PRIV_REQUIRES commands printfx rules syslog systiming
)
