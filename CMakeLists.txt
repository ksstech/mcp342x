# MCP342X

set( srcs "mcp342x.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires endpoints hal_esp32 )
set( priv_requires commands driver main printfx rules syslog systiming )
				
idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
