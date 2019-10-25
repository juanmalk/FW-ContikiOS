CONTIKI = ../..

APPS = powertrace

all: radio-test example-collect example-neighbors cp example-broadcast  \
     example-m2dagnos_power_trace_1_g example-khs_power_trace_1_g  \
		 example-trigger-broadcast example-runicast wurx_firm

CONTIKI_WITH_RIME = 1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
