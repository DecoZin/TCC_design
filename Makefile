TOP?=
ifneq ($(TOP),)
	_TOP = $(TOP)
else
	_TOP = uart_tb
endif

# DEBUG?=0
# ifneq ($(DEBUG),0)
# 	_DEBUG = --debug
# else
# 	_DEBUG =
# endif
run:
	xrun -f ./test/$(_TOP).f -access +rwc -linedebug +define+FSDB -top $(_TOP)

vsim:
	simvision waveform.vcd

help:
	@echo "run: Run a testbench. Usage:"
	@echo "		make run TOP=uart_tb"
	@echo ""
	@echo "vsim: Opens a waveform viewer, using a default file named waveform.vcd. Usage:"
	@echo "		make vsim"
	@echo ""
