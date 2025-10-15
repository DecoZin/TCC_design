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
	@echo "verilator: Build a verilator project. Usage:"
	@echo "		make verilator TOP=uart_tb"
	@echo ""
	@echo "verilator_run: Run a verilator project. Usage:"
	@echo "		make verilator_run TOP=uart_tb" 

verilator:
	~/Utils/oss-cad-suite/bin/verilator --binary -f ./test/$(_TOP).f -f ./config.f --top $(_TOP) --Mdir ./$(_TOP)_verilator

verilator_run:
	./$(_TOP)_verilator/V$(_TOP) | tee log/$(_TOP).log
