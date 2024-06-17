IMPL_DIR = ../src/impl
RTL_DIR = ../src/espacc_rtl_basic_dma
RTL_FILES = \
	$(IMPL_DIR)/espacc_rtl_basic_dma.v \
	$(RTL_DIR)64/espacc_rtl_basic_dma64.v \
	$(RTL_DIR)32/espacc_rtl_basic_dma32.v

DEFINES = -DTESTBENCH

all : test

.PHONY : clean, test

clean :
	-@$(RM) tb

# for this tb, really only need to compile espacc_rtl_basic_dma.v
test : tb.v $(RTL_FILES)
	iverilog -o tb $(DEFINES) -I $(IMPL_DIR) -I $(RTL_DIR) \
	tb.v $(RTL_FILES)
	vvp tb

espacc-gui : test
	gtkwave espacc.vcd testbench.gtkw &
