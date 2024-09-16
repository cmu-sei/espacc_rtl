/* Copyright (c) 2011-2023 Columbia University, System Level Design Group */
/* SPDX-License-Identifier: Apache-2.0 */

#include <stdio.h>
#ifndef __riscv
#include <stdlib.h>
#endif

#include <esp_accelerator.h>
#include <esp_probe.h>
#include <fixed_point.h>

typedef int32_t token_t;

static unsigned DMA_WORD_PER_BEAT(unsigned _st)
{
        return (sizeof(void *) / _st);
}


#define SLD_ESPACC 0x075
#define DEV_NAME "sld,espacc_rtl"

/* <<--params-->> */
const int32_t reg0 = 0; // offset in host memory sent to acc
const int32_t reg1 = 16; // num of accelerator instructions, multiple of beats

static unsigned in_words_adj;
static unsigned out_words_adj;
static unsigned in_len;
static unsigned out_len;
static unsigned in_size;
static unsigned out_size;
static unsigned out_offset;
static unsigned mem_size;

/* Size of the contiguous chunks for scatter/gather */
#define CHUNK_SHIFT 20
#define CHUNK_SIZE BIT(CHUNK_SHIFT)
#define NCHUNK(_sz) ((_sz % CHUNK_SIZE == 0) ?		\
			(_sz / CHUNK_SIZE) :		\
			(_sz / CHUNK_SIZE) + 1)

/* User defined registers */
/* <<--regs-->> */
#define ESPACC_REG0_REG 0x40
#define ESPACC_REG1_REG 0x44


static int validate_buf(token_t *out, token_t *gold)
{
	int i;
	int j;
	unsigned errors = 0;

	for (i = 0; i < 16; i++)
	  {
	    printf("out[%d]: %d, gold: %d\n", 16+i, out[16+i], 2*(i+1));
	    if (out[16+i] != 2*(i+1))
	      {
		errors++;
	      }
	  }

	return errors;
}


static void init_buf (token_t *mem, token_t * gold)
{
	int i;
	int j;

	// Radix 4,dir:0,twid_first:0,twid,transpose,flush all 1,done 0,vlen 4
	mem[0] = 0b00000000000000001000111000110000;
	// Mem Inst
	// Load input data at a stride of 1 and vector stride of 4
	mem[1] = 0b11111111000000100000000010011111;
	// Spad base address for loading inputs
	mem[2] = 0b00000000000000000000000000000000;
	// Mem Inst
	// Load input data at a stride of 1 and vector stride of 4
	mem[3] = 0b11111111000000100000000010101111;
	// Twid mem base address
	mem[4] = 0b00000000000000000000000000000000;
	// Mem Inst
	// Store output data at a stride of 1 and vector stride of 4
	mem[5] = 0b11111111000000100000000011001111;
	// Spad base address for storing outputs
	mem[6] = 0b00000000000000000000000000000000;
	// Config Inst
	// Radix 4,dir:0,twid_first:0,twid:0,transpose:1,flush:1,done:1,vlen:4
	mem[7] = 0b00000000000000001001110000110000;
	// Mem Inst
	// Load input data at a stride of 1 and vector stride of 4
	mem[8] = 0b11111111000000100000000010011111;
	// Spad base address for loading inputs
	mem[9] = 0b00000000000000000000000000000000;
	// Mem Inst
	// Load twiddles at a stride of 1 and vector stride of 4
	mem[10] = 0b11111111000000100000000010101111;
	// Twid mem base address
	mem[11] = 0b00000000000000000000000000000000;
	// Mem Inst
	// This says to store data at a stride of 1 and vector stride of 4
	mem[12] = 0b11111111000000100000000011001111;
	// Spad base address for storing outputs
	mem[13] = 0b00000000000000000000000000000000;

	mem[16] = 1;
	mem[17] = 2;
	mem[18] = 3;
	mem[19] = 4;
	mem[20] = 5;
	mem[21] = 6;
	mem[22] = 7;
	mem[23] = 8;
	mem[24] = 9;
	mem[25] = 10;
	mem[26] = 11;
	mem[27] = 12;
	mem[28] = 13;
	mem[29] = 14;
	mem[30] = 15;
	mem[31] = 16;
}


int main(int argc, char * argv[])
{
	int i;
	int n;
	int ndev;
	struct esp_device *espdevs;
	struct esp_device *dev;
	unsigned done;
	unsigned **ptable;
	token_t *mem;
	token_t *gold;
	unsigned errors = 0;
	unsigned coherence;

	printf("DMA_WORD_PER_BEAT(sizeof(token_t)): %d\n",
	       DMA_WORD_PER_BEAT(sizeof(token_t)));

	if (DMA_WORD_PER_BEAT(sizeof(token_t)) == 0) {
		in_words_adj = 8;
		out_words_adj = 8;
	} else {
		in_words_adj = round_up(8, DMA_WORD_PER_BEAT(sizeof(token_t)));
		out_words_adj = round_up(8, DMA_WORD_PER_BEAT(sizeof(token_t)));
	}
	in_len = in_words_adj * (1);
	out_len = out_words_adj * (1);
	in_size = in_len * sizeof(token_t);
	out_size = out_len * sizeof(token_t);
	out_offset  = in_len;

	// 16 32-bit "instructions" + 16 32-bit datum
	mem_size = ((16 + 16) * sizeof(token_t));

        printf("in_len: %u\n", in_len);
        printf("out_len: %u\n", out_len);
        printf("sizeof(token_t): %u\n", sizeof(token_t));
        printf("in_size: %u\n", in_size);
        printf("out_size: %u\n", out_size);
        printf("out_offset: %u\n", out_offset);
        printf("mem_size: %u\n", mem_size);

	// Search for the device
	printf("Scanning device tree... \n");

	ndev = probe(&espdevs, VENDOR_SLD, SLD_ESPACC, DEV_NAME);
	if (ndev == 0) {
		printf("espacc not found\n");
		return 0;
	}

	for (n = 0; n < ndev; n++) {

		printf("**************** %s.%d ****************\n", DEV_NAME, n);

		dev = &espdevs[n];

		// Check DMA capabilities
		if (ioread32(dev, PT_NCHUNK_MAX_REG) == 0) {
			printf("  -> scatter-gather DMA is disabled. Abort.\n");
			return 0;
		}

		if (ioread32(dev, PT_NCHUNK_MAX_REG) < NCHUNK(mem_size)) {
			printf("  -> Not enough TLB entries available. Abort.\n");
			return 0;
		}

		// Allocate memory
		gold = aligned_malloc(out_size);
		mem = aligned_malloc(mem_size);
		printf("  memory buffer base-address = %p\n", mem);

		// Alocate and populate page table
		ptable = aligned_malloc(NCHUNK(mem_size) * sizeof(unsigned *));
		for (i = 0; i < NCHUNK(mem_size); i++)
			ptable[i] = (unsigned *) &mem[i * (CHUNK_SIZE / sizeof(token_t))];

		printf("  ptable = %p\n", ptable);
		printf("  nchunk = %lu\n", NCHUNK(mem_size));

#ifndef __riscv
		for (coherence = ACC_COH_NONE; coherence <= ACC_COH_RECALL; coherence++) {
#else
                for (size_t ii=0; ii<10; ii++)
		{
			/* TODO: Restore full test once ESP caches are integrated */
			coherence = ACC_COH_NONE;
#endif
			printf("  --------------------\n");
			printf("  Generate input...\n");
			init_buf(mem, gold);

			// Pass common configuration parameters

			iowrite32(dev, SELECT_REG, ioread32(dev, DEVID_REG));
			iowrite32(dev, COHERENCE_REG, coherence);

#ifndef __sparc
			iowrite32(dev, PT_ADDRESS_REG, (unsigned long long) ptable);
#else
			iowrite32(dev, PT_ADDRESS_REG, (unsigned) ptable);
#endif
			iowrite32(dev, PT_NCHUNK_REG, NCHUNK(mem_size));
			iowrite32(dev, PT_SHIFT_REG, CHUNK_SHIFT);

			// Use the following if input and output data are not allocated at the default offsets
			iowrite32(dev, SRC_OFFSET_REG, 0x0);
			iowrite32(dev, DST_OFFSET_REG, 0x0);

			// Pass accelerator-specific configuration parameters
			/* <<--regs-config-->> */
                        iowrite32(dev, ESPACC_REG0_REG, reg0);
                        iowrite32(dev, ESPACC_REG1_REG, reg1);

			// Flush (customize coherence model here)
			esp_flush(coherence);

			// Start accelerators
			printf("  Start...\n");
                        printf("CMD_REG: %u, CMD_MASK_START: %u\n",
                               CMD_REG, CMD_MASK_START);
			iowrite32(dev, CMD_REG, CMD_MASK_START);

			// Wait for completion
			printf("  Wait...\n");
			done = 0;
			while (!done) {
				done = ioread32(dev, STATUS_REG);
				done &= STATUS_MASK_DONE;
			}
			iowrite32(dev, CMD_REG, 0x0);
			printf("  Done\n");
			printf("  validating...\n");

			/* Validation */
			errors = validate_buf(&mem[0], gold);
			if (errors)
				printf("  ... FAIL\n");
			else
				printf("  ... PASS\n");
		}
		aligned_free(ptable);
		aligned_free(mem);
		aligned_free(gold);
	}

	return 0;
}
