// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0
#include <linux/of_device.h>
#include <linux/mm.h>

#include <asm/io.h>

#include <esp_accelerator.h>
#include <esp.h>

#include "espacc_rtl.h"

#define DRV_NAME	"espacc_rtl"

/* <<--regs-->> */
#define ESPACC_REG0_REG 0x40
#define ESPACC_REG1_REG 0x44

struct espacc_rtl_device {
	struct esp_device esp;
};

static struct esp_driver espacc_driver;

static struct of_device_id espacc_device_ids[] = {
	{
		.name = "SLD_ESPACC_RTL",
	},
	{
		.name = "eb_075",
	},
	{
		.compatible = "sld,espacc_rtl",
	},
	{ },
};

static int espacc_devs;

static inline struct espacc_rtl_device *to_espacc(struct esp_device *esp)
{
	return container_of(esp, struct espacc_rtl_device, esp);
}

static void espacc_prep_xfer(struct esp_device *esp, void *arg)
{
	struct espacc_rtl_access *a = arg;

	/* <<--regs-config-->> */
	iowrite32be(a->reg0, esp->iomem + ESPACC_REG0_REG);
	iowrite32be(a->reg1, esp->iomem + ESPACC_REG1_REG);
	iowrite32be(a->src_offset, esp->iomem + SRC_OFFSET_REG);
	iowrite32be(a->dst_offset, esp->iomem + DST_OFFSET_REG);

}

static bool espacc_xfer_input_ok(struct esp_device *esp, void *arg)
{
	/* struct espacc_rtl_device *espacc = to_espacc(esp); */
	/* struct espacc_rtl_access *a = arg; */

	return true;
}

static int espacc_probe(struct platform_device *pdev)
{
	struct espacc_rtl_device *espacc;
	struct esp_device *esp;
	int rc;

	espacc = kzalloc(sizeof(*espacc), GFP_KERNEL);
	if (espacc == NULL)
		return -ENOMEM;
	esp = &espacc->esp;
	esp->module = THIS_MODULE;
	esp->number = espacc_devs;
	esp->driver = &espacc_driver;
	rc = esp_device_register(esp, pdev);
	if (rc)
		goto err;

	espacc_devs++;
	return 0;
 err:
	kfree(espacc);
	return rc;
}

static int __exit espacc_remove(struct platform_device *pdev)
{
	struct esp_device *esp = platform_get_drvdata(pdev);
	struct espacc_rtl_device *espacc = to_espacc(esp);

	esp_device_unregister(esp);
	kfree(espacc);
	return 0;
}

static struct esp_driver espacc_driver = {
	.plat = {
		.probe		= espacc_probe,
		.remove		= espacc_remove,
		.driver		= {
			.name = DRV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = espacc_device_ids,
		},
	},
	.xfer_input_ok	= espacc_xfer_input_ok,
	.prep_xfer	= espacc_prep_xfer,
	.ioctl_cm	= ESPACC_RTL_IOC_ACCESS,
	.arg_size	= sizeof(struct espacc_rtl_access),
};

static int __init espacc_init(void)
{
	return esp_driver_register(&espacc_driver);
}

static void __exit espacc_exit(void)
{
	esp_driver_unregister(&espacc_driver);
}

module_init(espacc_init)
module_exit(espacc_exit)

MODULE_DEVICE_TABLE(of, espacc_device_ids);

MODULE_AUTHOR("Emilio G. Cota <cota@braap.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("espacc_rtl driver");
