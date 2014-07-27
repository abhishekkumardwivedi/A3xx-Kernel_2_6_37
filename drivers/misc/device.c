#include <linux/module.h>

static char *mnfrparams;
module_param(mnfrparams, charp, S_IRUGO);
MODULE_PARM_DESC(mnfrparams, "Manufacture block");

static char *xldrver;
module_param(xldrver, charp, S_IRUGO);
MODULE_PARM_DESC(xldrver, "X-Loader version");

static char *ubootver;
module_param(ubootver, charp, S_IRUGO);
MODULE_PARM_DESC(ubootver, "U-Boot version");

static unsigned int hw_stat;
module_param(hw_stat, uint, S_IRUGO);
MODULE_PARM_DESC(hw_stat, "Boot status bits");

static char *product;
module_param(product, charp, S_IRUGO);
MODULE_PARM_DESC(product, "Product name from PCB ID");
