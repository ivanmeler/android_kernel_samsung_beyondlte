#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/slab.h>

#include <linux/vmalloc.h>
#include <linux/sched/signal.h>

/*
 * For storing callsite and calltarget
 * Give enough space about around 20k 
 * callsites or calltargets
 */


void panic_jopp(unsigned long calltarget, unsigned long callsite){
	panic("[CFP_JOPP] function has no jopp magic (calltarget: %pF, callsite: %pF).", calltarget, callsite);
}

#ifdef CONFIG_RKP_CFP_TEST

#define MAXPAIR 20*1024
unsigned long jopp_array[MAXPAIR*2];

void cfp_record_jopp(unsigned long calltarget, unsigned long callsite)
{
	unsigned int i = 0;
	/*
	 * Need to confirm
	 * 1) one callsite, call many calltargets
	 * 2) one calltarget, reached by many callsites
	 */

	for (i=0; i < MAXPAIR; i++) {
		if ((jopp_array[i*2] == callsite) && (jopp_array[i*2+1] == calltarget)) {
			return;
		}
	}

	for (i=0; i < MAXPAIR; i++) {
		if (jopp_array[i*2] == 0) {
			jopp_array[i*2] = callsite;
			jopp_array[i*2+1] = calltarget;
			break;
		}
	}
}

ssize_t	cfp_read(struct file *filep, char __user *buf, size_t size, loff_t *offset)
{
	unsigned int i = 0;
	for (i=0; i < MAXPAIR; i++) {
		if (jopp_array[i*2] == 0)
			break;
		printk("callsite target %p %p\n", (void *)(jopp_array[i*2]), (void *)(jopp_array[i*2+1]));
	}
	return 0;
}

static const struct file_operations cfp_proc_fops = {
	.read		= cfp_read,
};

static int __init cfp_test_read_init(void)
{
	if (proc_create("cfp_test", 0644, NULL, &cfp_proc_fops) == NULL) {
		printk(KERN_ERR "%s: Error creating proc entry\n", __func__);
		goto error_return;
	}
	return 0;

error_return:
	return -1;
}

static void __exit cfp_test_read_exit(void)
{
	remove_proc_entry("cfp_test", NULL);
	printk(KERN_INFO"Deregistering /proc/cfp_test Interface\n");
}

module_init(cfp_test_read_init);
module_exit(cfp_test_read_exit);

#endif // CONFIG_RKP_CFP_TEST
