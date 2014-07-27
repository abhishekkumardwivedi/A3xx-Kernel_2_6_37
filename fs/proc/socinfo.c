#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern const struct seq_operations socinfo_op;
static int socinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &socinfo_op);
}

static const struct file_operations proc_socinfo_operations = {
	.open		= socinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_socinfo_init(void)
{
	proc_create("socinfo", 0, NULL, &proc_socinfo_operations);
	return 0;
}
module_init(proc_socinfo_init);
