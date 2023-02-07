/* sound/soc/samsung/abox/abox_proc.c
 *
 * ALSA SoC Audio Layer - Samsung Abox Proc FS driver
 *
 * Copyright (c) 2020 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/slab.h>

#include "abox_proc.h"

#define ROOT_DIR_NAME "abox"

static struct proc_dir_entry *root;

/* for backward compatibility */
static struct dentry *debugfs_link;

void *abox_proc_data(const struct file *file)
{
	return PDE_DATA(file_inode(file));
}

struct proc_dir_entry *abox_proc_mkdir(const char *name,
		struct proc_dir_entry *parent)
{
	if (!parent)
		parent = root;

	return proc_mkdir(name, parent);
}

void abox_proc_remove_file(struct proc_dir_entry *pde)
{
	proc_remove(pde);
}

struct proc_dir_entry *abox_proc_create_file(const char *name, umode_t mode,
		struct proc_dir_entry *parent,
		const struct file_operations *fops, void *data, size_t size)
{
	struct proc_dir_entry *pde;

	if (!parent)
		parent = root;
	pde = proc_create_data(name, mode, parent, fops, data);
	if (!IS_ERR(pde))
		proc_set_size(pde, size);

	return pde;
}

static ssize_t abox_proc_bin_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	struct abox_proc_bin *bin = abox_proc_data(file);

	return simple_read_from_buffer(buf, count, pos, bin->data, bin->size);
}

static const struct file_operations abox_proc_bin_fops = {
	.read	= abox_proc_bin_read,
	.open	= simple_open,
	.llseek	= default_llseek,
};

struct proc_dir_entry *abox_proc_create_bin(const char *name, umode_t mode,
		struct proc_dir_entry *parent, struct abox_proc_bin *bin)
{
	return abox_proc_create_file(name, mode, parent, &abox_proc_bin_fops,
			bin, bin->size);
}

static ssize_t abox_proc_u64_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	unsigned long long *val = abox_proc_data(file);
	char buf_val[24]; /* enough to store a u64 and "\n\0" */
	size_t size;

	size = scnprintf(buf_val, sizeof(buf_val), "%llu\n", *val);

	return simple_read_from_buffer(buf, count, pos, buf_val, size);
}

static ssize_t abox_proc_u64_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	unsigned long long *val = abox_proc_data(file);
	int ret;

	ret = kstrtoull_from_user(buf, count, 0, val);
	if (ret < 0)
		return ret;

	return count;
}

static const struct file_operations abox_proc_u64_fops = {
	.read	= abox_proc_u64_read,
	.write	= abox_proc_u64_write,
	.open	= simple_open,
	.llseek	= no_llseek,
};

struct proc_dir_entry *abox_proc_create_u64(const char *name, umode_t mode,
		struct proc_dir_entry *parent, u64 *data)
{
	return abox_proc_create_file(name, mode, parent, &abox_proc_u64_fops,
			data, 0);
}

static ssize_t abox_proc_bool_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	bool *val = abox_proc_data(file);
	char buf_val[4]; /* enough to store a bool and "\n\0" */

	if (*val)
		buf_val[0] = 'Y';
	else
		buf_val[0] = 'N';
	buf_val[1] = '\n';
	buf_val[2] = 0x00;
	return simple_read_from_buffer(buf, count, pos, buf_val, 2);
}

static ssize_t abox_proc_bool_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	bool *val = abox_proc_data(file);
	int ret;

	ret = kstrtobool_from_user(buf, count, val);
	if (ret < 0)
		return ret;

	return count;
}

static const struct file_operations abox_proc_bool_fops = {
	.read	= abox_proc_bool_read,
	.write	= abox_proc_bool_write,
	.open	= simple_open,
	.llseek	= no_llseek,
};

struct proc_dir_entry *abox_proc_create_bool(const char *name, umode_t mode,
		struct proc_dir_entry *parent, bool *data)
{
	return abox_proc_create_file(name, mode, parent, &abox_proc_bool_fops,
			data, 0);
}

int abox_proc_symlink_attr(struct device *dev, const char *name,
		struct proc_dir_entry *parent)
{
	char *path, *attr_path;
	struct proc_dir_entry *pde;

	if (!parent)
		parent = root;

	path = kobject_get_path(&dev->kobj, GFP_KERNEL);
	attr_path = kasprintf(GFP_KERNEL, "/sys%s/%s", path, name);
	pde = proc_symlink(name, parent, attr_path);
	kfree(attr_path);
	kfree(path);

	return PTR_ERR_OR_ZERO(pde);
}

int abox_proc_symlink_kobj(const char *name, struct kobject *kobj)
{
	char *path, *sys_path;
	struct proc_dir_entry *pde;

	path = kobject_get_path(kobj, GFP_KERNEL);
	sys_path = kasprintf(GFP_KERNEL, "/sys%s", path);
	pde = proc_symlink(name, root, sys_path);
	kfree(sys_path);
	kfree(path);

	return PTR_ERR_OR_ZERO(pde);
}

int abox_proc_probe(void)
{
	root = proc_mkdir(ROOT_DIR_NAME, NULL);

	/* for backward compatibility */
	debugfs_link = debugfs_create_symlink(ROOT_DIR_NAME, NULL,
			"/proc/"ROOT_DIR_NAME);

	return PTR_ERR_OR_ZERO(root);
}

void abox_proc_remove(void)
{
	/* for backward compatibility */
	debugfs_remove(debugfs_link);

	proc_remove(root);
}
