/*
 * APEI Boot Error Record Table (BERT) support
 *
 * Copyright 2010 Intel Corp.
 *   Author: Huang Ying <ying.huang@intel.com>
 *
 * Under normal circumstances, when a hardware error occurs, kernel
 * will be notified via NMI, MCE or some other method, then kernel
 * will process the error condition, report it, and recover it if
 * possible. But sometime, the situation is so bad, so that firmware
 * may choose to reset directly without notifying Linux kernel.
 *
 * Linux kernel can use the Boot Error Record Table (BERT) to get the
 * un-notified hardware errors that occurred in a previous boot.
 *
 * For more information about ERST, please refer to ACPI Specification
 * version 4.0, section 17.3.1
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/list.h>

#include "apei-internal.h"

#define BERT_PFX "BERT: "

int bert_disable;

struct bert_blob {
	struct list_head list;
	char name[64];
	struct debugfs_blob_wrapper blob;
};

struct bert_ctx {
	u64 region_address;
	u32 region_len;
	struct acpi_hest_generic_status *bert_region;
	struct dentry *bert_dir;
	struct list_head blob_list;
	unsigned int blob_count;
};

static struct bert_ctx *bert_ctx;

static void bert_add_fw_crashlog_data(const struct cper_fw_err_rec_ref *fw_err,
				      size_t len)
{
	struct bert_blob *bblob;
	struct dentry *file;
	struct fw_err_type_crashlog *fw_cl;

	if (!fw_err || !len || !bert_ctx)
		return;

	if (len <= sizeof(*fw_err) ||
	    fw_err->error_type != CPER_FWERR_TYPE_IFW_CRASHLOG)
		return;

	bblob = kzalloc(sizeof(*bblob), GFP_KERNEL);
	if (!bblob)
		return;

	fw_cl = (struct fw_err_type_crashlog *)&fw_err->identifier;

	bblob->blob.data = (void *)(fw_err + 1);
	bblob->blob.size = len - sizeof(*fw_err);
	snprintf(bblob->name, sizeof(bblob->name), "%02d-fwerr-crashlog-%u.bin",
		 bert_ctx->blob_count, fw_cl->source);

	file = debugfs_create_blob(bblob->name, S_IRUSR,
				   bert_ctx->bert_dir, &bblob->blob);
	if (!file) {
		kfree(bblob);
		return;
	}

	list_add(&bblob->list, &bert_ctx->blob_list);
	bert_ctx->blob_count++;
}

static void __init bert_dump_all(struct acpi_hest_generic_status *region,
				  unsigned int region_len)
{
	int remain, first = 1;
	u32 estatus_len;
	struct acpi_hest_generic_status *estatus;

	remain = region_len;
	estatus = region;
	while (remain > sizeof(struct acpi_hest_generic_status)) {
		/* No more error record */
		if (!estatus->block_status)
			break;

		estatus_len = cper_estatus_len(estatus);
		if (estatus_len < sizeof(struct acpi_hest_generic_status) ||
		    remain < estatus_len) {
			pr_err(FW_BUG BERT_PFX "Invalid error status block with length %u\n",
			       estatus_len);
			return;
		}

		if (cper_estatus_check(estatus)) {
			pr_err(FW_BUG BERT_PFX "Invalid Error status block\n");
			goto next;
		}

		if (first) {
			pr_info(HW_ERR "Error record from previous boot:\n");
			first = 0;
		}
		cper_estatus_dump_data(KERN_INFO HW_ERR, estatus,
				       bert_add_fw_crashlog_data);
next:
		estatus = (void *)estatus + estatus_len;
		remain -= estatus_len;
	}
}

static int __init setup_bert_disable(char *str)
{
	bert_disable = 1;
	return 0;
}
__setup("bert_disable", setup_bert_disable);

static int __init bert_check_table(struct acpi_table_bert *bert_tab)
{
	if (bert_tab->header.length != sizeof(struct acpi_table_bert))
		return -EINVAL;
	if (bert_tab->region_length != 0 &&
	    bert_tab->region_length < sizeof(struct acpi_bert_region))
		return -EINVAL;

	return 0;
}

static int __init bert_init(void)
{
	acpi_status status;
	struct acpi_table_bert *bert_tab;
	struct bert_ctx *ctx;
	struct resource *r;
	int rc = -EINVAL;

	if (acpi_disabled)
		goto out;

	if (bert_disable) {
		pr_info(BERT_PFX "Boot Error Record Table (BERT) support is disabled.\n");
		goto out;
	}

	status = acpi_get_table(ACPI_SIG_BERT, 0,
				(struct acpi_table_header **)&bert_tab);
	if (status == AE_NOT_FOUND) {
		pr_err(BERT_PFX "Table is not found!\n");
		goto out;
	} else if (ACPI_FAILURE(status)) {
		const char *msg = acpi_format_exception(status);
		pr_err(BERT_PFX "Failed to get table, %s\n", msg);
		rc = -EINVAL;
		goto out;
	}

	rc = bert_check_table(bert_tab);
	if (rc) {
		pr_err(FW_BUG BERT_PFX "BERT table is invalid\n");
		goto out;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		pr_err(BERT_PFX "Could not allocate ctx.\n");
		goto out;
	}

	ctx->region_address = bert_tab->address;
	ctx->region_len = bert_tab->region_length;
	if (!ctx->region_len) {
		rc = 0;
		goto out_ctx;
	}

	r = request_mem_region(ctx->region_address, ctx->region_len,
			       "APEI BERT");
	if (!r) {
		pr_err(BERT_PFX "Can not request iomem region <%016llx-%016llx> for BERT.\n",
		       (unsigned long long)bert_tab->address,
		       (unsigned long long)bert_tab->address + ctx->region_len - 1);
		rc = -EIO;
		goto out_ctx;
	}

	ctx->bert_region = ioremap_cache(ctx->region_address, ctx->region_len);
	if (!ctx->bert_region) {
		rc = -ENOMEM;
		goto out_release;
	}

	ctx->bert_dir = debugfs_create_dir("bert", apei_get_debugfs_dir());
	INIT_LIST_HEAD(&ctx->blob_list);
	ctx->blob_count = 0;
	bert_ctx = ctx;

	bert_dump_all(ctx->bert_region, ctx->region_len);

	return 0;

out_release:
	release_mem_region(ctx->region_address, ctx->region_len);
out_ctx:
	kfree(ctx);
out:
	if (rc)
		bert_disable = 1;

	return rc;
}
late_initcall(bert_init);

static void bert_blob_list_free(struct list_head *blob_list)
{
	struct bert_blob *blob, *blobn;

	list_for_each_entry_safe(blob, blobn, blob_list, list) {
		list_del(&blob->list);
		kfree(blob);
	}
}

static void bert_exit(void)
{
	if (!bert_ctx)
		return;

	debugfs_remove_recursive(bert_ctx->bert_dir);
	bert_blob_list_free(&bert_ctx->blob_list);
	iounmap(bert_ctx->bert_region);
	release_mem_region(bert_ctx->region_address, bert_ctx->region_len);
	kfree(bert_ctx);
}
module_exit(bert_exit);
