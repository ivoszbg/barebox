// SPDX-License-Identifier: GPL-2.0-only
/*
 * resource.c - barebox resource management
 *
 * Copyright (c) 2011 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 */
#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <init.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/resource_ext.h>
#include <asm/io.h>

#ifdef CONFIG_DEBUG_RESOURCES
#define res_dbg(args...)	pr_info(args)
#else
#define res_dbg(args...)	pr_debug(args)
#endif

static int init_resource(struct resource *res, const char *name)
{
	INIT_LIST_HEAD(&res->children);
	res->parent = NULL;
	res->name = xstrdup_const(name);

	return 0;
}

/*
 * request a region.
 * This will succeed when the requested region is completely inside
 * the parent resource and does not conflict with any of the child
 * resources.
 */
struct resource *__request_region(struct resource *parent,
				  resource_size_t start, resource_size_t end,
				  const char *name, unsigned flags)
{
	struct resource *r, *new;

	if (end < start) {
		res_dbg("request region(0x%08llx:0x%08llx): end < start\n",
				(unsigned long long)start,
				(unsigned long long)end);
		return ERR_PTR(-EINVAL);
	}

	/* outside parent resource? */
	if (start < parent->start || end > parent->end) {
		res_dbg("request region(0x%08llx:0x%08llx): outside parent resource 0x%08llx:0x%08llx\n",
				(unsigned long long)start,
				(unsigned long long)end,
				(unsigned long long)parent->start,
				(unsigned long long)parent->end);
		return ERR_PTR(-EINVAL);
	}

	/*
	 * We keep the list of child resources ordered which helps
	 * us searching for conflicts here.
	 */
	list_for_each_entry(r, &parent->children, sibling) {
		if (end < r->start)
			goto ok;
		if (start > r->end)
			continue;
		res_dbg("request region(0x%08llx:0x%08llx): %s conflicts with 0x%08llx:0x%08llx (%s)\n",
				(unsigned long long)start,
				(unsigned long long)end,
				name,
				(unsigned long long)r->start,
				(unsigned long long)r->end,
				r->name);
		return ERR_PTR(-EBUSY);
	}

ok:
	res_dbg("request region(0x%08llx:0x%08llx): success flags=0x%x\n",
			(unsigned long long)start,
			(unsigned long long)end, flags);

	new = xzalloc(sizeof(*new));
	init_resource(new, name);
	new->start = start;
	new->end = end;
	new->parent = parent;
	new->flags = flags;
	list_add_tail(&new->sibling, &r->sibling);

	return new;
}

/*
 * release a region previously requested with request_*_region
 */
int release_region(struct resource *res)
{
	if (!list_empty(&res->children))
		return -EBUSY;

	list_del(&res->sibling);
	free_const(res->name);
	free(res);

	return 0;
}


/*
 * merge two adjacent sibling regions.
 */
int __merge_regions(const char *name,
		struct resource *resa, struct resource *resb)
{
	if (!resource_adjacent(resa, resb))
		return -EINVAL;

	if (resa->start < resb->start)
		resa->end = resb->end;
	else
		resa->start = resb->start;

	free_const(resa->name);
	resa->name = xstrdup_const(name);
	release_region(resb);

	return 0;
}

/* The root resource for the whole memory-mapped io space */
struct resource iomem_resource = {
	.start = 0,
	.end = ~(resource_size_t)0,
	.name = "iomem",
	.children = LIST_HEAD_INIT(iomem_resource.children),
};

/*
 * request a region inside the io space (memory)
 */
struct resource *request_iomem_region(const char *name,
		resource_size_t start, resource_size_t end)
{
	return __request_region(&iomem_resource, start, end, name, 0);
}

/* The root resource for the whole io-mapped io space */
struct resource ioport_resource = {
	.start = 0,
	.end = IO_SPACE_LIMIT,
	.name = "ioport",
	.children = LIST_HEAD_INIT(ioport_resource.children),
};

/*
 * request a region inside the io space (i/o port)
 */
struct resource *request_ioport_region(const char *name,
		resource_size_t start, resource_size_t end)
{
	struct resource *res;

	res = __request_region(&ioport_resource, start, end, name, 0);
	if (IS_ERR(res))
		return ERR_CAST(res);

	return res;
}

struct resource_entry *resource_list_create_entry(struct resource *res,
						  size_t extra_size)
{
	struct resource_entry *entry;

	entry = kzalloc(sizeof(*entry) + extra_size, GFP_KERNEL);
	if (entry) {
		INIT_LIST_HEAD(&entry->node);
		entry->res = res ? res : &entry->__res;
	}

	return entry;
}
EXPORT_SYMBOL(resource_list_create_entry);

static const char memory_type_name[][13] = {
	"Reserved",
	"Loader Code",
	"Loader Data",
	"Boot Code",
	"Boot Data",
	"Runtime Code",
	"Runtime Data",
	"Conventional",
	"Unusable",
	"ACPI Reclaim",
	"ACPI Mem NVS",
	"MMIO",
	"MMIO Port",
	"PAL Code",
	"Persistent",
	"Unaccepted",
};

const char *resource_typeattr_format(char *buf, size_t size,
				     const struct resource *res)
{
	char *pos;
	int type_len;
	u64 attr;

	if (!(res->flags & IORESOURCE_TYPE_VALID))
		return NULL;

	pos = buf;
	type_len = snprintf(pos, size, "[%-*s",
			    (int)(sizeof(memory_type_name[0]) - 1),
			    memory_type_name[res->type]);
	if (type_len >= size)
		return buf;

	pos += type_len;
	size -= type_len;

	attr = res->attrs;
	if (attr & ~(MEMATTR_UC | MEMATTR_WC | MEMATTR_WT |
		     MEMATTR_WB | MEMATTR_UCE | MEMATTR_RO |
		     MEMATTR_WP | MEMATTR_RP | MEMATTR_XP |
		     MEMATTR_NV | MEMATTR_SP | MEMATTR_MORE_RELIABLE)
		     )
		snprintf(pos, size, "|attr=0x%08llx]",
			 (unsigned long long)attr);
	else
		snprintf(pos, size,
			 "|%3s|%2s|%2s|%2s|%2s|%2s|%2s|%2s|%3s|%2s|%2s|%2s|%2s]",
			 res->runtime			? "RUN" : "",
			 attr & MEMATTR_MORE_RELIABLE	? "MR"  : "",
			 attr & MEMATTR_SP		? "SP"  : "",
			 attr & MEMATTR_NV		? "NV"  : "",
			 attr & MEMATTR_XP		? "XP"  : "",
			 attr & MEMATTR_RP		? "RP"  : "",
			 attr & MEMATTR_WP		? "WP"  : "",
			 attr & MEMATTR_RO		? "RO"  : "",
			 attr & MEMATTR_UCE		? "UCE" : "",
			 attr & MEMATTR_WB		? "WB"  : "",
			 attr & MEMATTR_WT		? "WT"  : "",
			 attr & MEMATTR_WC		? "WC"  : "",
			 attr & MEMATTR_UC		? "UC"  : "");
	return buf;
}
