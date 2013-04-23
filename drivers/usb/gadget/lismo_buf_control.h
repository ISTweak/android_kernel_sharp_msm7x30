#define SC_VENDOR_START			0xe4
#define SC_VENDOR_END			0xef
#define VENDOR_CMD_NR	(SC_VENDOR_END - SC_VENDOR_START + 1)

struct op_desc {
	struct device	dev;
	unsigned long	flags;
/* flag symbols are bit numbers */
#define FLAG_IS_READ	0	/* not use */
#define FLAG_IS_WRITE	1	/* not use */
#define FLAG_EXPORT	2	/* protected by sysfs_lock */
	char			*buffer;
	size_t			len;
	struct bin_attribute	dev_bin_attr_buffer;
	unsigned long 		update;
	struct work_struct	work;
	struct sysfs_dirent	*value_sd;
};

/* buffer size alloc at __init() */
#define ALLOC_INI_SIZE  0x101000
#define ALLOC_CMD_CNT   1

static void op_release(struct device *dev);

static DEFINE_MUTEX(sysfs_lock);

struct fsg_lun {
	struct device	dev;
	struct op_desc *op_desc[VENDOR_CMD_NR];
	char   *reserve_buf[VENDOR_CMD_NR];
};

#define fsg_lun_is_open(curlun)	((curlun)->filp != NULL)

static struct fsg_lun *fsg_lun_from_dev(struct device *dev)
{
	return container_of(dev, struct fsg_lun, dev);
}

static struct op_desc *dev_to_desc(struct device *dev)
{
	return container_of(dev, struct op_desc, dev);
}

