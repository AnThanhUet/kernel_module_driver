/*
 * ten file: vchar_driver.c
 * tac gia : theanuet@gmail.com
 * ngay tao: 18/11/2020
 * mo ta   : char driver cho thiet bi gia lap vchar_device.
 *           vchar_device la mot thiet bi nam tren RAM.
 */

#include <linux/module.h> /* thu vien nay dinh nghia cac macro nhu module_init va module_exit */
#include <linux/fs.h> /*cun cap cac ham cap phat device number*/
#include <linux/device.h> /* tao device file */
#include <linux/slab.h> /* chua ham kmalloc va kfree */
#include <linux/cdev.h> /* cac ham lam viec cdev */
#include <linux/uaccess.h> /* chua cac ham trao doi data user <-> kernel */

#include "vchar_driver.h"

#define DRIVER_AUTHOR "Nguyen The An <theanuet@gmail.com>"
#define DRIVER_DESC   "A sample character device driver"
#define DRIVER_VERSION "0.1"

typedef struct vchar_dev
{
	unsigned char * control_regs;
	unsigned char * status_regs;
	unsigned char * data_regs;
}vchar_dev_t;

struct _var_drv
{
	dev_t dev_num;
	struct class * dev_class;
	struct device * dev;
	vchar_dev_t * vchar_hw;
	struct cdev * vcdev;
	unsigned int open_cnt;
}vchar_drv;

/****************************** device specific - START *****************************/
/* ham khoi tao thiet bi */
int vchar_hw_init(vchar_dev_t *hw)
{
	char * buf;
	buf = kzalloc(NUM_DEV_REGS * REG_SIZE, GFP_KERNEL);
	if(!buf)
	{
		return -ENOMEM;
	}
	
	hw -> control_regs = buf;
	hw -> status_regs = hw -> control_regs + NUM_CTRL_REGS;
	hw -> data_regs = hw -> status_regs + NUM_STS_REGS;

	/* Khoi tao gia tri ban dau cho cac thanh ghi */
	hw -> control_regs[CONTROL_ACCESS_REG] = 0x03;
	hw -> status_regs[DEVICE_STATUS_REG] = 0x03;

	return 0;
}

/* ham giai phong thiet bi */
void vchar_hw_exit(vchar_dev_t *hw)
{
	kfree(hw -> control_regs);
}

/* ham doc tu cac thanh ghi du lieu cua thiet bi */
int vchar_hw_read_data(vchar_dev_t *hw, int start_reg, int num_regs, char *kbuf)
{
	int read_bytes = num_regs;

	/* ktra quyen doc hay khong */
	if((hw -> control_regs[CONTROL_ACCESS_REG] & CTRL_READ_DATA_BIT) == DISABLE)
	{
		return -1;
	}
	/* ktra dia chi kernel buffer co hop le khong */
	if(kbuf == NULL)
	{
		return -1;
	}
	/* ktra vi tri cac thanh ghi can doc co hop ly khong */
	if(start_reg > NUM_DATA_REGS)
	{
		return -1;
	}
	/* dieu chinh lai so luong thanh ghi du lieu can doc */
	if(num_regs > (NUM_DATA_REGS - start_reg))
	{
		read_bytes = NUM_DATA_REGS - start_reg;
	}
	/* ghi data kernel buffer vao thanh ghi data */
	memcpy(kbuf, hw -> data_regs + start_reg, read_bytes);
	/* cap nhat so lan doc tu thanh ghi data */
	hw -> status_regs[READ_COUNT_L_REG] += 1;
	if(hw -> status_regs[READ_COUNT_L_REG] == 0)
	{
		hw -> status_regs[READ_COUNT_H_REG] += 1;
	}
	/* tra ve so byte doc duoc tu cac thanh ghi data */
	return read_bytes;
}

/* ham ghi vao cac thanh ghi du lieu cua thiet bi */
int vchar_hw_write_data(vchar_dev_t *hw, int start_reg, int num_regs, char *kbuf)
{
	int write_bytes = num_regs;

	/* ktra quyen ghi hay khong */
	if((hw -> control_regs[CONTROL_ACCESS_REG] & CTRL_WRITE_DATA_BIT) == DISABLE)
	{
		return -1;
	}
	/* ktra dia chi kernel buffer co hop le khong */
	if(kbuf == NULL)
	{
		return -1;
	}
	/* ktra vi tri cac thanh ghi can ghi co hop ly khong */
	if(start_reg > NUM_DATA_REGS)
	{
		return -1;
	}
	/* dieu chinh lai so luong thanh ghi du lieu can ghi */
	if(num_regs > (NUM_DATA_REGS - start_reg))
	{
		write_bytes = NUM_DATA_REGS - start_reg;
		hw -> status_regs[DEVICE_STATUS_REG] |= STS_DATAREGS_OVERFLOW_BIT;
	}
	/* doc data thanh ghi data vao kernel buffer*/
	memcpy(hw -> data_regs + start_reg, kbuf, write_bytes);

	/* cap nhat so lan doc tu thanh ghi data */
	hw -> status_regs[WRITE_COUNT_L_REG] += 1;
	if(hw -> status_regs[WRITE_COUNT_L_REG] == 0)
	{
		hw -> status_regs[WRITE_COUNT_H_REG] += 1;
	}
	/* tra ve so byte ghi vao tu cac thanh ghi data */
	return write_bytes;
}
/* ham doc tu cac thanh ghi trang thai cua thiet bi */

/* ham ghi vao cac thanh ghi dieu khien cua thiet bi */

/* ham xu ly tin hieu ngat gui tu thiet bi */

/******************************* device specific - END *****************************/

/******************************** OS specific - START *******************************/
/* cac ham entry points */
static int vchar_driver_open(struct inode *inode, struct file *filp)
{
	vchar_drv.open_cnt++;
	printk("Handle openned event (%d)\n", vchar_drv.open_cnt);
	return 0;
}

static int vchar_driver_release(struct inode *inode, struct file *filp)
{
	printk("Handle closed event\n");
	return 0;
}

static ssize_t vchar_driver_read(struct file *filp, char __user *user_buf, size_t len, loff_t *off)
{
	char *kernel_buf = NULL;
	int num_bytes = 0;

	printk("Handle read event start from %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		return 0;
	}

	num_bytes = vchar_hw_read_data(vchar_drv.vchar_hw, *off, len, kernel_buf);
	printk("read % byte from HW\n", num_bytes);

	if(num_bytes < 0)
	{
		return -EFAULT;
	}
	if(copy_to_user(user_buf, kernel_buf, num_bytes))
	{
		return -EFAULT;
	}
	*off += num_bytes;
	return num_bytes;
}

static ssize_t vchar_driver_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	char *kernel_buf = NULL;
	int num_bytes = 0;
	printk("Handle write event start from %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);
	if(copy_from_user(kernel_buf, user_buf, len))
	{
		return -EFAULT;
	}

	num_bytes = vchar_hw_write_data(vchar_drv.vchar_hw, *off, len, kernel_buf);
	printk("Write %d bytes to HW\n", num_bytes);

	if(num_bytes < 0)
	{
		return -EFAULT;
	}
	*off += num_bytes;
	return num_bytes;
}

static struct file_operations fops =
{
	.owner 		= THIS_MODULE,
	.open 		= vchar_driver_open,
	.release 	= vchar_driver_release,
	.read 		= vchar_driver_read,
	.write 		= vchar_driver_write,
};
/*==============================================================================================================================
									KHOI TAO DRIVER
================================================================================================================================*/
/* ham khoi tao driver */
static int __init vchar_driver_init(void)
{
	int ret = 0;
	/* cap phat device number */
	vchar_drv.dev_num = 0;
	ret = alloc_chrdev_region(&vchar_drv.dev_num, 0, 1, "vchar_device");
	if(ret < 0)
	{
		printk("failed register device number\n");
		goto failed_register_devnum;
	}
	printk("Device number (major: %d, minor: %d)\n", MAJOR(vchar_drv.dev_num), MINOR(vchar_drv.dev_num));
	

	/* tao device file */
	vchar_drv.dev_class = class_create(THIS_MODULE, "class_vchar_dev");
	if(vchar_drv.dev_class == NULL)
	{
		printk("failed to create a device class\n");
		goto failed_create_class;

	}
	vchar_drv.dev = device_create(vchar_drv.dev_class, NULL, vchar_drv.dev_num, NULL, "vchar_dev");
	if(IS_ERR(vchar_drv.dev))
	{
		printk("failed create device\n");
		goto failed_create_device;
	}

	/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao */
	vchar_drv.vchar_hw = kzalloc(sizeof(vchar_dev_t), GFP_KERNEL);
	if(!vchar_drv.vchar_hw)
	{
		printk("failed allocate data structure driver\n");
		ret = -ENOMEM;
		goto failed_allocate_structure;
	}

	/* khoi tao thiet bi vat ly */
	ret = vchar_hw_init(vchar_drv.vchar_hw);
	if(ret < 0)
	{
		printk("fail initalize a virtual char device\n");
		goto failed_init_hw;
	}

	/* dang ky cac entry point voi kernel */
	vchar_drv.vcdev = cdev_alloc();
	if(vchar_drv.vcdev == NULL)
	{
		printk("fail allocate cdev structure\n");
	}
	cdev_init(vchar_drv.vcdev, &fops);
	ret = cdev_add(vchar_drv.vcdev, vchar_drv.dev_num, 1);
	if(ret < 0)
	{
		printk("faile add char device system\n");
	}

	/* dang ky ham xu ly ngat */


	printk("Initialize vchar driver successfully - add read write\n");
	return 0;

failed_init_hw:
	kfree(vchar_drv.vchar_hw);
failed_allocate_structure:
	device_destroy(vchar_drv.dev_class, vchar_drv.dev_num);
failed_create_device:
	class_destroy(vchar_drv.dev_class);
failed_create_class:
	unregister_chrdev_region(vchar_drv.dev_num, 1);
failed_register_devnum:
	return ret;

}

/*==============================================================================================================================
									KET THUC DRIVER
================================================================================================================================*/
/* ham ket thuc driver */
static void __exit vchar_driver_exit(void)
{
	/* huy dang ky xu ly ngat */

	/* huy dang ky entry point voi kernel */
	cdev_del(vchar_drv.vcdev);

	/* giai phong thiet bi vat ly */
	vchar_hw_exit(vchar_drv.vchar_hw);

	/* giai phong bo nho da cap phat cau truc du lieu cua driver */
	kfree(vchar_drv.vchar_hw);	

	/* xoa bo device file */
	device_destroy(vchar_drv.dev_class, vchar_drv.dev_num);
	class_destroy(vchar_drv.dev_class);

	/* giai phong device number */
	unregister_chrdev_region(vchar_drv.dev_num, 1);
	printk("Exit vchar driver - add read write\n");
}
/********************************* OS specific - END ********************************/

module_init(vchar_driver_init);
module_exit(vchar_driver_exit);

MODULE_LICENSE("GPL"); /* giay phep su dung cua module */
MODULE_AUTHOR(DRIVER_AUTHOR); /* tac gia cua module */
MODULE_DESCRIPTION(DRIVER_DESC); /* mo ta chuc nang cua module */
MODULE_VERSION(DRIVER_VERSION); /* mo ta phien ban cuar module */
MODULE_SUPPORTED_DEVICE("testdevice"); /* kieu device ma module ho tro */