#include<linux/module.h>
#include<linux/types.h>
#include<linux/fs.h>
#include<linux/errno.h>
#include<linux/mm.h>
#include<linux/slab.h>
#include<linux/sched.h>
#include<linux/init.h>
#include<asm/io.h>
#include<asm/system.h>
#include<asm/uaccess.h>
#include<linux/cdev.h>
#include<linux/semaphore.h>

#define GLOBALMEM_SEM_SIZE 0x1000
#define MEM_CLEAR 0x1
#define GLOBALMEM_SEM_MAJOR 250

int globalmem_major = GLOBALMEM_SEM_MAJOR;

struct globalmem_dev {
	struct cdev cdev;
	unsigned char mem[GLOBALMEM_SEM_SIZE];
	struct semaphore sem;
};

struct globalmem_dev *globalmem_devp;

int globalmem_open(struct inode *inode, struct file *filp);
int globalmem_release(struct inode *inode ,struct file* filp);
long globalmem_ioctl(struct file *filp ,unsigned int cmd , unsigned long arg);
static ssize_t globalmem_read(struct file *filp, char __user *buf ,size_t size ,loff_t *ppos);
static ssize_t globalmem_write(struct file *filp, const char __user *buf ,size_t size ,loff_t *ppos);
static loff_t globalmem_llseek(struct file *filp, loff_t offset , int orig);
int __init globalmem_init(void);
void __exit globalmem_exit(void);

static const struct file_operations globalmem_fops = {
	.owner = THIS_MODULE,
	.llseek = globalmem_llseek,
	.release = globalmem_release,
	.unlocked_ioctl = globalmem_ioctl,
	.read = globalmem_read,
	.write = globalmem_write,
	.open = globalmem_open,
};

int globalmem_open(struct inode *inode, struct file *filp)
{
	filp->private_data = globalmem_devp;
	return 0;
}

int globalmem_release(struct inode *inode ,struct file* filp)
{
	return 0;
}

long globalmem_ioctl(struct file *filp, unsigned int cmd , unsigned long arg)
{
	struct globalmem_dev *dev = filp->private_data;

	switch(cmd) {
	case MEM_CLEAR:
		if(down_interruptible(&dev->sem)) 
			return -ERESTARTSYS;
		memset(dev->mem, 0 , GLOBALMEM_SEM_SIZE);
		printk(KERN_INFO "[GLOBALMEM_SEM]: globalmem is clear to zero\n");
		up(&dev->sem);
		break;
	default:
		return -EINVAL; 
	}

	return 0;
}

static ssize_t globalmem_read(struct file *filp, char __user *buf ,size_t size ,loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	
	struct globalmem_dev *dev = filp->private_data;
	
	if(p >= GLOBALMEM_SEM_SIZE)
		return 0;
	if(count > GLOBALMEM_SEM_SIZE - p)
		count = GLOBALMEM_SEM_SIZE - p;
	
	if(down_interruptible(&dev->sem)) {
		printk(KERN_INFO "there're another process is reading.....");
		return -ERESTARTSYS;
	}
	if(copy_to_user(buf,(void *)(dev->mem+p),count)) {
		ret = -EFAULT;
	} else {
		*ppos += count;
		ret = count;
		printk(KERN_INFO "[GLOBALMEM_SEM]: read %u bytes from %lu\n",count,p);
	}
	up(&dev->sem);
	return ret;
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf ,size_t size ,loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	
	struct globalmem_dev *dev = filp->private_data;
	
	if(p > GLOBALMEM_SEM_SIZE)
		return 0;
	if(count > GLOBALMEM_SEM_SIZE - p)
		count = GLOBALMEM_SEM_SIZE - p;
	
	if(down_interruptible(&dev->sem)) {
		printk(KERN_INFO "there're another process is writeing.....");
		return -ERESTARTSYS;
	}
	
	if(copy_from_user((void *)(dev->mem+p),buf,count)) {
		ret = -EFAULT;
	} else {
		*ppos += count;
		ret = count;
		printk(KERN_INFO "[GLOBALMEM_SEM]: write %u bytes from %lu\n",count,p);
	}
	up(&dev->sem);
	return ret;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset , int orig)
{
	loff_t ret = 0;
	switch(orig) {
	case 0:
		if(offset < 0) {
			ret = -EINVAL;
			break;
		}
		if((unsigned int ) offset > GLOBALMEM_SEM_SIZE) {
			ret = -EINVAL;
			break;
		}
		filp->f_pos = (unsigned int ) offset;
		ret = filp->f_pos;
		break;
	case 1:
		if((filp->f_pos + offset) > GLOBALMEM_SEM_SIZE) {
			ret = -EINVAL;
			break;
		}
		
		if((filp->f_pos + offset) < 0) {
			ret = -EINVAL;
			break;
		}
		filp->f_pos += (unsigned int ) offset;
		ret = filp->f_pos;
		break;
	default:
		ret = -EINVAL;
		break;
	} 
	return ret;
}

static void globalmem_setup_cdev(struct globalmem_dev *dev,int index)
{
	int err ,devno = MKDEV(globalmem_major, index);
	
	cdev_init(&dev->cdev , &globalmem_fops); 
	dev->cdev.owner = THIS_MODULE;
	err= cdev_add(&dev->cdev,devno,1);
	printk("[GLOBALMEM_SEM]: char device globalmem has regitered as %d\n",globalmem_major);
	if(err)
		printk(KERN_NOTICE "[GLOBALMEM_SEM]: Error %d adding globalmem %d",err,devno);
}

int __init globalmem_init(void)
{
	int result ;
	dev_t devno = MKDEV(globalmem_major,0);
	
	if(globalmem_major) 
		result = register_chrdev_region(devno,1,"globalmem");
	else {
		result = alloc_chrdev_region(&devno,0,1,"globalmem");
		globalmem_major = MAJOR(devno); 
	}

	if(result < 0 )
		return result;
	
	globalmem_devp = kmalloc(sizeof(struct globalmem_dev),GFP_KERNEL);
	
	if(!globalmem_devp) {
		result = -ENOMEM;
		goto fail_malloc;
	}
	
	memset(globalmem_devp,0,sizeof(struct globalmem_dev));
	globalmem_setup_cdev(globalmem_devp,0);
	sema_init(&globalmem_devp->sem,1);
	return 0;

fail_malloc:
	unregister_chrdev_region(devno,1);
	return result;
}

void globalmem_exit(void)
{
	cdev_del(&globalmem_devp->cdev);
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major,0),1);
}

MODULE_LICENSE("Dual BSD/GPL");
module_param(globalmem_major, int ,S_IRUGO);
module_init(globalmem_init);
module_exit(globalmem_exit);
