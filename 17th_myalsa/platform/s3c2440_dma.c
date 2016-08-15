#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>


/* 参考 sound\soc\samsung\dma.c
 */

/* 1. 分配DMA BUFFER,这个 buffer 用于DMA传输         s3c2440_dma_new
 * 2. 从BUFFER取出period
 * 3. 启动DMA传输
 * 4. 一次传输完毕,更新状态(hw_ptr)
 *    2,3,4这部分主要有: request_irq, 触发DMA传输, 中断处理
 */

#define DMA0_BASE_ADDR  0x4B000000  // DMA0 寄存器物理地址
#define DMA1_BASE_ADDR  0x4B000040  // DMA1 寄存器物理地址
#define DMA2_BASE_ADDR  0x4B000080  // DMA2 寄存器物理地址
#define DMA3_BASE_ADDR  0x4B0000C0  // DMA3 寄存器物理地址


// 构造 s3c dma 寄存器结构体类型
struct s3c_dma_regs {
	unsigned long disrc;
	unsigned long disrcc;
	unsigned long didst;
	unsigned long didstc;
	unsigned long dcon;
	unsigned long dstat;
	unsigned long dcsrc;
	unsigned long dcdst;
	unsigned long dmasktrig;
};

static volatile struct s3c_dma_regs *dma_regs;  // 定义一个结构体指针，这个指针指向 s3c dma 寄存器


// 应用程序把一段数据写入buffer，再把下一段数据写入buffer….
// 应用程序写入的位置称为 appl_ptr , 每写入的一段数据称为一个 period,每个 period里面有多个frame，一个frame里面有一个采样点的数据。
// 驱动程序从这个buffer里面取出一个period数据发给硬件，再取出一个period数据发给硬件….
// 驱动程序读出的位置称为hw_ptr。
// 这种方法可以解决声音断断续续的问题。因为数据是源源不断地从应用程序发给驱动，从驱动程序发给硬件。
// 使用这种方法就必须创建一个非常大的缓冲区，这个缓冲区是在驱动程序里面申请的。


// 构造一个描述 dma_buffer 结构体类型
struct s3c2440_dma_info {
    unsigned int buf_max_size;  // 用于存放驱动分配的 dma_buffer 最大大小  .  你尽管分配dma_buffer大小 , 但是到底用多大，由APP决定
    unsigned int buffer_size;   // 用于存放应用程序使用了 dma_buffer 多大空间
    unsigned int period_size;   // 用于存放一个 period 的大小
    unsigned int phy_addr;      // 用于存放驱动分配的 dma_buffer 物理地址
    unsigned int virt_addr;     // 用于存放驱动分配的 dma_buffer 虚拟地址
    unsigned int dma_ofs;       // dma 偏移地址
    unsigned int be_running;    // 是否还在运行 dma 传输  1  --- 还在进行DMA传输    0 --- 未进行DMA传输
};

static struct s3c2440_dma_info playback_dma_info; // 定义一个描述用于播放的 dma_buffer 结构体变量

                                                  // 如果你还要录音的话，再定义一个描述录音的结构体变量
                                                  // 本程序只演示播放功能


// SNDRV_PCM_INFO_INTERLEAVED  数据存放的排列方式，左声道右声道数据交叉排列

static const struct snd_pcm_hardware s3c2440_dma_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				    SNDRV_PCM_INFO_BLOCK_TRANSFER |
				    SNDRV_PCM_INFO_MMAP |
				    SNDRV_PCM_INFO_MMAP_VALID |
				    SNDRV_PCM_INFO_PAUSE |
				    SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_U16_LE |
				    SNDRV_PCM_FMTBIT_U8 |
				    SNDRV_PCM_FMTBIT_S8,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 128*1024,     // dma_buffer  128k  . 你尽管分配 128k , 但是用不用到 128k 由APP决定
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};


// 应用程序把一段数据写入buffer，再把下一段数据写入buffer….
// 应用程序写入的位置称为 appl_ptr , 每写入的一段数据称为一个 period,每个 period里面有多个frame，一个frame里面有一个采样点的数据。
// 驱动程序从这个buffer里面取出一个period数据发给硬件，再取出一个period数据发给硬件….
// 驱动程序读出的位置称为hw_ptr。
// 这种方法可以解决声音断断续续的问题。因为数据是源源不断地从应用程序发给驱动，从驱动程序发给硬件。
// 使用这种方法就必须创建一个非常大的缓冲区，这个缓冲区是在驱动程序里面申请的。


/* 数据传输: 源,目的,长度 */
// 把一个 period 数据加载进 DMA 
static void load_dma_period(void)
{       
	/* 把源地址,目的地址,要传输的数据长度告诉DMA */
	dma_regs->disrc      = playback_dma_info.phy_addr + playback_dma_info.dma_ofs; /* 源的物理地址 */
	dma_regs->disrcc     = (0<<1) | (0<<0);                                       /* 源位于AHB总线, 源地址递增 */
	dma_regs->didst      = 0x55000010;                                           /* 目的的物理地址 --- 是 s3c IIS 控制器 */
	dma_regs->didstc     = (0<<2) | (1<<1) | (1<<0);                              /* 目的位于APB总线, 目的地址不变 */

    /* bit22: 0 -  reload   表示一次DMA传输完之后，会自己加载某些值，然后启动DMA */
	/* bit22: 1 -  noreload 表示它传输完之后，不自己加载值，不启动DMA */
	
	// 因为我们在 DMA中断服务程序里面自己加载了下一次DMA传输要传输的数据，自己启动DMA，所以 bit22 应该设为 1
	// bit22 设为 0 ，会有杂音
	
	/* 使能中断,单个传输,硬件触发 */
	// playback_dma_info.period_size/2  :  一次DMA传输的长度
	// 为什么除以 2 ?
	// 因为 1<<20 将 dcon 寄存器的 bit20 置1 ，则DMA是一次传输2个字节( 见s3c2440芯片手册 P270 )
	dma_regs->dcon       = (1<<31)|(0<<30)|(1<<29)|(0<<28)|(0<<27)|(0<<24)|(1<<23)|(1<<22)|(1<<20)|(playback_dma_info.period_size/2);
}


// 启动 DMA 传输
static void s3c2440_dma_start(void)
{
	/* 启动DMA */
	dma_regs->dmasktrig  = (1<<1);
}


// 停止 DMA 传输
static void s3c2440_dma_stop(void)
{
	/* 停止DMA */
	dma_regs->dmasktrig  &= ~(1<<1);
}


// 应用程序把一段数据写入buffer，再把下一段数据写入buffer….
// 应用程序写入的位置称为 appl_ptr , 每写入的一段数据称为一个 period,每个 period里面有多个frame，一个frame里面有一个采样点的数据。
// 驱动程序从这个buffer里面取出一个period数据发给硬件，再取出一个period数据发给硬件….
// 驱动程序读出的位置称为hw_ptr。
// 这种方法可以解决声音断断续续的问题。因为数据是源源不断地从应用程序发给驱动，从驱动程序发给硬件。
// 使用这种方法就必须创建一个非常大的缓冲区，这个缓冲区是在驱动程序里面申请的。


// DMA2 中断处理函数
// 一个DMA传输一个period , DMA 传输完成，会产生一个 DMA 中断
// 在中断处理函数里更新状态信息，加载下一个period ，再次启动DMA传输
static irqreturn_t s3c2440_dma2_irq(int irq, void *devid)
{
    struct snd_pcm_substream *substream = devid;
	
    playback_dma_info.dma_ofs += playback_dma_info.period_size;     // 指向下一个 period

    if (playback_dma_info.dma_ofs >= playback_dma_info.buffer_size) // 如果偏移值超过了应用程序使用的 dma_buffer 大小
    {
		playback_dma_info.dma_ofs = 0;  // 回到 dma_buffer 起始地址
	}

    snd_pcm_period_elapsed(substream);  // 更新hw_ptr等信息,并且判断:如果 buffer 里没有数据了,则调用 s3c2440_dma_trigger 来停止DMA

    if (playback_dma_info.be_running)   // 如果还在进行 dma 传输
    {
		//如果还有数据
		
        load_dma_period();   // 加载下一个period ,作为下一次 DMA传输的数据
        s3c2440_dma_start(); // 再次启动DMA传输
    }

    return IRQ_HANDLED;
}


// 打开 dma 函数，设置属性，注册DMA传输中断
static int s3c2440_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
    int ret;

    /* 设置属性 */
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	snd_soc_set_runtime_hwparams(substream, &s3c2440_dma_hardware);

    /* 注册中断 */
	// IRQ_DMA2          :  通道 DMA2
	// s3c2440_dma2_irq  :  中断处理函数,一次DMA传输完成，会产生一个DMA中断
	// IRQF_DISABLED     :  当发生中断时，在中断处理过程中，这个中断保持屏蔽
	// substream         :  dev
    ret = request_irq(IRQ_DMA2, s3c2440_dma2_irq, IRQF_DISABLED, "myalsa for playback", substream);
    if (ret) // 如果注册中断出错
    {
        printk("request_irq error!\n");
        return -EIO;
    }

	return 0;
}


// 设置平台的 dma 参数函数
// params  :  由应用程序传进来的 params 来决定使用多大的 dma_buffer
static int s3c2440_dma_hw_params(struct snd_pcm_substream *substream,struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime  =   substream->runtime;
	unsigned long totbytes           =   params_buffer_bytes(params);  // 得到应用程序使用了 dma_buffer 多大空间
    
    /* 根据params设置DMA */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

    /* s3c2440_dma_new分配了很大的DMA BUFFER
     * params决定使用多大,params的值从用户空间传进来
     */
	runtime->dma_bytes            = totbytes;
    playback_dma_info.buffer_size = totbytes;                    // 记录应用程序使用了 dma_buffer 多大空间
    playback_dma_info.period_size = params_period_bytes(params); // 记录一个 period 的大小

    return 0;
}


// 准备 DMA 传输函数
// 复位各种状态信息，加载第一个 period
static int s3c2440_dma_prepare(struct snd_pcm_substream *substream)
{
    /* 准备DMA传输 */

    /* 复位各种状态信息 */
    playback_dma_info.dma_ofs = 0;     // 偏移值清零
    playback_dma_info.be_running = 0;  // 是否正在进行 DMA 传输清零 ，1  --- 正在进行DMA传输    0 --- 未进行DMA传输
    
    /* 加载第1个period */
    load_dma_period();

	return 0;
}


// 触发 DMA 传输函数
// cmd   :  根据 cmd 启动或者停止DMA传输
static int s3c2440_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

    /* 根据cmd启动或停止DMA传输 */


	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        /* 启动DMA传输 */
        playback_dma_info.be_running = 1;  // 设为 1 ，表正在进行 dma 传输
        s3c2440_dma_start();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        /* 停止DMA传输 */
        playback_dma_info.be_running = 0;  // 设为 0 ，表未进行 dma 传输
        s3c2440_dma_stop();
		break;

	default:
		ret = -EINVAL;
		break;
	}


	return ret;
}


/* 返回已经传输了多少个frame */
static snd_pcm_uframes_t s3c2440_dma_pointer(struct snd_pcm_substream *substream)
{
	return bytes_to_frames(substream->runtime, playback_dma_info.dma_ofs);
}


// 关闭 dma
static int s3c2440_dma_close(struct snd_pcm_substream *substream)
{
	// IRQ_DMA2  :  中断号
	// substream :  dev_id
    free_irq(IRQ_DMA2, substream);
	
    return 0;
}


// 构造 s3c2440_dma_ops 结构体
static struct snd_pcm_ops s3c2440_dma_ops = {
	.open		= s3c2440_dma_open,        // 打开 dma 函数，设置属性，注册DMA传输中断
	.close		= s3c2440_dma_close,       // 关闭 dma 函数，释放DMA中断
	.ioctl		= snd_pcm_lib_ioctl,       // ioctl 函数 snd_pcm_lib_ioctl 由内核提供，不需要我们自己写
	.hw_params	= s3c2440_dma_hw_params,   // 设置平台的 dma 参数函数
	.prepare    = s3c2440_dma_prepare,     // 准备 dma 传输
	.trigger	= s3c2440_dma_trigger,     // 触发 dma 传输,根据 cmd 启动或者停止DMA传输
	.pointer	= s3c2440_dma_pointer,     // 返回已经传输了多少个frame 
};


static u64 dma_mask = DMA_BIT_MASK(32);


// 在装载驱动程序，创建声卡时，调用这个函数，分配 dma_buffer,并将分配的 dma_buffer 信息告诉 ALSA 核心层
static int s3c2440_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	// snd_card 可以说是整个ALSA音频驱动最顶层的一个结构，整个声卡的软件逻辑结构开始于该结构，几乎所有与声
	// 音相关的逻辑设备都是在snd_card的管理之下，声卡驱动的第一个动作通常就是创建一个snd_card结构体。
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm  *pcm  = rtd->pcm;

	/* 下面两个用于保存分配的 dma_buffer 信息 */
	//  SNDRV_PCM_STREAM_PLAYBACK  对于播放
	struct snd_pcm_substream *substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct snd_dma_buffer    *buf       = &substream->dma_buffer;

	int ret = 0;

    /* 1. 分配DMA BUFFER */
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	
	//	SNDRV_PCM_STREAM_PLAYBACK  对于播放
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
	{
		// 分配 dma_buffer  :
		// playback_dma_info.virt_addr             :  存放分配得到的缓冲区的虚拟地址
		// s3c2440_dma_hardware.buffer_bytes_max   :  分配的缓冲区大小
		// playback_dma_info.phy_addr              :  分配得到的缓冲区的物理地址存放在这个变量里面
	    playback_dma_info.virt_addr = (unsigned int)dma_alloc_writecombine(pcm->card->dev, s3c2440_dma_hardware.buffer_bytes_max,&playback_dma_info.phy_addr, GFP_KERNEL);
	    
        if (!playback_dma_info.virt_addr) // 如果分配失败
        {
            return -ENOMEM;
        }

		// 如果程序执行到这里，说明刚才分配的缓冲区分配成功了，下面记录分配的缓冲区信息
		
        playback_dma_info.buf_max_size = s3c2440_dma_hardware.buffer_bytes_max;  // 记录分配得到的缓冲区大小

		// 将分配的 dma_buffer 信息保存 , 告诉 ALSA 的核心层
    	buf->dev.type      =  SNDRV_DMA_TYPE_DEV;
    	buf->dev.dev       =  pcm->card->dev;
    	buf->private_data  =  NULL;
        buf->area          =  playback_dma_info.virt_addr;
        buf->bytes         =  playback_dma_info.buf_max_size;
	}

	return ret;
}


// 在卸载驱动程序，销毁声卡时，调用这个函数，释放 s3c2440_dma_new 函数分配的 dma_buffer
static void s3c2440_dma_free(struct snd_pcm *pcm)
{
	// playback_dma_info.buf_max_size  :  前面分配的缓冲区大小
	// playback_dma_info.virt_addr     :  前面分配的缓冲区的虚拟地址
	// playback_dma_info.phy_addr      :  前面分配的缓冲区的物理地址
	dma_free_writecombine(pcm->card->dev, playback_dma_info.buf_max_size,(void *)playback_dma_info.virt_addr, playback_dma_info.phy_addr);
}


// 构造一个 snd_soc_platform_driver --- DMA 传数据
// 注意 :
// 这个 snd_soc_platform_driver ，没有名字，那么 machine 里面怎么知道调用这个 snd_soc_platform_driver ?
// 注册这个 snd_soc_platform_driver 的时候，继承使用 platform_device 里的名字。( snd_soc_register_platform(&pdev->dev, &s3c2440_dma_platform) )
static struct snd_soc_platform_driver s3c2440_dma_platform = {
	.ops		= &s3c2440_dma_ops,   // dma 操作函数结构体
	.pcm_new	= s3c2440_dma_new,    // 在装载驱动程序，创建声卡时，调用这个函数，分配 dma_buffer
	.pcm_free	= s3c2440_dma_free,   // 在卸载驱动程序，销毁声卡时，调用这个函数，释放 s3c2440_dma_new 函数分配的 dma_buffer
};


// 内核里面有同名的 platform_device ，platform_driver ,就会调用 probe 函数
static int s3c2440_dma_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &s3c2440_dma_platform);
}


static int s3c2440_dma_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
    return 0;
}


static void s3c2440_dma_release(struct device * dev)
{
}


// 下面运用了 platform_device , platform_driver 的小技巧 :
// 当内核里有同名的 platform_device , platform_driver ,就会调用 platform_driver 里的 probe 函数

// 构造一个 platform_device
static struct platform_device s3c2440_dma_dev = {
    .name         = "s3c2440-dma", // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .platform_name 一样
    .id       = -1,
    .dev = { 
    	.release = s3c2440_dma_release, 
	},
};


// 构造一个 platform_driver
struct platform_driver s3c2440_dma_drv = {
	.probe		= s3c2440_dma_probe,
	.remove		= s3c2440_dma_remove,
	.driver		= {
		.name	= "s3c2440-dma", // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .platform_name 一样
	}
};


// 入口函数
static int s3c2440_dma_init(void)
{
    dma_regs = ioremap(DMA2_BASE_ADDR, sizeof(struct s3c_dma_regs));  // 将 s3c dma2 寄存器映射到虚拟地址

    platform_device_register(&s3c2440_dma_dev);   // 注册平台设备
    platform_driver_register(&s3c2440_dma_drv);   // 注册平台驱动

    return 0;
}


// 出口函数
static void s3c2440_dma_exit(void)
{
    platform_device_unregister(&s3c2440_dma_dev); // 卸载平台设备
    platform_driver_unregister(&s3c2440_dma_drv); // 卸载平台驱动
    iounmap(dma_regs);                            // 取消映射
}

module_init(s3c2440_dma_init);
module_exit(s3c2440_dma_exit);

MODULE_LICENSE("GPL");

