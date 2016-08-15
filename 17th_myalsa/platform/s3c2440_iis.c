#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <mach/regs-gpio.h>
#include <mach/dma.h>

/* 参考sound\soc\samsung\s3c24xx-i2s.c
 */


// 构造一个 s3c2440 iis 寄存器结构体类型
struct s3c2440_iis_regs {
    unsigned int iiscon ; 
    unsigned int iismod ; 
    unsigned int iispsr ; 
    unsigned int iisfcon; 
    unsigned int iisfifo; 
};

static volatile unsigned int *gpecon; // 用于指向 GPIOE 控制寄存器映射到虚拟地址

static volatile struct s3c2440_iis_regs *iis_regs; // 用于指向 iis_regs 寄存器映射到虚拟地址

#define ABS(a, b) ((a>b)?(a-b):(b-a))


// 启动 IIS
static void s3c2440_iis_start(void)
{
    iis_regs->iiscon |= (1);
}


// 关闭 IIS
static void s3c2440_iis_stop(void)
{
    iis_regs->iiscon &= ~(1);
}


// 平台 iis 控制器寄存器参数设置函数
static int s3c2440_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
    /* 根据params设置IIS控制器 */

    int tmp_fs;        // 用于保存I2S控制器采样频率
    int i;
    int min = 0xffff;
    int pre = 0;       // 用于保存一个让I2S控制器采样频率与WAV文件里的记录的采样频率最接近的值
    unsigned int fs;   // 用来保存采样率
	
    struct clk *clk = clk_get(NULL, "pclk");

    /* 配置GPIO用于IIS */
    *gpecon &= ~((3<<0) | (3<<2) | (3<<4) | (3<<6) | (3<<8));
    *gpecon |= ((2<<0) | (2<<2) | (2<<4) | (2<<6) | (2<<8));
    
    
    /* bit[9] : Master clock select, 0-PCLK
     * bit[8] : 0 = Master mode
     * bit[7:6] : 10 = Transmit mode
     * bit[4] : 0-IIS compatible format
     * bit[2] : 384fs, 确定了MASTER CLOCK之后, fs = MASTER CLOCK/384
     * bit[1:0] : Serial bit clock frequency select, 32fs
     */

	// params_format(params) 得到每个采样值用多少位表示
	
    if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE) // 如果每个采样值用16位表示
        iis_regs->iismod = (2<<6) | (0<<4) | (1<<3) | (1<<2) | (1);
    else if (params_format(params) == SNDRV_PCM_FORMAT_S8) // 如果每个采样值用8位表示
        iis_regs->iismod = (2<<6) | (0<<4) | (0<<3) | (1<<2) | (1);
    else  // 其他的不支持
        return -EINVAL;

    /* 时 钟  :  Master clock = PCLK/(n+1)
     * 采样率 :  fs = Master clock / 384  =  PCLK / (n+1) / 384 = 50000000 / (n+1) / 384
     */
 
	// IISPSR               |    Bit          Description                                              | Initial State
	// ---------------------------------------------------------------------------------------------------------------
    // Prescaler control A  |   [9:5]         Data value: 0 ~ 31                                       |    00000
    //                      |                 Note: Prescaler A makes the master clock that            |
    //                      |                 is used the internal block and division factor is N+1.   |
	// ------------------------------------------------------------------------------------------------------------------
	// Prescaler control B  |   [4:0]         Data value: 0 ~ 31                                       |    00000
	//                      |                 Note: Prescaler B makes the master clock that            |
    //                      |                 is used the external block and division factor is N+1.   |
	


    /* Master clock = PCLK/(n+1)
     * fs = Master clock / 384
     * fs = PCLK / (n+1) / 384
     */
    fs = params_rate(params);   // 得到采样率
	
    for (i = 0; i <= 31; i++)   // 在 0 ~ 31 里面找一个使I2S设置的采样频率与WAV文件里的记录的采样频率最接近的值
    {
        tmp_fs = clk_get_rate(clk)/384/(i+1);
		
        if (ABS(tmp_fs, fs) < min)
        {
            min = ABS(tmp_fs, fs);
            pre = i;
        }
    }
    iis_regs->iispsr = (pre << 5) | (pre); // 设置分频参数

    /*
     * bit15 : Transmit FIFO access mode select, 1-DMA
     * bit13 : Transmit FIFO, 1-enable
     */
    iis_regs->iisfcon = (1<<15) | (1<<13);
    
    /*
     * bit[5] : Transmit DMA service request, 1-enable
     * bit[1] : IIS prescaler, 1-enable
     */
    iis_regs->iiscon = (1<<5) | (1<<1) ;

    clk_put(clk);
    
    return 0;
}


// 根据 cmd 来决定 启动/停止 IIS
static int s3c2440_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
       		s3c2440_iis_start();  // 启动 IIS
			break;
			
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			
		default:
        	s3c2440_iis_stop();  // 停止 IIS
			ret = -EINVAL;
			break;
	}

exit_err:
	return ret;
}


// 构造一个 snd_soc_dai_ops 结构体
static const struct snd_soc_dai_ops s3c2440_i2s_dai_ops = {
	.hw_params	= s3c2440_i2s_hw_params,   // 平台 iis 控制器寄存器参数设置函数
	.trigger	= s3c2440_i2s_trigger,     // 根据 cmd 来决定 启动/停止 IIS
};

#define S3C24XX_I2S_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)


// 构造一个 snd_soc_dai_driver  ---  CPU DAI 设置接口 ( CPU 的 IIS 设置接口 )
// 注意 :
// 这个 snd_soc_dai_driver ，没有名字，那么 machine 里面怎么知道调用这个 snd_soc_dai_driver ?
// 注册这个 snd_soc_dai_driver 的时候，继承使用 platform_device 里的名字。( snd_soc_register_dai(&pdev->dev, &s3c2440_i2s_dai) )
static struct snd_soc_dai_driver s3c2440_i2s_dai = {
	.playback = {
		.channels_min = 2,  // 最小通道是 2
		.channels_max = 2,  // 最大通道是 2 . 最小通道和最大通道都是 2 ，说明 2440 只支持双声道
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,  // 最小通道是 2
		.channels_max = 2,  // 最大通道是 2 . 最小通道和最大通道都是 2 ，说明 2440 只支持双声道
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &s3c2440_i2s_dai_ops,  // 数据传输相关的操作
};


// 内核里面有同名的 platform_device ，platform_driver ,就会调用 probe 函数
static int s3c2440_iis_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &s3c2440_i2s_dai);   // 注册 s3c2440_i2s_dai
}


static int s3c2440_iis_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
    return 0;
}

static void s3c2440_iis_release(struct device * dev)
{
}


// 此处应用 platform_device , platform_driver 小技巧 :
// 当内核里有同名的 platform_device ，platform_driver 时，就会调用 platform_driver 里的 probe 函数

// 构造一个 platform_device
static struct platform_device s3c2440_iis_dev = {
    .name         = "s3c2440-iis", // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .cpu_dai_name 一样
    .id       = -1,
    .dev = { 
    	.release = s3c2440_iis_release, 
	},
};

// 构造一个 platform_driver
struct platform_driver s3c2440_iis_drv = {
	.probe		= s3c2440_iis_probe,
	.remove		= s3c2440_iis_remove,
	.driver		= {
		.name	= "s3c2440-iis",   // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .cpu_dai_name一样
	}
};


static int s3c2440_iis_init(void)
{
    struct clk *clk;  // 用于存放 IIS 控制器时钟
	
    gpecon   = ioremap(0x56000040, 4);                               // 映射到虚拟地址
    iis_regs = ioremap(0x55000000, sizeof(struct s3c2440_iis_regs)); // 映射到虚拟地址

	// 打开 IIS 控制器时钟
	
	clk = clk_get(NULL, "iis"); // 获得 IIS 控制器时钟
    clk_enable(clk);
    clk_put(clk);

    platform_device_register(&s3c2440_iis_dev);
    platform_driver_register(&s3c2440_iis_drv);
    return 0;
}

static void s3c2440_iis_exit(void)
{
    struct clk *clk; // 用于存放 IIS 控制器时钟

    platform_device_unregister(&s3c2440_iis_dev);  // 注册平台设备
    platform_driver_unregister(&s3c2440_iis_drv);  // 注册平台驱动
    iounmap(gpecon);
    iounmap(iis_regs);

	// 关闭 IIS 控制器时钟
    
	clk = clk_get(NULL, "iis"); // 获得 IIS 控制器时钟
    clk_disable(clk);
    clk_put(clk);
}

module_init(s3c2440_iis_init);
module_exit(s3c2440_iis_exit);

MODULE_LICENSE("GPL");

