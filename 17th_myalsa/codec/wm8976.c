#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <asm/io.h>


/* 参考 sound\soc\codecs\uda134x.c
 */

/* 1. 构造一个 snd_soc_dai_driver    ----  codec DAI ( IIS 接口 )
 * 2. 构造一个 snd_soc_codec_driver  ----  codec 控制接口( L3接口 )
 * 3. 注册它们
 */



/* 采样率从 8K 支持到 48K --- 详见WM8976芯片手册 P1 */
#define WM8976_RATES   SNDRV_PCM_RATE_8000_48000


#define WM8976_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	        SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


#define WM8976_REG_NUM 58  // wm8976 芯片一共有 58 个寄存器


/* wm8976 所有寄存器的默认值 */
// unsigned short 7 位地址值，9位数据
static const unsigned short wm8976_reg[WM8976_REG_NUM] = {
};

static volatile unsigned int *gpbdat;
static volatile unsigned int *gpbcon;


/*
 * 获得音量信息,比如最小值最大值
 */
int wm8976_info_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;                           // 双声道
	uinfo->value.integer.min = 0;               // 对于wm8976音量最小值 0
	uinfo->value.integer.max = 63;              // 对于wm8976音量最大值 63
	return 0;
}


/*
 * 获得当前音量值
 */
int wm8976_get_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	// 音量寄存器 --- 详见 WM8976 P100  . 全 0 的时候表示最小 , 全 1 的时候表示最大
	// 我们要注意在获得音量函数 wm8976_get_vol 里面，要注意只需要保留低6位，
	// 因为wm8976芯片那个寄存器只有低6位表示当前音量值
	ucontrol->value.integer.value[0] = snd_soc_read(codec, 52) & 0x3f; // 保留低 6 位
	ucontrol->value.integer.value[1] = snd_soc_read(codec, 53) & 0x3f; // 保留低 6 位
	
	return 0;
}


/*
 * 设置当前音量值
 */
int wm8976_put_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	// 将要设置的音量值写入寄存器
	// 查看wm8976芯片手册 P100 ，看看 1<<8 的含义：
	// 只有第 52 ，53 号寄存器的第8位被置 1 了，你设置的音量值才会被更新
	// 那么我们要注意在获得音量函数 wm8976_get_vol 里面，要注意只需要保留低6位，
	// 因为wm8976芯片那个寄存器只有低6位表示当前音量值
	val = ucontrol->value.integer.value[0];
    snd_soc_write(codec, 52, (1<<8)|val);

	val = ucontrol->value.integer.value[1];
    snd_soc_write(codec, 53, (1<<8)|val);
    
	return 0;
}

static void wm8976_init_regs(struct snd_soc_codec *codec);


// 音量控制
// 每个声卡驱动程序的 snd_kcontrol_new 类型的变量各不相同，为什么应用程序都可以去调整它们的音量 ?
// 肯定是对于常用的属性有约定俗成的名字
// 应用程序根据它的名字找到 snd_kcontrol_new 项，调用里面的 put 函数
// 在 uda1341_soc_probe 将这个函数注册进去
static const struct snd_kcontrol_new wm8976_vol_control = 
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, 
    .name = "Master Playback Volume",      // 这个名字很重要，对于音量控制来说，就用这个名字
	.info = wm8976_info_vol,               // 获得一些信息，比如说你的音量范围是 0 ~ 100 还是 0 ~ 99
	.get  = wm8976_get_vol,                // 获得当前的音量值
	.put  = wm8976_put_vol,                // 设置音量
};


// 在这个 probe 函数里初始化wm8976 寄存器 ，添加音量控制等
static int wm8976_soc_probe(struct snd_soc_codec *codec)
{
    int ret;

    wm8976_init_regs(codec); // 初始化 wm8976 寄存器
    
	// wm8976_vol_control  --- 音量控制结构体
	// 1                   --- 1项
	ret = snd_soc_add_codec_controls(codec, &wm8976_vol_control, 1);
    return ret;
}

/* wm8976的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */

/*
 * The codec has no support for reading its registers except for peak level...
 */
static inline unsigned int wm8976_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= WM8976_REG_NUM)
		return -1;
	return cache[reg];
}

/*
 *    L3MODE - GPB2
 *    L3DATA - GPB3
 *    L3CLK  - GPB4
 */



// 设置模式引脚 GPB2 电平
static void set_csb(int val)
{
    if (val)
    {
        *gpbdat |= (1<<2);
    }
    else
    {
        *gpbdat &= ~(1<<2);
    }
}


// 设置数据引脚 GPB3 电平
static void set_dat(int val)
{
    if (val)
    {
        *gpbdat |= (1<<3);
    }
    else
    {
        *gpbdat &= ~(1<<3);
    }
}


// 设置时钟引脚 GPB4 电平
static void set_clk(int val)
{
    if (val)
    {
        *gpbdat |= (1<<4);
    }
    else
    {
        *gpbdat &= ~(1<<4);
    }
}


/* UDA1341的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */

// wm8976 写寄存器函数,发出7位的地址值，9位的数据，共16位
static int wm8976_write_reg(struct snd_soc_codec *codec, unsigned int reg,unsigned int value)
{
	u8 *cache = codec->reg_cache;
	int i;

	// reg << 9       :  保留低 7 位，原来的低 7 位左移到高 7 位上
	// value & 0x1ff  :  保留低 9 位
	unsigned short val = (reg << 9) | (value & 0x1ff);  // 合并为要发出的16位 val 值

	if (reg >= WM8976_REG_NUM)  // 如果寄存器数大于 wm8976 的寄存器数，则不支持
		return -1;
	
	/* 先保存将要写入 wm8976 寄存器的值 */
	cache[reg] = value;

    /* 再写入硬件 */
    set_csb(1);
    set_dat(1);
    set_clk(1);

	// 一位一位将数据发送出去,先发送最高位
	for (i = 0; i < 16; i++)
	{
		if (val & (1<<15))
		{
            set_clk(0);
            set_dat(1);
			udelay(1);
            set_clk(1);
		}
		else
		{
            set_clk(0);
            set_dat(0);
			udelay(1);
            set_clk(1);
		}

		val = val << 1;
	}

    set_csb(0);
	udelay(1);
    set_csb(1);
    set_dat(1);
    set_clk(1);
    

    return 0;
}

/* UDA1341的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */

// 构造一个 snd_soc_codec_driver ---- codec 控制接口(L3接口)
// 注意 :
// 这个 snd_soc_codec_driver ，没有名字，那么 machine 里面怎么知道调用这个 snd_soc_codec_driver ?
// 注册这个 snd_soc_codec_driver 的时候，继承使用 platform_device 里的名字。( snd_soc_register_codec(&pdev->dev,&soc_codec_dev_wm8976, &wm8976_dai, 1); )
static struct snd_soc_codec_driver soc_codec_dev_wm8976 = {
    .probe = wm8976_soc_probe,             // 在这个 probe 函数里初始化wm8976 寄存器 ，添加音量控制等
	.reg_cache_size = sizeof(wm8976_reg),  // 保存写入 wm8976 寄存器的值
	.reg_word_size = sizeof(u16),          // wm8976_reg 数组里，每一项占据16位
	.reg_cache_default = wm8976_reg,
	.reg_cache_step = 2,                   // 用两个字节表示一个寄存器的值
	.read  = wm8976_read_reg_cache,        // 读寄存器函数
	.write = wm8976_write_reg,             // 写寄存器函数
};


// wm8976 初始化寄存器函数
static void wm8976_init_regs(struct snd_soc_codec *codec)
{

	/* GPB 4: L3CLOCK */
	/* GPB 3: L3DATA */
	/* GPB 2: L3MODE */
    *gpbcon &= ~((3<<4) | (3<<6) | (3<<8));
    *gpbcon |= ((1<<4) | (1<<6) | (1<<8));

	/* software reset */
	wm8976_write_reg(codec, 0, 0);

	/* OUT2的左/右声道打开
	 * 左/右通道输出混音打开
	 * 左/右DAC打开
	 */
	wm8976_write_reg(codec, 0x3, 0x6f);
	
	wm8976_write_reg(codec, 0x1, 0x1f);  // biasen,BUFIOEN.VMIDSEL=11b  
	wm8976_write_reg(codec, 0x2, 0x185); // ROUT1EN LOUT1EN, inpu PGA enable ,ADC enable

	wm8976_write_reg(codec, 0x6, 0x0);   // SYSCLK=MCLK  
	wm8976_write_reg(codec, 0x4, 0x10);  // 16bit 		
	wm8976_write_reg(codec, 0x2B,0x10);  // BTL OUTPUT  
	wm8976_write_reg(codec, 0x9, 0x50);  // Jack detect enable  
	wm8976_write_reg(codec, 0xD, 0x21);  // Jack detect  
	wm8976_write_reg(codec, 0x7, 0x01);  // Jack detect 

}


// 编解码芯片参数设置函数 
static int wm8976_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
    /* 根据params的值,设置wm8976的寄存器 
     * 比如时钟设置,格式
     */
    /* 为了简单, 在 wm8976_init_regs 里就设置好时钟、格式等参数 */
    return 0;
}


// 构造一个 snd_soc_dai_ops 结构体
static const struct snd_soc_dai_ops wm8976_dai_ops = {
	.hw_params	= wm8976_hw_params,
};

// snd_soc_dai_driver  ---  codec DAI ( IIS 接口 )
static struct snd_soc_dai_driver wm8976_dai = {
	.name = "wm8976-iis",         // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_dai_name 一样
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",  // 播放
		.channels_min = 1,          // 最小通道是 1 . 说明 wm8976 支持单声道播放
		.channels_max = 2,          // 最大通道是 2 . 说明 wm8976 支持双声道播放
		.rates = WM8976_RATES,      // 采样率
		.formats = WM8976_FORMATS,  // 格式
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",   // 捕获，指录音
		.channels_min = 1,          // 最小通道是 1 . 说明 wm8976 支持单声道录音
		.channels_max = 2,          // 最大通道是 2 . 说明 wm8976 支持双声道录音
		.rates = WM8976_RATES,      // 采样率
		.formats = WM8976_FORMATS,  // 格式
	},
	/* pcm operations */
	.ops = &wm8976_dai_ops,        // 这个结构体里有各种操作
};


/* 通过注册平台设备、平台驱动来实现对snd_soc_register_codec的调用
 *
 */

static void wm8976_dev_release(struct device * dev)
{
}


// 内核里面有同名的 platform_device ，platform_driver ,就会调用 probe 函数
// 注册 codec 控制接口(L3接口)  和  codec dai 接口
// soc_codec_dev_wm8976 的名字继承使用 platform_device ， platform_driver 的名字
static int wm8976_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,&soc_codec_dev_wm8976, &wm8976_dai, 1);
}

static int wm8976_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);
    return 0;
}

// 此处应用 platform_device , platform_driver 小技巧 :
// 当内核里有同名的 platform_device ，platform_driver 时，就会调用 platform_driver 里的 probe 函数

// 构造一个 platform_device
static struct platform_device wm8976_dev = {
    .name         = "wm8976-codec",   // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_name 一样
    .id       = -1,
    .dev = { 
    	.release = wm8976_dev_release, 
	},
};


// 构造一个 platform_driver
struct platform_driver wm8976_drv = {
	.probe		= wm8976_probe,     // 注册 codec 控制接口(L3接口)  和  codec dai 接口
	.remove		= wm8976_remove,
	.driver		= {
		.name	= "wm8976-codec",   // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_name 一样
	}
};


// 入口函数
static int wm8976_init(void)
{
    gpbcon = ioremap(0x56000010, 4);        // 映射到虚拟地址
    gpbdat = ioremap(0x56000014, 4);        // 映射到虚拟地址
    
    platform_device_register(&wm8976_dev);  // 注册平台设备
    platform_driver_register(&wm8976_drv);  // 注册平台驱动
    return 0;
}


// 出口函数
static void wm8976_exit(void)
{
    platform_device_unregister(&wm8976_dev);
    platform_driver_unregister(&wm8976_drv);

    iounmap(gpbcon);
    iounmap(gpbdat);
}

module_init(wm8976_init);
module_exit(wm8976_exit);

MODULE_LICENSE("GPL");
