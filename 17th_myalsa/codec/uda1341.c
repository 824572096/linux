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

#define UDA134X_RATES SNDRV_PCM_RATE_8000_48000
#define UDA134X_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE)


/* status control */
#define STAT0 (0x00)
#define STAT0_RST (1 << 6)
#define STAT0_SC_MASK (3 << 4)
#define STAT0_SC_512FS (0 << 4)
#define STAT0_SC_384FS (1 << 4)
#define STAT0_SC_256FS (2 << 4)
#define STAT0_IF_MASK (7 << 1)
#define STAT0_IF_I2S (0 << 1)
#define STAT0_IF_LSB16 (1 << 1)
#define STAT0_IF_LSB18 (2 << 1)
#define STAT0_IF_LSB20 (3 << 1)
#define STAT0_IF_MSB (4 << 1)
#define STAT0_IF_LSB16MSB (5 << 1)
#define STAT0_IF_LSB18MSB (6 << 1)
#define STAT0_IF_LSB20MSB (7 << 1)
#define STAT0_DC_FILTER (1 << 0)
#define STAT0_DC_NO_FILTER (0 << 0)

#define STAT1 (0x80)
#define STAT1_DAC_GAIN (1 << 6) /* gain of DAC */
#define STAT1_ADC_GAIN (1 << 5) /* gain of ADC */
#define STAT1_ADC_POL (1 << 4) /* polarity of ADC */
#define STAT1_DAC_POL (1 << 3) /* polarity of DAC */
#define STAT1_DBL_SPD (1 << 2) /* double speed playback */
#define STAT1_ADC_ON (1 << 1) /* ADC powered */
#define STAT1_DAC_ON (1 << 0) /* DAC powered */

/* data0 direct control */
#define DATA0 (0x00)
#define DATA0_VOLUME_MASK (0x3f)
#define DATA0_VOLUME(x) (x)

#define DATA1 (0x40)
#define DATA1_BASS(x) ((x) << 2)
#define DATA1_BASS_MASK (15 << 2)
#define DATA1_TREBLE(x) ((x))
#define DATA1_TREBLE_MASK (3)

#define DATA2 (0x80)
#define DATA2_PEAKAFTER (0x1 << 5)
#define DATA2_DEEMP_NONE (0x0 << 3)
#define DATA2_DEEMP_32KHz (0x1 << 3)
#define DATA2_DEEMP_44KHz (0x2 << 3)
#define DATA2_DEEMP_48KHz (0x3 << 3)
#define DATA2_MUTE (0x1 << 2)
#define DATA2_FILTER_FLAT (0x0 << 0)
#define DATA2_FILTER_MIN (0x1 << 0)
#define DATA2_FILTER_MAX (0x3 << 0)
/* data0 extend control */
#define EXTADDR(n) (0xc0 | (n))
#define EXTDATA(d) (0xe0 | (d))

#define EXT0 0
#define EXT0_CH1_GAIN(x) (x)
#define EXT1 1
#define EXT1_CH2_GAIN(x) (x)
#define EXT2 2
#define EXT2_MIC_GAIN_MASK (7 << 2)
#define EXT2_MIC_GAIN(x) ((x) << 2)
#define EXT2_MIXMODE_DOUBLEDIFF (0)
#define EXT2_MIXMODE_CH1 (1)
#define EXT2_MIXMODE_CH2 (2)
#define EXT2_MIXMODE_MIX (3)
#define EXT4 4
#define EXT4_AGC_ENABLE (1 << 4)
#define EXT4_INPUT_GAIN_MASK (3)
#define EXT4_INPUT_GAIN(x) ((x) & 3)
#define EXT5 5
#define EXT5_INPUT_GAIN(x) ((x) >> 2)
#define EXT6 6
#define EXT6_AGC_CONSTANT_MASK (7 << 2)
#define EXT6_AGC_CONSTANT(x) ((x) << 2)
#define EXT6_AGC_LEVEL_MASK (3)
#define EXT6_AGC_LEVEL(x) (x)


// 当我想要访问UDA1341，地址的前六位表示UDA1341，后两位表示访问哪一类寄存器
#define UDA1341_L3ADDR	    5
#define UDA1341_DATA0_ADDR	((UDA1341_L3ADDR << 2) | 0)  // DATA0  类
#define UDA1341_DATA1_ADDR	((UDA1341_L3ADDR << 2) | 1)  // DATA1  类
#define UDA1341_STATUS_ADDR	((UDA1341_L3ADDR << 2) | 2)  // STATUS 类


/* UDA1341 registers 编号 */
// UDA1341共有12个寄存器，分为三类 : DATA0 类  ,  DATA1类  ,  STATUS 类
#define UDA1341_DATA00 0
#define UDA1341_DATA01 1         // DATA0 类  -- 9个
#define UDA1341_DATA10 2
#define UDA1341_EA000  3
#define UDA1341_EA001  4
#define UDA1341_EA010  5
#define UDA1341_EA100  6
#define UDA1341_EA101  7
#define UDA1341_EA110  8

#define UDA1341_DATA1  9         // DATA1 类 --- 1个

#define UDA1341_STATUS0 10       // STATUS 类 --- 2个
#define UDA1341_STATUS1 11




#define UDA1341_REG_NUM 12

#define UDA1341_EXTADDR_PREFIX	0xC0  // 拓展地址   1100 0000
#define UDA1341_EXTDATA_PREFIX	0xE0  // 拓展数据   1110 0000


/* uda1341 所有寄存器的默认值 --- 参考UDA1341芯片手册 P14 Table5 Default settings */
// 这个数组了写得默认值顺序要和上面 UDA1341 registers 编号顺序一致
// char  一个寄存器的值 8 位
static const char uda1341_reg[UDA1341_REG_NUM] = {
    /* DATA0 类寄存器默认值 */
    0x00, 0x40, 0x80,
    
	/* Extended address registers 默认值 */
	0x04, 0x04, 0x04, 0x00, 0x00, 0x00,

    /* DATA1 类寄存器默认值 */
    0x00,
    
    /* STATUS 类寄存器默认值 */
    0x00, 0x83,
};


// UDA1341 三类寄存器地址
static const char uda1341_reg_addr[UDA1341_REG_NUM] = {
	/* DATA0 类地址 */
    UDA1341_DATA0_ADDR, UDA1341_DATA0_ADDR, UDA1341_DATA0_ADDR,

	/* Extended address registers 前三位数字 */
    0, 1, 2, 4, 5, 6,

	/* DATA1 类地址 */
    UDA1341_DATA1_ADDR,

	/* STATUS 类地址 */
    UDA1341_STATUS_ADDR, UDA1341_STATUS_ADDR
};


// UDA1341 访问三类寄存器，bit7 , bit6 类应设的值
static const char uda1341_data_bit[UDA1341_REG_NUM] = {
	/* DATA0 类高位应设的值 */
    0, (1<<6), (1<<7),

	/* Extended address registers 的访问方式与 DATA0 DATA1 STATUS 不同 */
	/* 所以下面全设为0 ，如何访问见 uda1341_write_reg 函数 */
    0, 0, 0, 0, 0, 0,

	/* DATA1 类高位应设的值 */
    0,

	/* STATUS 类高位应设的值 */	
    0, (1<<7),
};


static volatile unsigned int *gpbdat;
static volatile unsigned int *gpbcon;


/*
 * 获得音量信息,比如最小值最大值
 */
int uda1341_info_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
	uinfo->type              =  SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count             =  2;                           // 双声道
	uinfo->value.integer.min =  0;                           // 对于uda1341音量最小值 0
	uinfo->value.integer.max =  63;                          // 对于uda1341音量最大值 63
	
	return 0;
}


/*
 * 获得当前音量值
 */
int uda1341_get_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

    ucontrol->value.integer.value[1] = \
	ucontrol->value.integer.value[0] = 63 - snd_soc_read(codec, UDA1341_DATA00);
	return 0;
}


/*
 * 设置当前音量值
 */
int uda1341_put_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	// 将要设置的音量值写入寄存器

	val = 63 - ucontrol->value.integer.value[0];

    snd_soc_write(codec, UDA1341_DATA00, val);
    
	return 0;
}

static void uda1341_init_regs(struct snd_soc_codec *codec);


// 音量控制
// 每个声卡驱动程序的 snd_kcontrol_new 类型的变量各不相同，为什么应用程序都可以去调整它们的音量 ? 
// 肯定是对于常用的属性有约定俗成的名字 --- 见 linux3.4.2内核里的 sound\soc\codecs uda134x.c
// 应用程序根据它的名字找到 snd_kcontrol_new 项，调用里面的 put 函数
// 在 uda1341_soc_probe 将这个函数注册进去
static const struct snd_kcontrol_new uda1341_vol_control = 
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, 
    .name = "Master Playback Volume",     // 这个名字很重要，对于音量控制来说，就用这个名字
	.info = uda1341_info_vol,             // 获得一些信息，比如说你的音量范围是 0 ~ 100 还是 0 ~ 99 . 是双声道还是单声道
	.get  = uda1341_get_vol,              // 获得当前的音量值
	.put  = uda1341_put_vol,              // 设置音量
};



static int uda1341_soc_probe(struct snd_soc_codec *codec)
{
    int ret;
	
    uda1341_init_regs(codec); // 初始化uda1341 寄存器

	// uda1341_vol_control  --- 音量控制结构体
	// 1                    --- 1项
	ret = snd_soc_add_codec_controls(codec, &uda1341_vol_control, 1);
    return ret;
}


/* UDA1341的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */

/*
 * The codec has no support for reading its registers except for peak level...
 */
static inline unsigned int uda1341_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= UDA1341_REG_NUM)
		return -1;
	return cache[reg];
}


/*
 *    L3MODE - GPB2
 *    L3DATA - GPB3
 *    L3CLK  - GPB4
 */



// 设置模式引脚 GPB2 电平
static void set_mod(int val)
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



// 发送一个字节函数
static void sendbyte(unsigned int byte)
{
	int i;

	for (i = 0; i < 8; i++) {
		set_clk(0);
		udelay(1);
		set_dat(byte & 1);
		udelay(1);
		set_clk(1);
		udelay(1);
		byte >>= 1;
	}
}


// L3 写函数
static void l3_write(u8 addr, u8 data)
{
	set_clk(1);
	set_dat(1);
	set_mod(1);
	udelay(1);

	set_mod(0);
	udelay(1);
	sendbyte(addr);  // 发送地址
	udelay(1);

	set_mod(1);
	sendbyte(data);  // 发送数据

	set_clk(1);
	set_dat(1);
	set_mod(1);
}


/* UDA1341的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */


// uda1341 写寄存器函数
static int uda1341_write_reg(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 *cache = codec->reg_cache;


	if (reg >= UDA1341_REG_NUM)  // 如果寄存器数大于 UDA1341的寄存器数，则不支持
		return -1;
	
	/* 先保存将要写入 UDA1341 寄存器的值 */
	cache[reg] = value;

    /* 再写入硬件 */
    
    /* 对于EA,需要调用2次l3_write */
    if ((reg >= UDA1341_EA000) && (reg <= UDA1341_EA110))  // 如果要访问的是 EA 寄存器
    {
        l3_write(UDA1341_DATA0_ADDR, uda1341_reg_addr[reg] | UDA1341_EXTADDR_PREFIX);  // 访问拓展地址
        l3_write(UDA1341_DATA0_ADDR, value | UDA1341_EXTDATA_PREFIX);                  // 访问拓展数据
    }
    else   // 如果要访问的是 DAT0 或 DATA1 或 STATUS 寄存器
    {
    	// 访问具体哪个寄存器，单纯靠地址值还不够，还需要写入寄存器的值再或上某些位
        l3_write(uda1341_reg_addr[reg], value | uda1341_data_bit[reg]);
    }

    return 0;
}



/* UDA1341的寄存器不支持读操作
 * 要知道某个寄存器的当前值,
 * 只能在写入时保存起来
 */

// 构造一个 snd_soc_codec_driver ---- codec 控制接口(L3接口)
// 注意 :
// 这个 snd_soc_codec_driver ，没有名字，那么 machine 里面怎么知道调用这个 snd_soc_codec_driver ?
// 注册这个 snd_soc_codec_driver 的时候，继承使用 platform_device 里的名字。( snd_soc_register_codec(&pdev->dev,&soc_codec_dev_uda1341, &uda1341_dai, 1) )
static struct snd_soc_codec_driver soc_codec_dev_uda1341 = {
    .probe             = uda1341_soc_probe,
	.reg_cache_size    = sizeof(uda1341_reg),    // 保存写入 uda1341 寄存器的值
	.reg_word_size     = sizeof(u8),             // 用一个字节的数据来表示一个寄存器
	.reg_cache_default = uda1341_reg,
	.reg_cache_step    = 1,                      // 用一个字节的数据来表示一个寄存器
	.read              = uda1341_read_reg_cache,
	.write             = uda1341_write_reg,      // 写寄存器函数
};


// uda1341 初始化寄存器函数
static void uda1341_init_regs(struct snd_soc_codec *codec)
{

	/* GPB 4: L3CLOCK */
	/* GPB 3: L3DATA */
	/* GPB 2: L3MODE */
    *gpbcon &= ~((3<<4) | (3<<6) | (3<<8));
    *gpbcon |= ((1<<4) | (1<<6) | (1<<8));


    uda1341_write_reg(codec, UDA1341_STATUS0, 0x40 | STAT0_SC_384FS | STAT0_DC_FILTER); // reset uda1341
    uda1341_write_reg(codec, UDA1341_STATUS1, STAT1_ADC_ON | STAT1_DAC_ON);

    uda1341_write_reg(codec, UDA1341_DATA00, DATA0_VOLUME(0x0)); // maximum volume
    uda1341_write_reg(codec, UDA1341_DATA01, DATA1_BASS(0)| DATA1_TREBLE(0));
    uda1341_write_reg(codec, UDA1341_DATA10, 0);  // not mute 不要静音
}


// 编解码芯片参数设置函数 
static int uda1341_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
    /* 根据params的值,设置UDA1341的寄存器 
     * 比如时钟设置,格式
     */
    /* 为了简单, 在 uda1341_init_regs 里就设置好时钟、格式等参数 */
    return 0;
}

// 构造一个 snd_soc_dai_ops 结构体
static const struct snd_soc_dai_ops uda1341_dai_ops = {
	.hw_params	= uda1341_hw_params,  // 编解码芯片参数设置函数
};


// snd_soc_dai_driver  ---  codec DAI ( IIS 接口 )
static struct snd_soc_dai_driver uda1341_dai = {
	.name = "uda1341-iis",          // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_dai_name 一样
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",  // 播放
		.channels_min = 1,          // 最小通道是 1 . 说明 uda1341 支持单声道
		.channels_max = 2,          // 最大通道是 2 . 说明 uda1341 支持双声道
		.rates = UDA134X_RATES,     // 采样率
		.formats = UDA134X_FORMATS, // 格式
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",   // 捕获，指录音
		.channels_min = 1,          // 最小通道是 1 . 说明 uda1341 支持单声道
		.channels_max = 2,          // 最大通道是 2 . 说明 uda1341 支持双声道
		.rates = UDA134X_RATES,     // 采样率
		.formats = UDA134X_FORMATS, // 格式
	},
	/* pcm operations */
	.ops = &uda1341_dai_ops,        // 这个结构体里有各种操作
};


/* 通过注册平台设备、平台驱动来实现对snd_soc_register_codec的调用
 *
 */

static void uda1341_dev_release(struct device * dev)
{
}


// 内核里面有同名的 platform_device ，platform_driver ,就会调用 probe 函数
static int uda1341_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,&soc_codec_dev_uda1341, &uda1341_dai, 1);
}

static int uda1341_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);

    return 0;
}


// 此处应用 platform_device , platform_driver 小技巧 :
// 当内核里有同名的 platform_device ，platform_driver 时，就会调用 platform_driver 里的 probe 函数

// 构造一个 platform_device
static struct platform_device uda1341_dev = {
    .name         = "uda1341-codec",   // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_name 一样
    .id       = -1,
    .dev = { 
    	.release = uda1341_dev_release, 
	},
};


// 构造一个 platform_driver
struct platform_driver uda1341_drv = {
	.probe		= uda1341_probe,
	.remove		= uda1341_remove,
	.driver		= {
		.name	= "uda1341-codec",   // 这个名字要和 s3c2440_uda1341.c 里 snd_soc_dai_link 的 .codec_name 一样
	}
};


// 入口函数
static int uda1341_init(void)
{
    gpbcon = ioremap(0x56000010, 4);        // 映射到虚拟地址
    gpbdat = ioremap(0x56000014, 4);        // 映射到虚拟地址
    
    platform_device_register(&uda1341_dev);  // 注册平台设备
    platform_driver_register(&uda1341_drv);  // 注册平台驱动

    return 0;
}


// 出口函数
static void uda1341_exit(void)
{
    platform_device_unregister(&uda1341_dev);
    platform_driver_unregister(&uda1341_drv);

    iounmap(gpbcon);
    iounmap(gpbdat);
}

module_init(uda1341_init);
module_exit(uda1341_exit);

MODULE_LICENSE("GPL");
