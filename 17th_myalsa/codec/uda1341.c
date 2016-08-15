#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <asm/io.h>


/* �ο� sound\soc\codecs\uda134x.c
 */

/* 1. ����һ�� snd_soc_dai_driver    ----  codec DAI ( IIS �ӿ� )
 * 2. ����һ�� snd_soc_codec_driver  ----  codec ���ƽӿ�( L3�ӿ� )
 * 3. ע������
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


// ������Ҫ����UDA1341����ַ��ǰ��λ��ʾUDA1341������λ��ʾ������һ��Ĵ���
#define UDA1341_L3ADDR	    5
#define UDA1341_DATA0_ADDR	((UDA1341_L3ADDR << 2) | 0)  // DATA0  ��
#define UDA1341_DATA1_ADDR	((UDA1341_L3ADDR << 2) | 1)  // DATA1  ��
#define UDA1341_STATUS_ADDR	((UDA1341_L3ADDR << 2) | 2)  // STATUS ��


/* UDA1341 registers ��� */
// UDA1341����12���Ĵ�������Ϊ���� : DATA0 ��  ,  DATA1��  ,  STATUS ��
#define UDA1341_DATA00 0
#define UDA1341_DATA01 1         // DATA0 ��  -- 9��
#define UDA1341_DATA10 2
#define UDA1341_EA000  3
#define UDA1341_EA001  4
#define UDA1341_EA010  5
#define UDA1341_EA100  6
#define UDA1341_EA101  7
#define UDA1341_EA110  8

#define UDA1341_DATA1  9         // DATA1 �� --- 1��

#define UDA1341_STATUS0 10       // STATUS �� --- 2��
#define UDA1341_STATUS1 11




#define UDA1341_REG_NUM 12

#define UDA1341_EXTADDR_PREFIX	0xC0  // ��չ��ַ   1100 0000
#define UDA1341_EXTDATA_PREFIX	0xE0  // ��չ����   1110 0000


/* uda1341 ���мĴ�����Ĭ��ֵ --- �ο�UDA1341оƬ�ֲ� P14 Table5 Default settings */
// ���������д��Ĭ��ֵ˳��Ҫ������ UDA1341 registers ���˳��һ��
// char  һ���Ĵ�����ֵ 8 λ
static const char uda1341_reg[UDA1341_REG_NUM] = {
    /* DATA0 ��Ĵ���Ĭ��ֵ */
    0x00, 0x40, 0x80,
    
	/* Extended address registers Ĭ��ֵ */
	0x04, 0x04, 0x04, 0x00, 0x00, 0x00,

    /* DATA1 ��Ĵ���Ĭ��ֵ */
    0x00,
    
    /* STATUS ��Ĵ���Ĭ��ֵ */
    0x00, 0x83,
};


// UDA1341 ����Ĵ�����ַ
static const char uda1341_reg_addr[UDA1341_REG_NUM] = {
	/* DATA0 ���ַ */
    UDA1341_DATA0_ADDR, UDA1341_DATA0_ADDR, UDA1341_DATA0_ADDR,

	/* Extended address registers ǰ��λ���� */
    0, 1, 2, 4, 5, 6,

	/* DATA1 ���ַ */
    UDA1341_DATA1_ADDR,

	/* STATUS ���ַ */
    UDA1341_STATUS_ADDR, UDA1341_STATUS_ADDR
};


// UDA1341 ��������Ĵ�����bit7 , bit6 ��Ӧ���ֵ
static const char uda1341_data_bit[UDA1341_REG_NUM] = {
	/* DATA0 ���λӦ���ֵ */
    0, (1<<6), (1<<7),

	/* Extended address registers �ķ��ʷ�ʽ�� DATA0 DATA1 STATUS ��ͬ */
	/* ��������ȫ��Ϊ0 ����η��ʼ� uda1341_write_reg ���� */
    0, 0, 0, 0, 0, 0,

	/* DATA1 ���λӦ���ֵ */
    0,

	/* STATUS ���λӦ���ֵ */	
    0, (1<<7),
};


static volatile unsigned int *gpbdat;
static volatile unsigned int *gpbcon;


/*
 * ���������Ϣ,������Сֵ���ֵ
 */
int uda1341_info_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
	uinfo->type              =  SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count             =  2;                           // ˫����
	uinfo->value.integer.min =  0;                           // ����uda1341������Сֵ 0
	uinfo->value.integer.max =  63;                          // ����uda1341�������ֵ 63
	
	return 0;
}


/*
 * ��õ�ǰ����ֵ
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
 * ���õ�ǰ����ֵ
 */
int uda1341_put_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	// ��Ҫ���õ�����ֵд��Ĵ���

	val = 63 - ucontrol->value.integer.value[0];

    snd_soc_write(codec, UDA1341_DATA00, val);
    
	return 0;
}

static void uda1341_init_regs(struct snd_soc_codec *codec);


// ��������
// ÿ��������������� snd_kcontrol_new ���͵ı���������ͬ��ΪʲôӦ�ó��򶼿���ȥ�������ǵ����� ? 
// �϶��Ƕ��ڳ��õ�������Լ���׳ɵ����� --- �� linux3.4.2�ں���� sound\soc\codecs uda134x.c
// Ӧ�ó���������������ҵ� snd_kcontrol_new ���������� put ����
// �� uda1341_soc_probe ���������ע���ȥ
static const struct snd_kcontrol_new uda1341_vol_control = 
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, 
    .name = "Master Playback Volume",     // ������ֺ���Ҫ����������������˵�������������
	.info = uda1341_info_vol,             // ���һЩ��Ϣ������˵���������Χ�� 0 ~ 100 ���� 0 ~ 99 . ��˫�������ǵ�����
	.get  = uda1341_get_vol,              // ��õ�ǰ������ֵ
	.put  = uda1341_put_vol,              // ��������
};



static int uda1341_soc_probe(struct snd_soc_codec *codec)
{
    int ret;
	
    uda1341_init_regs(codec); // ��ʼ��uda1341 �Ĵ���

	// uda1341_vol_control  --- �������ƽṹ��
	// 1                    --- 1��
	ret = snd_soc_add_codec_controls(codec, &uda1341_vol_control, 1);
    return ret;
}


/* UDA1341�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
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



// ����ģʽ���� GPB2 ��ƽ
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


// ������������ GPB3 ��ƽ
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


// ����ʱ������ GPB4 ��ƽ
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



// ����һ���ֽں���
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


// L3 д����
static void l3_write(u8 addr, u8 data)
{
	set_clk(1);
	set_dat(1);
	set_mod(1);
	udelay(1);

	set_mod(0);
	udelay(1);
	sendbyte(addr);  // ���͵�ַ
	udelay(1);

	set_mod(1);
	sendbyte(data);  // ��������

	set_clk(1);
	set_dat(1);
	set_mod(1);
}


/* UDA1341�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
 */


// uda1341 д�Ĵ�������
static int uda1341_write_reg(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 *cache = codec->reg_cache;


	if (reg >= UDA1341_REG_NUM)  // ����Ĵ��������� UDA1341�ļĴ���������֧��
		return -1;
	
	/* �ȱ��潫Ҫд�� UDA1341 �Ĵ�����ֵ */
	cache[reg] = value;

    /* ��д��Ӳ�� */
    
    /* ����EA,��Ҫ����2��l3_write */
    if ((reg >= UDA1341_EA000) && (reg <= UDA1341_EA110))  // ���Ҫ���ʵ��� EA �Ĵ���
    {
        l3_write(UDA1341_DATA0_ADDR, uda1341_reg_addr[reg] | UDA1341_EXTADDR_PREFIX);  // ������չ��ַ
        l3_write(UDA1341_DATA0_ADDR, value | UDA1341_EXTDATA_PREFIX);                  // ������չ����
    }
    else   // ���Ҫ���ʵ��� DAT0 �� DATA1 �� STATUS �Ĵ���
    {
    	// ���ʾ����ĸ��Ĵ�������������ֵַ������������Ҫд��Ĵ�����ֵ�ٻ���ĳЩλ
        l3_write(uda1341_reg_addr[reg], value | uda1341_data_bit[reg]);
    }

    return 0;
}



/* UDA1341�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
 */

// ����һ�� snd_soc_codec_driver ---- codec ���ƽӿ�(L3�ӿ�)
// ע�� :
// ��� snd_soc_codec_driver ��û�����֣���ô machine ������ô֪��������� snd_soc_codec_driver ?
// ע����� snd_soc_codec_driver ��ʱ�򣬼̳�ʹ�� platform_device ������֡�( snd_soc_register_codec(&pdev->dev,&soc_codec_dev_uda1341, &uda1341_dai, 1) )
static struct snd_soc_codec_driver soc_codec_dev_uda1341 = {
    .probe             = uda1341_soc_probe,
	.reg_cache_size    = sizeof(uda1341_reg),    // ����д�� uda1341 �Ĵ�����ֵ
	.reg_word_size     = sizeof(u8),             // ��һ���ֽڵ���������ʾһ���Ĵ���
	.reg_cache_default = uda1341_reg,
	.reg_cache_step    = 1,                      // ��һ���ֽڵ���������ʾһ���Ĵ���
	.read              = uda1341_read_reg_cache,
	.write             = uda1341_write_reg,      // д�Ĵ�������
};


// uda1341 ��ʼ���Ĵ�������
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
    uda1341_write_reg(codec, UDA1341_DATA10, 0);  // not mute ��Ҫ����
}


// �����оƬ�������ú��� 
static int uda1341_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
    /* ����params��ֵ,����UDA1341�ļĴ��� 
     * ����ʱ������,��ʽ
     */
    /* Ϊ�˼�, �� uda1341_init_regs ������ú�ʱ�ӡ���ʽ�Ȳ��� */
    return 0;
}

// ����һ�� snd_soc_dai_ops �ṹ��
static const struct snd_soc_dai_ops uda1341_dai_ops = {
	.hw_params	= uda1341_hw_params,  // �����оƬ�������ú���
};


// snd_soc_dai_driver  ---  codec DAI ( IIS �ӿ� )
static struct snd_soc_dai_driver uda1341_dai = {
	.name = "uda1341-iis",          // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_dai_name һ��
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",  // ����
		.channels_min = 1,          // ��Сͨ���� 1 . ˵�� uda1341 ֧�ֵ�����
		.channels_max = 2,          // ���ͨ���� 2 . ˵�� uda1341 ֧��˫����
		.rates = UDA134X_RATES,     // ������
		.formats = UDA134X_FORMATS, // ��ʽ
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",   // ����ָ¼��
		.channels_min = 1,          // ��Сͨ���� 1 . ˵�� uda1341 ֧�ֵ�����
		.channels_max = 2,          // ���ͨ���� 2 . ˵�� uda1341 ֧��˫����
		.rates = UDA134X_RATES,     // ������
		.formats = UDA134X_FORMATS, // ��ʽ
	},
	/* pcm operations */
	.ops = &uda1341_dai_ops,        // ����ṹ�����и��ֲ���
};


/* ͨ��ע��ƽ̨�豸��ƽ̨������ʵ�ֶ�snd_soc_register_codec�ĵ���
 *
 */

static void uda1341_dev_release(struct device * dev)
{
}


// �ں�������ͬ���� platform_device ��platform_driver ,�ͻ���� probe ����
static int uda1341_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,&soc_codec_dev_uda1341, &uda1341_dai, 1);
}

static int uda1341_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);

    return 0;
}


// �˴�Ӧ�� platform_device , platform_driver С���� :
// ���ں�����ͬ���� platform_device ��platform_driver ʱ���ͻ���� platform_driver ��� probe ����

// ����һ�� platform_device
static struct platform_device uda1341_dev = {
    .name         = "uda1341-codec",   // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_name һ��
    .id       = -1,
    .dev = { 
    	.release = uda1341_dev_release, 
	},
};


// ����һ�� platform_driver
struct platform_driver uda1341_drv = {
	.probe		= uda1341_probe,
	.remove		= uda1341_remove,
	.driver		= {
		.name	= "uda1341-codec",   // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_name һ��
	}
};


// ��ں���
static int uda1341_init(void)
{
    gpbcon = ioremap(0x56000010, 4);        // ӳ�䵽�����ַ
    gpbdat = ioremap(0x56000014, 4);        // ӳ�䵽�����ַ
    
    platform_device_register(&uda1341_dev);  // ע��ƽ̨�豸
    platform_driver_register(&uda1341_drv);  // ע��ƽ̨����

    return 0;
}


// ���ں���
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
