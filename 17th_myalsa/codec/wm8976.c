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



/* �����ʴ� 8K ֧�ֵ� 48K --- ���WM8976оƬ�ֲ� P1 */
#define WM8976_RATES   SNDRV_PCM_RATE_8000_48000


#define WM8976_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	        SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


#define WM8976_REG_NUM 58  // wm8976 оƬһ���� 58 ���Ĵ���


/* wm8976 ���мĴ�����Ĭ��ֵ */
// unsigned short 7 λ��ֵַ��9λ����
static const unsigned short wm8976_reg[WM8976_REG_NUM] = {
};

static volatile unsigned int *gpbdat;
static volatile unsigned int *gpbcon;


/*
 * ���������Ϣ,������Сֵ���ֵ
 */
int wm8976_info_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;                           // ˫����
	uinfo->value.integer.min = 0;               // ����wm8976������Сֵ 0
	uinfo->value.integer.max = 63;              // ����wm8976�������ֵ 63
	return 0;
}


/*
 * ��õ�ǰ����ֵ
 */
int wm8976_get_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	// �����Ĵ��� --- ��� WM8976 P100  . ȫ 0 ��ʱ���ʾ��С , ȫ 1 ��ʱ���ʾ���
	// ����Ҫע���ڻ���������� wm8976_get_vol ���棬Ҫע��ֻ��Ҫ������6λ��
	// ��Ϊwm8976оƬ�Ǹ��Ĵ���ֻ�е�6λ��ʾ��ǰ����ֵ
	ucontrol->value.integer.value[0] = snd_soc_read(codec, 52) & 0x3f; // ������ 6 λ
	ucontrol->value.integer.value[1] = snd_soc_read(codec, 53) & 0x3f; // ������ 6 λ
	
	return 0;
}


/*
 * ���õ�ǰ����ֵ
 */
int wm8976_put_vol(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	// ��Ҫ���õ�����ֵд��Ĵ���
	// �鿴wm8976оƬ�ֲ� P100 ������ 1<<8 �ĺ��壺
	// ֻ�е� 52 ��53 �żĴ����ĵ�8λ���� 1 �ˣ������õ�����ֵ�Żᱻ����
	// ��ô����Ҫע���ڻ���������� wm8976_get_vol ���棬Ҫע��ֻ��Ҫ������6λ��
	// ��Ϊwm8976оƬ�Ǹ��Ĵ���ֻ�е�6λ��ʾ��ǰ����ֵ
	val = ucontrol->value.integer.value[0];
    snd_soc_write(codec, 52, (1<<8)|val);

	val = ucontrol->value.integer.value[1];
    snd_soc_write(codec, 53, (1<<8)|val);
    
	return 0;
}

static void wm8976_init_regs(struct snd_soc_codec *codec);


// ��������
// ÿ��������������� snd_kcontrol_new ���͵ı���������ͬ��ΪʲôӦ�ó��򶼿���ȥ�������ǵ����� ?
// �϶��Ƕ��ڳ��õ�������Լ���׳ɵ�����
// Ӧ�ó���������������ҵ� snd_kcontrol_new ���������� put ����
// �� uda1341_soc_probe ���������ע���ȥ
static const struct snd_kcontrol_new wm8976_vol_control = 
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, 
    .name = "Master Playback Volume",      // ������ֺ���Ҫ����������������˵�������������
	.info = wm8976_info_vol,               // ���һЩ��Ϣ������˵���������Χ�� 0 ~ 100 ���� 0 ~ 99
	.get  = wm8976_get_vol,                // ��õ�ǰ������ֵ
	.put  = wm8976_put_vol,                // ��������
};


// ����� probe �������ʼ��wm8976 �Ĵ��� �������������Ƶ�
static int wm8976_soc_probe(struct snd_soc_codec *codec)
{
    int ret;

    wm8976_init_regs(codec); // ��ʼ�� wm8976 �Ĵ���
    
	// wm8976_vol_control  --- �������ƽṹ��
	// 1                   --- 1��
	ret = snd_soc_add_codec_controls(codec, &wm8976_vol_control, 1);
    return ret;
}

/* wm8976�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
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



// ����ģʽ���� GPB2 ��ƽ
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


/* UDA1341�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
 */

// wm8976 д�Ĵ�������,����7λ�ĵ�ֵַ��9λ�����ݣ���16λ
static int wm8976_write_reg(struct snd_soc_codec *codec, unsigned int reg,unsigned int value)
{
	u8 *cache = codec->reg_cache;
	int i;

	// reg << 9       :  ������ 7 λ��ԭ���ĵ� 7 λ���Ƶ��� 7 λ��
	// value & 0x1ff  :  ������ 9 λ
	unsigned short val = (reg << 9) | (value & 0x1ff);  // �ϲ�ΪҪ������16λ val ֵ

	if (reg >= WM8976_REG_NUM)  // ����Ĵ��������� wm8976 �ļĴ���������֧��
		return -1;
	
	/* �ȱ��潫Ҫд�� wm8976 �Ĵ�����ֵ */
	cache[reg] = value;

    /* ��д��Ӳ�� */
    set_csb(1);
    set_dat(1);
    set_clk(1);

	// һλһλ�����ݷ��ͳ�ȥ,�ȷ������λ
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

/* UDA1341�ļĴ�����֧�ֶ�����
 * Ҫ֪��ĳ���Ĵ����ĵ�ǰֵ,
 * ֻ����д��ʱ��������
 */

// ����һ�� snd_soc_codec_driver ---- codec ���ƽӿ�(L3�ӿ�)
// ע�� :
// ��� snd_soc_codec_driver ��û�����֣���ô machine ������ô֪��������� snd_soc_codec_driver ?
// ע����� snd_soc_codec_driver ��ʱ�򣬼̳�ʹ�� platform_device ������֡�( snd_soc_register_codec(&pdev->dev,&soc_codec_dev_wm8976, &wm8976_dai, 1); )
static struct snd_soc_codec_driver soc_codec_dev_wm8976 = {
    .probe = wm8976_soc_probe,             // ����� probe �������ʼ��wm8976 �Ĵ��� �������������Ƶ�
	.reg_cache_size = sizeof(wm8976_reg),  // ����д�� wm8976 �Ĵ�����ֵ
	.reg_word_size = sizeof(u16),          // wm8976_reg �����ÿһ��ռ��16λ
	.reg_cache_default = wm8976_reg,
	.reg_cache_step = 2,                   // �������ֽڱ�ʾһ���Ĵ�����ֵ
	.read  = wm8976_read_reg_cache,        // ���Ĵ�������
	.write = wm8976_write_reg,             // д�Ĵ�������
};


// wm8976 ��ʼ���Ĵ�������
static void wm8976_init_regs(struct snd_soc_codec *codec)
{

	/* GPB 4: L3CLOCK */
	/* GPB 3: L3DATA */
	/* GPB 2: L3MODE */
    *gpbcon &= ~((3<<4) | (3<<6) | (3<<8));
    *gpbcon |= ((1<<4) | (1<<6) | (1<<8));

	/* software reset */
	wm8976_write_reg(codec, 0, 0);

	/* OUT2����/��������
	 * ��/��ͨ�����������
	 * ��/��DAC��
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


// �����оƬ�������ú��� 
static int wm8976_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
    /* ����params��ֵ,����wm8976�ļĴ��� 
     * ����ʱ������,��ʽ
     */
    /* Ϊ�˼�, �� wm8976_init_regs ������ú�ʱ�ӡ���ʽ�Ȳ��� */
    return 0;
}


// ����һ�� snd_soc_dai_ops �ṹ��
static const struct snd_soc_dai_ops wm8976_dai_ops = {
	.hw_params	= wm8976_hw_params,
};

// snd_soc_dai_driver  ---  codec DAI ( IIS �ӿ� )
static struct snd_soc_dai_driver wm8976_dai = {
	.name = "wm8976-iis",         // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_dai_name һ��
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",  // ����
		.channels_min = 1,          // ��Сͨ���� 1 . ˵�� wm8976 ֧�ֵ���������
		.channels_max = 2,          // ���ͨ���� 2 . ˵�� wm8976 ֧��˫��������
		.rates = WM8976_RATES,      // ������
		.formats = WM8976_FORMATS,  // ��ʽ
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",   // ����ָ¼��
		.channels_min = 1,          // ��Сͨ���� 1 . ˵�� wm8976 ֧�ֵ�����¼��
		.channels_max = 2,          // ���ͨ���� 2 . ˵�� wm8976 ֧��˫����¼��
		.rates = WM8976_RATES,      // ������
		.formats = WM8976_FORMATS,  // ��ʽ
	},
	/* pcm operations */
	.ops = &wm8976_dai_ops,        // ����ṹ�����и��ֲ���
};


/* ͨ��ע��ƽ̨�豸��ƽ̨������ʵ�ֶ�snd_soc_register_codec�ĵ���
 *
 */

static void wm8976_dev_release(struct device * dev)
{
}


// �ں�������ͬ���� platform_device ��platform_driver ,�ͻ���� probe ����
// ע�� codec ���ƽӿ�(L3�ӿ�)  ��  codec dai �ӿ�
// soc_codec_dev_wm8976 �����ּ̳�ʹ�� platform_device �� platform_driver ������
static int wm8976_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,&soc_codec_dev_wm8976, &wm8976_dai, 1);
}

static int wm8976_remove(struct platform_device *pdev)
{
    snd_soc_unregister_codec(&pdev->dev);
    return 0;
}

// �˴�Ӧ�� platform_device , platform_driver С���� :
// ���ں�����ͬ���� platform_device ��platform_driver ʱ���ͻ���� platform_driver ��� probe ����

// ����һ�� platform_device
static struct platform_device wm8976_dev = {
    .name         = "wm8976-codec",   // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_name һ��
    .id       = -1,
    .dev = { 
    	.release = wm8976_dev_release, 
	},
};


// ����һ�� platform_driver
struct platform_driver wm8976_drv = {
	.probe		= wm8976_probe,     // ע�� codec ���ƽӿ�(L3�ӿ�)  ��  codec dai �ӿ�
	.remove		= wm8976_remove,
	.driver		= {
		.name	= "wm8976-codec",   // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .codec_name һ��
	}
};


// ��ں���
static int wm8976_init(void)
{
    gpbcon = ioremap(0x56000010, 4);        // ӳ�䵽�����ַ
    gpbdat = ioremap(0x56000014, 4);        // ӳ�䵽�����ַ
    
    platform_device_register(&wm8976_dev);  // ע��ƽ̨�豸
    platform_driver_register(&wm8976_drv);  // ע��ƽ̨����
    return 0;
}


// ���ں���
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