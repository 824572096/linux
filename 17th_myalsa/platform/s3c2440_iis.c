#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <mach/regs-gpio.h>
#include <mach/dma.h>

/* �ο�sound\soc\samsung\s3c24xx-i2s.c
 */


// ����һ�� s3c2440 iis �Ĵ����ṹ������
struct s3c2440_iis_regs {
    unsigned int iiscon ; 
    unsigned int iismod ; 
    unsigned int iispsr ; 
    unsigned int iisfcon; 
    unsigned int iisfifo; 
};

static volatile unsigned int *gpecon; // ����ָ�� GPIOE ���ƼĴ���ӳ�䵽�����ַ

static volatile struct s3c2440_iis_regs *iis_regs; // ����ָ�� iis_regs �Ĵ���ӳ�䵽�����ַ

#define ABS(a, b) ((a>b)?(a-b):(b-a))


// ���� IIS
static void s3c2440_iis_start(void)
{
    iis_regs->iiscon |= (1);
}


// �ر� IIS
static void s3c2440_iis_stop(void)
{
    iis_regs->iiscon &= ~(1);
}


// ƽ̨ iis �������Ĵ����������ú���
static int s3c2440_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
    /* ����params����IIS������ */

    int tmp_fs;        // ���ڱ���I2S����������Ƶ��
    int i;
    int min = 0xffff;
    int pre = 0;       // ���ڱ���һ����I2S����������Ƶ����WAV�ļ���ļ�¼�Ĳ���Ƶ����ӽ���ֵ
    unsigned int fs;   // �������������
	
    struct clk *clk = clk_get(NULL, "pclk");

    /* ����GPIO����IIS */
    *gpecon &= ~((3<<0) | (3<<2) | (3<<4) | (3<<6) | (3<<8));
    *gpecon |= ((2<<0) | (2<<2) | (2<<4) | (2<<6) | (2<<8));
    
    
    /* bit[9] : Master clock select, 0-PCLK
     * bit[8] : 0 = Master mode
     * bit[7:6] : 10 = Transmit mode
     * bit[4] : 0-IIS compatible format
     * bit[2] : 384fs, ȷ����MASTER CLOCK֮��, fs = MASTER CLOCK/384
     * bit[1:0] : Serial bit clock frequency select, 32fs
     */

	// params_format(params) �õ�ÿ������ֵ�ö���λ��ʾ
	
    if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE) // ���ÿ������ֵ��16λ��ʾ
        iis_regs->iismod = (2<<6) | (0<<4) | (1<<3) | (1<<2) | (1);
    else if (params_format(params) == SNDRV_PCM_FORMAT_S8) // ���ÿ������ֵ��8λ��ʾ
        iis_regs->iismod = (2<<6) | (0<<4) | (0<<3) | (1<<2) | (1);
    else  // �����Ĳ�֧��
        return -EINVAL;

    /* ʱ ��  :  Master clock = PCLK/(n+1)
     * ������ :  fs = Master clock / 384  =  PCLK / (n+1) / 384 = 50000000 / (n+1) / 384
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
    fs = params_rate(params);   // �õ�������
	
    for (i = 0; i <= 31; i++)   // �� 0 ~ 31 ������һ��ʹI2S���õĲ���Ƶ����WAV�ļ���ļ�¼�Ĳ���Ƶ����ӽ���ֵ
    {
        tmp_fs = clk_get_rate(clk)/384/(i+1);
		
        if (ABS(tmp_fs, fs) < min)
        {
            min = ABS(tmp_fs, fs);
            pre = i;
        }
    }
    iis_regs->iispsr = (pre << 5) | (pre); // ���÷�Ƶ����

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


// ���� cmd ������ ����/ֹͣ IIS
static int s3c2440_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) 
	{
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
       		s3c2440_iis_start();  // ���� IIS
			break;
			
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			
		default:
        	s3c2440_iis_stop();  // ֹͣ IIS
			ret = -EINVAL;
			break;
	}

exit_err:
	return ret;
}


// ����һ�� snd_soc_dai_ops �ṹ��
static const struct snd_soc_dai_ops s3c2440_i2s_dai_ops = {
	.hw_params	= s3c2440_i2s_hw_params,   // ƽ̨ iis �������Ĵ����������ú���
	.trigger	= s3c2440_i2s_trigger,     // ���� cmd ������ ����/ֹͣ IIS
};

#define S3C24XX_I2S_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)


// ����һ�� snd_soc_dai_driver  ---  CPU DAI ���ýӿ� ( CPU �� IIS ���ýӿ� )
// ע�� :
// ��� snd_soc_dai_driver ��û�����֣���ô machine ������ô֪��������� snd_soc_dai_driver ?
// ע����� snd_soc_dai_driver ��ʱ�򣬼̳�ʹ�� platform_device ������֡�( snd_soc_register_dai(&pdev->dev, &s3c2440_i2s_dai) )
static struct snd_soc_dai_driver s3c2440_i2s_dai = {
	.playback = {
		.channels_min = 2,  // ��Сͨ���� 2
		.channels_max = 2,  // ���ͨ���� 2 . ��Сͨ�������ͨ������ 2 ��˵�� 2440 ֻ֧��˫����
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,  // ��Сͨ���� 2
		.channels_max = 2,  // ���ͨ���� 2 . ��Сͨ�������ͨ������ 2 ��˵�� 2440 ֻ֧��˫����
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &s3c2440_i2s_dai_ops,  // ���ݴ�����صĲ���
};


// �ں�������ͬ���� platform_device ��platform_driver ,�ͻ���� probe ����
static int s3c2440_iis_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &s3c2440_i2s_dai);   // ע�� s3c2440_i2s_dai
}


static int s3c2440_iis_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
    return 0;
}

static void s3c2440_iis_release(struct device * dev)
{
}


// �˴�Ӧ�� platform_device , platform_driver С���� :
// ���ں�����ͬ���� platform_device ��platform_driver ʱ���ͻ���� platform_driver ��� probe ����

// ����һ�� platform_device
static struct platform_device s3c2440_iis_dev = {
    .name         = "s3c2440-iis", // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .cpu_dai_name һ��
    .id       = -1,
    .dev = { 
    	.release = s3c2440_iis_release, 
	},
};

// ����һ�� platform_driver
struct platform_driver s3c2440_iis_drv = {
	.probe		= s3c2440_iis_probe,
	.remove		= s3c2440_iis_remove,
	.driver		= {
		.name	= "s3c2440-iis",   // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .cpu_dai_nameһ��
	}
};


static int s3c2440_iis_init(void)
{
    struct clk *clk;  // ���ڴ�� IIS ������ʱ��
	
    gpecon   = ioremap(0x56000040, 4);                               // ӳ�䵽�����ַ
    iis_regs = ioremap(0x55000000, sizeof(struct s3c2440_iis_regs)); // ӳ�䵽�����ַ

	// �� IIS ������ʱ��
	
	clk = clk_get(NULL, "iis"); // ��� IIS ������ʱ��
    clk_enable(clk);
    clk_put(clk);

    platform_device_register(&s3c2440_iis_dev);
    platform_driver_register(&s3c2440_iis_drv);
    return 0;
}

static void s3c2440_iis_exit(void)
{
    struct clk *clk; // ���ڴ�� IIS ������ʱ��

    platform_device_unregister(&s3c2440_iis_dev);  // ע��ƽ̨�豸
    platform_driver_unregister(&s3c2440_iis_drv);  // ע��ƽ̨����
    iounmap(gpecon);
    iounmap(iis_regs);

	// �ر� IIS ������ʱ��
    
	clk = clk_get(NULL, "iis"); // ��� IIS ������ʱ��
    clk_disable(clk);
    clk_put(clk);
}

module_init(s3c2440_iis_init);
module_exit(s3c2440_iis_exit);

MODULE_LICENSE("GPL");

