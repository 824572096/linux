#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <sound/soc.h>


/* �ο�sound\soc\samsung\s3c24xx_uda134x.c
 */


/*
 * 1. ����ע��һ����Ϊsoc-audio��ƽ̨�豸
 * 2. ���ƽ̨�豸��һ��˽������ snd_soc_card
 *    snd_soc_card����һ��snd_soc_dai_link
 *    snd_soc_dai_link����������ASOC�����ֵ�����
 *    ASOC�����������Ǹ��� snd_soc_dai_link �ṹ�������������Ӧ���ҵ������ֵ���������
 */

static struct snd_soc_ops s3c2440_uda1341_ops = {
	//.hw_params = s3c24xx_uda134x_hw_params,
};


// snd_soc_dai_link ȷ������������������Ӧ����������
// �������������ж�� codec ������ʹ����һ�� codec ���� .codec_name  .codec_dai_name ����
static struct snd_soc_dai_link s3c2440_uda1341_dai_link = {
	.name = "100ask_UDA1341",           // ������ֲ���Ҫ
	.stream_name = "100ask_UDA1341",    // ������ֲ���Ҫ
	.codec_name = "wm8976-codec",       // ���ĸ�codec    . ���������Ҫ���� wm8976.c ���� platform_device , platform_driver ����Ҳ�������������
	.codec_dai_name = "wm8976-iis",     // codec   �� DAI . ���������Ҫ���� wm8976.c ���� snd_soc_dai_driver ������ҲҪʹ���������
	.cpu_dai_name = "s3c2440-iis",      // s3c2440 �� DAI . ���������Ҫ���� s3c2440_iis.c ���� platform_device , platform_driver ����Ҳ�������������
	.ops = &s3c2440_uda1341_ops,
	.platform_name	= "s3c2440-dma",    // dma            . ���������Ҫ���� s3c2440_dma.c ���� platform_device , platform_driver ����Ҳ�������������
};


// ��Ϊsoc-audio��ƽ̨�豸��һ��˽������ snd_soc_card
static struct snd_soc_card myalsa_card = {
	.name = "S3C2440_UDA1341",               // ���� : S3C2440_UDA1341 �������������ν
	.owner = THIS_MODULE,
	.dai_link = &s3c2440_uda1341_dai_link,   // �ؼ�������ṹ��
	.num_links = 1,
};

static void asoc_release(struct device * dev)
{
}


// ����һ����Ϊ soc-audio ��ƽ̨�豸
static struct platform_device asoc_dev = {
    .name         = "soc-audio",
    .id       = -1,
    .dev = { 
    	.release = asoc_release, 
	},
};


// ��ں���
static int s3c2440_uda1341_init(void)
{
	platform_set_drvdata(&asoc_dev, &myalsa_card); // �� myalsa_card �ŵ� asoc_dev ��˽�г�Ա����ȥ

    platform_device_register(&asoc_dev);             // ע��ƽ̨�豸

    return 0;
}


// ���ں���
static void s3c2440_uda1341_exit(void)
{
    platform_device_unregister(&asoc_dev);
}


module_init(s3c2440_uda1341_init);
module_exit(s3c2440_uda1341_exit);

MODULE_LICENSE("GPL");
