#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <sound/soc.h>


/* 参考sound\soc\samsung\s3c24xx_uda134x.c
 */


/*
 * 1. 分配注册一个名为soc-audio的平台设备
 * 2. 这个平台设备有一个私有数据 snd_soc_card
 *    snd_soc_card里有一项snd_soc_dai_link
 *    snd_soc_dai_link被用来决定ASOC各部分的驱动
 *    ASOC的驱动程序是根据 snd_soc_dai_link 结构体里的名字来对应地找到各部分的驱动程序
 */

static struct snd_soc_ops s3c2440_uda1341_ops = {
	//.hw_params = s3c24xx_uda134x_hw_params,
};


// snd_soc_dai_link 确定了声卡各部分所对应的驱动程序
// 驱动程序里面有多个 codec ，具体使用哪一个 codec ，由 .codec_name  .codec_dai_name 决定
static struct snd_soc_dai_link s3c2440_uda1341_dai_link = {
	.name = "100ask_UDA1341",           // 这个名字不重要
	.stream_name = "100ask_UDA1341",    // 这个名字不重要
	.codec_name = "wm8976-codec",       // 用哪个codec    . 这个名字重要，在 wm8976.c 里面 platform_device , platform_driver 名字也必须是这个名字
	.codec_dai_name = "wm8976-iis",     // codec   的 DAI . 这个名字重要，在 wm8976.c 里面 snd_soc_dai_driver 的名字也要使用这个名字
	.cpu_dai_name = "s3c2440-iis",      // s3c2440 的 DAI . 这个名字重要，在 s3c2440_iis.c 里面 platform_device , platform_driver 名字也必须是这个名字
	.ops = &s3c2440_uda1341_ops,
	.platform_name	= "s3c2440-dma",    // dma            . 这个名字重要，在 s3c2440_dma.c 里面 platform_device , platform_driver 名字也必须是这个名字
};


// 名为soc-audio的平台设备有一个私有数据 snd_soc_card
static struct snd_soc_card myalsa_card = {
	.name = "S3C2440_UDA1341",               // 名称 : S3C2440_UDA1341 ，这个名称无所谓
	.owner = THIS_MODULE,
	.dai_link = &s3c2440_uda1341_dai_link,   // 关键是这个结构体
	.num_links = 1,
};

static void asoc_release(struct device * dev)
{
}


// 分配一个名为 soc-audio 的平台设备
static struct platform_device asoc_dev = {
    .name         = "soc-audio",
    .id       = -1,
    .dev = { 
    	.release = asoc_release, 
	},
};


// 入口函数
static int s3c2440_uda1341_init(void)
{
	platform_set_drvdata(&asoc_dev, &myalsa_card); // 把 myalsa_card 放到 asoc_dev 的私有成员里面去

    platform_device_register(&asoc_dev);             // 注册平台设备

    return 0;
}


// 出口函数
static void s3c2440_uda1341_exit(void)
{
    platform_device_unregister(&asoc_dev);
}


module_init(s3c2440_uda1341_init);
module_exit(s3c2440_uda1341_exit);

MODULE_LICENSE("GPL");
