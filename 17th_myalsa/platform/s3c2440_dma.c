#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>


/* �ο� sound\soc\samsung\dma.c
 */

/* 1. ����DMA BUFFER,��� buffer ����DMA����         s3c2440_dma_new
 * 2. ��BUFFERȡ��period
 * 3. ����DMA����
 * 4. һ�δ������,����״̬(hw_ptr)
 *    2,3,4�ⲿ����Ҫ��: request_irq, ����DMA����, �жϴ���
 */

#define DMA0_BASE_ADDR  0x4B000000  // DMA0 �Ĵ��������ַ
#define DMA1_BASE_ADDR  0x4B000040  // DMA1 �Ĵ��������ַ
#define DMA2_BASE_ADDR  0x4B000080  // DMA2 �Ĵ��������ַ
#define DMA3_BASE_ADDR  0x4B0000C0  // DMA3 �Ĵ��������ַ


// ���� s3c dma �Ĵ����ṹ������
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

static volatile struct s3c_dma_regs *dma_regs;  // ����һ���ṹ��ָ�룬���ָ��ָ�� s3c dma �Ĵ���


// Ӧ�ó����һ������д��buffer���ٰ���һ������д��buffer��.
// Ӧ�ó���д���λ�ó�Ϊ appl_ptr , ÿд���һ�����ݳ�Ϊһ�� period,ÿ�� period�����ж��frame��һ��frame������һ������������ݡ�
// ������������buffer����ȡ��һ��period���ݷ���Ӳ������ȡ��һ��period���ݷ���Ӳ����.
// �������������λ�ó�Ϊhw_ptr��
// ���ַ������Խ�������϶����������⡣��Ϊ������ԴԴ���ϵش�Ӧ�ó��򷢸����������������򷢸�Ӳ����
// ʹ�����ַ����ͱ��봴��һ���ǳ���Ļ������������������������������������ġ�


// ����һ������ dma_buffer �ṹ������
struct s3c2440_dma_info {
    unsigned int buf_max_size;  // ���ڴ����������� dma_buffer ����С  .  �㾡�ܷ���dma_buffer��С , ���ǵ����ö����APP����
    unsigned int buffer_size;   // ���ڴ��Ӧ�ó���ʹ���� dma_buffer ���ռ�
    unsigned int period_size;   // ���ڴ��һ�� period �Ĵ�С
    unsigned int phy_addr;      // ���ڴ����������� dma_buffer �����ַ
    unsigned int virt_addr;     // ���ڴ����������� dma_buffer �����ַ
    unsigned int dma_ofs;       // dma ƫ�Ƶ�ַ
    unsigned int be_running;    // �Ƿ������� dma ����  1  --- ���ڽ���DMA����    0 --- δ����DMA����
};

static struct s3c2440_dma_info playback_dma_info; // ����һ���������ڲ��ŵ� dma_buffer �ṹ�����

                                                  // ����㻹Ҫ¼���Ļ����ٶ���һ������¼���Ľṹ�����
                                                  // ������ֻ��ʾ���Ź���


// SNDRV_PCM_INFO_INTERLEAVED  ���ݴ�ŵ����з�ʽ�����������������ݽ�������

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
	.buffer_bytes_max	= 128*1024,     // dma_buffer  128k  . �㾡�ܷ��� 128k , �����ò��õ� 128k ��APP����
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};


// Ӧ�ó����һ������д��buffer���ٰ���һ������д��buffer��.
// Ӧ�ó���д���λ�ó�Ϊ appl_ptr , ÿд���һ�����ݳ�Ϊһ�� period,ÿ�� period�����ж��frame��һ��frame������һ������������ݡ�
// ������������buffer����ȡ��һ��period���ݷ���Ӳ������ȡ��һ��period���ݷ���Ӳ����.
// �������������λ�ó�Ϊhw_ptr��
// ���ַ������Խ�������϶����������⡣��Ϊ������ԴԴ���ϵش�Ӧ�ó��򷢸����������������򷢸�Ӳ����
// ʹ�����ַ����ͱ��봴��һ���ǳ���Ļ������������������������������������ġ�


/* ���ݴ���: Դ,Ŀ��,���� */
// ��һ�� period ���ݼ��ؽ� DMA 
static void load_dma_period(void)
{       
	/* ��Դ��ַ,Ŀ�ĵ�ַ,Ҫ��������ݳ��ȸ���DMA */
	dma_regs->disrc      = playback_dma_info.phy_addr + playback_dma_info.dma_ofs; /* Դ�������ַ */
	dma_regs->disrcc     = (0<<1) | (0<<0);                                       /* Դλ��AHB����, Դ��ַ���� */
	dma_regs->didst      = 0x55000010;                                           /* Ŀ�ĵ������ַ --- �� s3c IIS ������ */
	dma_regs->didstc     = (0<<2) | (1<<1) | (1<<0);                              /* Ŀ��λ��APB����, Ŀ�ĵ�ַ���� */

    /* bit22: 0 -  reload   ��ʾһ��DMA������֮�󣬻��Լ�����ĳЩֵ��Ȼ������DMA */
	/* bit22: 1 -  noreload ��ʾ��������֮�󣬲��Լ�����ֵ��������DMA */
	
	// ��Ϊ������ DMA�жϷ�����������Լ���������һ��DMA����Ҫ��������ݣ��Լ�����DMA������ bit22 Ӧ����Ϊ 1
	// bit22 ��Ϊ 0 ����������
	
	/* ʹ���ж�,��������,Ӳ������ */
	// playback_dma_info.period_size/2  :  һ��DMA����ĳ���
	// Ϊʲô���� 2 ?
	// ��Ϊ 1<<20 �� dcon �Ĵ����� bit20 ��1 ����DMA��һ�δ���2���ֽ�( ��s3c2440оƬ�ֲ� P270 )
	dma_regs->dcon       = (1<<31)|(0<<30)|(1<<29)|(0<<28)|(0<<27)|(0<<24)|(1<<23)|(1<<22)|(1<<20)|(playback_dma_info.period_size/2);
}


// ���� DMA ����
static void s3c2440_dma_start(void)
{
	/* ����DMA */
	dma_regs->dmasktrig  = (1<<1);
}


// ֹͣ DMA ����
static void s3c2440_dma_stop(void)
{
	/* ֹͣDMA */
	dma_regs->dmasktrig  &= ~(1<<1);
}


// Ӧ�ó����һ������д��buffer���ٰ���һ������д��buffer��.
// Ӧ�ó���д���λ�ó�Ϊ appl_ptr , ÿд���һ�����ݳ�Ϊһ�� period,ÿ�� period�����ж��frame��һ��frame������һ������������ݡ�
// ������������buffer����ȡ��һ��period���ݷ���Ӳ������ȡ��һ��period���ݷ���Ӳ����.
// �������������λ�ó�Ϊhw_ptr��
// ���ַ������Խ�������϶����������⡣��Ϊ������ԴԴ���ϵش�Ӧ�ó��򷢸����������������򷢸�Ӳ����
// ʹ�����ַ����ͱ��봴��һ���ǳ���Ļ������������������������������������ġ�


// DMA2 �жϴ�����
// һ��DMA����һ��period , DMA ������ɣ������һ�� DMA �ж�
// ���жϴ����������״̬��Ϣ��������һ��period ���ٴ�����DMA����
static irqreturn_t s3c2440_dma2_irq(int irq, void *devid)
{
    struct snd_pcm_substream *substream = devid;
	
    playback_dma_info.dma_ofs += playback_dma_info.period_size;     // ָ����һ�� period

    if (playback_dma_info.dma_ofs >= playback_dma_info.buffer_size) // ���ƫ��ֵ������Ӧ�ó���ʹ�õ� dma_buffer ��С
    {
		playback_dma_info.dma_ofs = 0;  // �ص� dma_buffer ��ʼ��ַ
	}

    snd_pcm_period_elapsed(substream);  // ����hw_ptr����Ϣ,�����ж�:��� buffer ��û��������,����� s3c2440_dma_trigger ��ֹͣDMA

    if (playback_dma_info.be_running)   // ������ڽ��� dma ����
    {
		//�����������
		
        load_dma_period();   // ������һ��period ,��Ϊ��һ�� DMA���������
        s3c2440_dma_start(); // �ٴ�����DMA����
    }

    return IRQ_HANDLED;
}


// �� dma �������������ԣ�ע��DMA�����ж�
static int s3c2440_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
    int ret;

    /* �������� */
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	snd_soc_set_runtime_hwparams(substream, &s3c2440_dma_hardware);

    /* ע���ж� */
	// IRQ_DMA2          :  ͨ�� DMA2
	// s3c2440_dma2_irq  :  �жϴ�����,һ��DMA������ɣ������һ��DMA�ж�
	// IRQF_DISABLED     :  �������ж�ʱ�����жϴ�������У�����жϱ�������
	// substream         :  dev
    ret = request_irq(IRQ_DMA2, s3c2440_dma2_irq, IRQF_DISABLED, "myalsa for playback", substream);
    if (ret) // ���ע���жϳ���
    {
        printk("request_irq error!\n");
        return -EIO;
    }

	return 0;
}


// ����ƽ̨�� dma ��������
// params  :  ��Ӧ�ó��򴫽����� params ������ʹ�ö��� dma_buffer
static int s3c2440_dma_hw_params(struct snd_pcm_substream *substream,struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime  =   substream->runtime;
	unsigned long totbytes           =   params_buffer_bytes(params);  // �õ�Ӧ�ó���ʹ���� dma_buffer ���ռ�
    
    /* ����params����DMA */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

    /* s3c2440_dma_new�����˺ܴ��DMA BUFFER
     * params����ʹ�ö��,params��ֵ���û��ռ䴫����
     */
	runtime->dma_bytes            = totbytes;
    playback_dma_info.buffer_size = totbytes;                    // ��¼Ӧ�ó���ʹ���� dma_buffer ���ռ�
    playback_dma_info.period_size = params_period_bytes(params); // ��¼һ�� period �Ĵ�С

    return 0;
}


// ׼�� DMA ���亯��
// ��λ����״̬��Ϣ�����ص�һ�� period
static int s3c2440_dma_prepare(struct snd_pcm_substream *substream)
{
    /* ׼��DMA���� */

    /* ��λ����״̬��Ϣ */
    playback_dma_info.dma_ofs = 0;     // ƫ��ֵ����
    playback_dma_info.be_running = 0;  // �Ƿ����ڽ��� DMA �������� ��1  --- ���ڽ���DMA����    0 --- δ����DMA����
    
    /* ���ص�1��period */
    load_dma_period();

	return 0;
}


// ���� DMA ���亯��
// cmd   :  ���� cmd ��������ֹͣDMA����
static int s3c2440_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

    /* ����cmd������ֹͣDMA���� */


	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        /* ����DMA���� */
        playback_dma_info.be_running = 1;  // ��Ϊ 1 �������ڽ��� dma ����
        s3c2440_dma_start();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        /* ֹͣDMA���� */
        playback_dma_info.be_running = 0;  // ��Ϊ 0 ����δ���� dma ����
        s3c2440_dma_stop();
		break;

	default:
		ret = -EINVAL;
		break;
	}


	return ret;
}


/* �����Ѿ������˶��ٸ�frame */
static snd_pcm_uframes_t s3c2440_dma_pointer(struct snd_pcm_substream *substream)
{
	return bytes_to_frames(substream->runtime, playback_dma_info.dma_ofs);
}


// �ر� dma
static int s3c2440_dma_close(struct snd_pcm_substream *substream)
{
	// IRQ_DMA2  :  �жϺ�
	// substream :  dev_id
    free_irq(IRQ_DMA2, substream);
	
    return 0;
}


// ���� s3c2440_dma_ops �ṹ��
static struct snd_pcm_ops s3c2440_dma_ops = {
	.open		= s3c2440_dma_open,        // �� dma �������������ԣ�ע��DMA�����ж�
	.close		= s3c2440_dma_close,       // �ر� dma �������ͷ�DMA�ж�
	.ioctl		= snd_pcm_lib_ioctl,       // ioctl ���� snd_pcm_lib_ioctl ���ں��ṩ������Ҫ�����Լ�д
	.hw_params	= s3c2440_dma_hw_params,   // ����ƽ̨�� dma ��������
	.prepare    = s3c2440_dma_prepare,     // ׼�� dma ����
	.trigger	= s3c2440_dma_trigger,     // ���� dma ����,���� cmd ��������ֹͣDMA����
	.pointer	= s3c2440_dma_pointer,     // �����Ѿ������˶��ٸ�frame 
};


static u64 dma_mask = DMA_BIT_MASK(32);


// ��װ���������򣬴�������ʱ������������������� dma_buffer,��������� dma_buffer ��Ϣ���� ALSA ���Ĳ�
static int s3c2440_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	// snd_card ����˵������ALSA��Ƶ��������һ���ṹ����������������߼��ṹ��ʼ�ڸýṹ��������������
	// ����ص��߼��豸������snd_card�Ĺ���֮�£����������ĵ�һ������ͨ�����Ǵ���һ��snd_card�ṹ�塣
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm  *pcm  = rtd->pcm;

	/* �����������ڱ������� dma_buffer ��Ϣ */
	//  SNDRV_PCM_STREAM_PLAYBACK  ���ڲ���
	struct snd_pcm_substream *substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct snd_dma_buffer    *buf       = &substream->dma_buffer;

	int ret = 0;

    /* 1. ����DMA BUFFER */
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	
	//	SNDRV_PCM_STREAM_PLAYBACK  ���ڲ���
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
	{
		// ���� dma_buffer  :
		// playback_dma_info.virt_addr             :  ��ŷ���õ��Ļ������������ַ
		// s3c2440_dma_hardware.buffer_bytes_max   :  ����Ļ�������С
		// playback_dma_info.phy_addr              :  ����õ��Ļ������������ַ����������������
	    playback_dma_info.virt_addr = (unsigned int)dma_alloc_writecombine(pcm->card->dev, s3c2440_dma_hardware.buffer_bytes_max,&playback_dma_info.phy_addr, GFP_KERNEL);
	    
        if (!playback_dma_info.virt_addr) // �������ʧ��
        {
            return -ENOMEM;
        }

		// �������ִ�е����˵���ղŷ���Ļ���������ɹ��ˣ������¼����Ļ�������Ϣ
		
        playback_dma_info.buf_max_size = s3c2440_dma_hardware.buffer_bytes_max;  // ��¼����õ��Ļ�������С

		// ������� dma_buffer ��Ϣ���� , ���� ALSA �ĺ��Ĳ�
    	buf->dev.type      =  SNDRV_DMA_TYPE_DEV;
    	buf->dev.dev       =  pcm->card->dev;
    	buf->private_data  =  NULL;
        buf->area          =  playback_dma_info.virt_addr;
        buf->bytes         =  playback_dma_info.buf_max_size;
	}

	return ret;
}


// ��ж������������������ʱ����������������ͷ� s3c2440_dma_new ��������� dma_buffer
static void s3c2440_dma_free(struct snd_pcm *pcm)
{
	// playback_dma_info.buf_max_size  :  ǰ�����Ļ�������С
	// playback_dma_info.virt_addr     :  ǰ�����Ļ������������ַ
	// playback_dma_info.phy_addr      :  ǰ�����Ļ������������ַ
	dma_free_writecombine(pcm->card->dev, playback_dma_info.buf_max_size,(void *)playback_dma_info.virt_addr, playback_dma_info.phy_addr);
}


// ����һ�� snd_soc_platform_driver --- DMA ������
// ע�� :
// ��� snd_soc_platform_driver ��û�����֣���ô machine ������ô֪��������� snd_soc_platform_driver ?
// ע����� snd_soc_platform_driver ��ʱ�򣬼̳�ʹ�� platform_device ������֡�( snd_soc_register_platform(&pdev->dev, &s3c2440_dma_platform) )
static struct snd_soc_platform_driver s3c2440_dma_platform = {
	.ops		= &s3c2440_dma_ops,   // dma ���������ṹ��
	.pcm_new	= s3c2440_dma_new,    // ��װ���������򣬴�������ʱ������������������� dma_buffer
	.pcm_free	= s3c2440_dma_free,   // ��ж������������������ʱ����������������ͷ� s3c2440_dma_new ��������� dma_buffer
};


// �ں�������ͬ���� platform_device ��platform_driver ,�ͻ���� probe ����
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


// ���������� platform_device , platform_driver ��С���� :
// ���ں�����ͬ���� platform_device , platform_driver ,�ͻ���� platform_driver ��� probe ����

// ����һ�� platform_device
static struct platform_device s3c2440_dma_dev = {
    .name         = "s3c2440-dma", // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .platform_name һ��
    .id       = -1,
    .dev = { 
    	.release = s3c2440_dma_release, 
	},
};


// ����һ�� platform_driver
struct platform_driver s3c2440_dma_drv = {
	.probe		= s3c2440_dma_probe,
	.remove		= s3c2440_dma_remove,
	.driver		= {
		.name	= "s3c2440-dma", // �������Ҫ�� s3c2440_uda1341.c �� snd_soc_dai_link �� .platform_name һ��
	}
};


// ��ں���
static int s3c2440_dma_init(void)
{
    dma_regs = ioremap(DMA2_BASE_ADDR, sizeof(struct s3c_dma_regs));  // �� s3c dma2 �Ĵ���ӳ�䵽�����ַ

    platform_device_register(&s3c2440_dma_dev);   // ע��ƽ̨�豸
    platform_driver_register(&s3c2440_dma_drv);   // ע��ƽ̨����

    return 0;
}


// ���ں���
static void s3c2440_dma_exit(void)
{
    platform_device_unregister(&s3c2440_dma_dev); // ж��ƽ̨�豸
    platform_driver_unregister(&s3c2440_dma_drv); // ж��ƽ̨����
    iounmap(dma_regs);                            // ȡ��ӳ��
}

module_init(s3c2440_dma_init);
module_exit(s3c2440_dma_exit);

MODULE_LICENSE("GPL");

