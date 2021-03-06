#ifndef __SDRAM_CONFIG_H
#define __SDRAM_CONFIG_H

#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMTYPE			(2)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_MEMBL			(8)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ADDRORDER		(0)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCEN			(0)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_ECCCORREN		(0)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_REORDEREN		(1)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_STARVELIMIT		(10)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_DQSTRKEN			(0)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLCFG_NODMPINS			(0)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCWL			(6)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_AL			(0)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TCL			(6)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRRD			(4)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TFAW			(14)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING1_TRFC			(117)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TREFI		(1300)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRCD		(5)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TRP		(5)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWR		(5)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING2_IF_TWTR		(4)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRTP			(4)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRAS			(12)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TRC			(17)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TMRD			(4)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING3_TCCD			(4)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_SELFRFSHEXIT		(200)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMTIMING4_PWRDOWNEXIT		(3)
#define CONFIG_HPS_SDR_CTRLCFG_LOWPWRTIMING_AUTOPDCYCLES	(0)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_COLBITS		(10)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_ROWBITS		(15)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_BANKBITS		(3)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMADDRW_CSBITS			(1)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMIFWIDTH_IFWIDTH		(32)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMDEVWIDTH_DEVWIDTH		(8)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMINTR_INTREN			(0)
#define CONFIG_HPS_SDR_CTRLCFG_STATICCFG_MEMBL			(2)
#define CONFIG_HPS_SDR_CTRLCFG_STATICCFG_USEECCASDATA		(0)
#define CONFIG_HPS_SDR_CTRLCFG_CTRLWIDTH_CTRLWIDTH		(2)
#define CONFIG_HPS_SDR_CTRLCFG_PORTCFG_AUTOPCHEN		(0)
#define CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_SYNCMODE			(0)
#define CONFIG_HPS_SDR_CTRLCFG_FIFOCFG_INCSYNC			(0)
#define CONFIG_HPS_SDR_CTRLCFG_MPPRIORITY_USERPRIORITY		(0x3FFD1088)
#define CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_0_STATICWEIGHT_31_0	(0x21084210)
#define CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_STATICWEIGHT_49_32	(0x1EF84)
#define CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_1_SUMOFWEIGHT_13_0	(0x2020)
#define CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_2_SUMOFWEIGHT_45_14	(0x0)
#define CONFIG_HPS_SDR_CTRLCFG_MPWIEIGHT_3_SUMOFWEIGHT_63_46	(0xF800)
#define CONFIG_HPS_SDR_CTRLCFG_PHYCTRL_PHYCTRL_0		(0x200)

#define CONFIG_HPS_SDR_CTRLCFG_CPORTWIDTH_CPORTWIDTH		(0x44555)
#define CONFIG_HPS_SDR_CTRLCFG_CPORTWMAP_CPORTWMAP		(0x2C011000)
#define CONFIG_HPS_SDR_CTRLCFG_CPORTRMAP_CPORTRMAP		(0xB00088)
#define CONFIG_HPS_SDR_CTRLCFG_RFIFOCMAP_RFIFOCMAP		(0x760210)
#define CONFIG_HPS_SDR_CTRLCFG_WFIFOCMAP_WFIFOCMAP		(0x980543)
#define CONFIG_HPS_SDR_CTRLCFG_CPORTRDWR_CPORTRDWR		(0x5A56A)
#define CONFIG_HPS_SDR_CTRLCFG_MPPACING_0_THRESHOLD1_31_0	(0x20820820)
#define CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD1_59_32	(0x8208208)
#define CONFIG_HPS_SDR_CTRLCFG_MPPACING_1_THRESHOLD2_3_0	(0)
#define CONFIG_HPS_SDR_CTRLCFG_MPPACING_2_THRESHOLD2_35_4	(0x41041041)
#define CONFIG_HPS_SDR_CTRLCFG_MPPACING_3_THRESHOLD2_59_36	(0x410410)
#define CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_0_THRESHOLDRSTCYCLES_31_0 (0x80808080)
#define CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_1_THRESHOLDRSTCYCLES_63_32 (0x80808080)
#define CONFIG_HPS_SDR_CTRLCFG_MPTHRESHOLDRST_2_THRESHOLDRSTCYCLES_79_64 (0x8080)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMODT_READ			(0)
#define CONFIG_HPS_SDR_CTRLCFG_DRAMODT_WRITE			(1)
#define CONFIG_HPS_SDR_CTRLCFG_FPGAPORTRST_READ_PORT_USED	(0)
#define CONFIG_HPS_SDR_CTRLCFG_FPGAPORTRST_WRITE_PORT_USED	(0)
#define CONFIG_HPS_SDR_CTRLCFG_FPGAPORTRST_COMMAND_PORT_USED	(0)

#endif	/*#ifndef__SDRAM_CONFIG_H*/
