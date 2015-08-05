#ifndef __MT6752_SMI_DEBUG_H__
#define __MT6752_SMI_DEBUG_H__

#define SMI_DBG_DISPSYS (1<<0)
#define SMI_DBG_VDEC (1<<1)
#define SMI_DBG_IMGSYS (1<<2)
#define SMI_DBG_VENC (1<<3)
#define SMI_DBG_MJC (1<<4)

#define SMI_DGB_LARB_SELECT(smi_dbg_larb,n) ((smi_dbg_larb) & (1<<n))

#ifndef CONFIG_MTK_SMI
    #define smi_debug_bus_hanging_detect(larbs, show_dump) {}
    #define smi_debug_bus_ovl_request_done_check(ovl_num) {}
#else
    int smi_debug_bus_hanging_detect(unsigned int larbs, int show_dump);
    
    // Check if OVL SMI bus request has been cleared
    // 1: No OVL request in SMI
    // 0: SMI is still handling OVL request
    int smi_debug_bus_ovl_request_done_check(int ovl_num);
#endif

#endif //__MT6752_SMI_DEBUG_H__

