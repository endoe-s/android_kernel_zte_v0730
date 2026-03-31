#ifndef FINGERPRINT_CORE_H
#define FINGERPRINT_CORE_H

struct fingerprint{
	struct pinctrl *pc;	
	struct pinctrl_state *ps_pwr_on;	
	struct pinctrl_state *ps_pwr_off;	
	struct pinctrl_state *ps_rst_low;	
	struct pinctrl_state *ps_rst_high;
	struct pinctrl_state *ps_rst_disable;	
	struct pinctrl_state *ps_irq_pulldown;		
	struct pinctrl_state *ps_irq_en;	
	struct pinctrl_state *ps_id_up;	
	struct pinctrl_state *ps_id_down;
	struct pinctrl_state *ps_cs_low;
	struct pinctrl_state *ps_cs_high;
	struct pinctrl_state *ps_mi_low;
	struct pinctrl_state *ps_mi_high;
	struct pinctrl_state *ps_mo_low;
	struct pinctrl_state *ps_mo_high;
	struct pinctrl_state *ps_ck_low;
	struct pinctrl_state *ps_ck_high;
	int irq_gpio;
	int irq_num;
};

#define FINGERPRINT_DEV_NAME "fingerprint_spi"
extern struct fingerprint fp;

#define get_gpio(name) fp.irq_##name

#define pin_select(name) (\
{\
	if((fp.pc != NULL) && (fp.ps_##name != NULL)){\
		pinctrl_select_state(fp.pc, fp.ps_##name);\
	}\
}\
)

#endif