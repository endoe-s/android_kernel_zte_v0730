/*file discription : otp*/
//#define OTP_DRV_LSC_SIZE 240

struct ov8856_otp_struct {
int flag;//bit[7]:info,bit[6]:wb,bit[5]:vcm,bit[4]:lenc
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
unsigned char lenc[240];
int checksum;
int VCM_start;
int VCM_end;
int VCM_dir;
};

//#define RG_Ratio_Typical 0x137
//#define BG_Ratio_Typical 0x121

int ov8856_read_otp(struct ov8856_otp_struct *otp_ptr);
int ov8856_apply_otp(struct ov8856_otp_struct *otp_ptr);
void ov8856_otp_cali(unsigned char writeid);

