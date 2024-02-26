/*
 *  server_example_config_file.c
 *
 *  This example shows how to use dynamic server data model with a configuration file.
 *
 *  - How to open and parse the model configuration file
 *  - How to access data attributes by object reference strings
 *  - How to access data attributes by short addresses
 *
 *  Note: If building with cmake the vmd-filestore folder containing the configuration file
 *  (model.cfg) has to be copied to the folder where the example is executed!
 *  The configuration file can be created from the SCL(ICD) file with the Java tool genconfig.jar
 *  that is included in the source distribution of libiec61580.
 *
 */

#include "iec61850_server.h"
#include "iec61850_config_file_parser.h"
#include "hal_thread.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include "control.h"

enum _modbus_status {IDLE,RCVING,RCVED,SENT,STMO,BUSY,ERROR};

typedef struct iedupdate_s {
    bool running;
    Thread hThread;
    struct LD_s* ld;
} iedupdate_t;

static int running = 0;
static IedServer iedServer = NULL;
static char ln_list[1024][64];
static int  ln_cnt = 0;
static char _iedName[256];
static IedModel* model=NULL;
static bool isSim = false;
static bool init_spc = false;

//modmst_t mst;
static iedupdate_t iedu;

//=====================================================

static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard() {
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard() {
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit() {
    unsigned char ch;
    int nread;

    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int _getch() {
    char ch;

    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

//=====================================================

int ret;
int fd;

#define MAX_BITS_STORAGE 65536
#define MAX_ANAS_STORAGE 65536

static uint8_t  bits[MAX_BITS_STORAGE/8];
static uint16_t anas[MAX_ANAS_STORAGE];
static uint8_t  bits_b[MAX_BITS_STORAGE/8];
static uint16_t anas_b[MAX_ANAS_STORAGE];

void Hex(char* msg, int len) {
    if(len) {
        for(int i=0; i<len; i++) {
            printf("%02X ", msg[i] &0xFF);
        }
        printf("\n");
    }
}

void Bin(char msg) {
    for(int j=0; j<8; j++) {
        printf("%d",(((msg<<j)&0x80)==0x80)?1:0);
        if(j % 4 == 3) printf(" ");
    }
    printf("  ");
}

ssize_t safe_write(int fd, const void *vptr, size_t n) {
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = vptr;
    nleft = n;

    while(nleft > 0) {
        if((nwritten = write(fd, ptr, nleft)) <= 0) {
            if(nwritten < 0 && errno == EINTR) {
                nwritten = 0;
            } else {
                return -1;
            }
        }
        nleft -= nwritten;
        ptr   += nwritten;
    }
    return(n);
}

ssize_t safe_read(int fd,void *vptr,size_t n) {
    size_t nleft;
    ssize_t nread;
    char *ptr;

    ptr=vptr;
    nleft=n;

    while(nleft > 0) {
        if((nread = read(fd,ptr,nleft)) < 0) {
            if(errno == EINTR)//被信号中断
                nread = 0;
            else
                return -1;
        } else if(nread == 0)
            break;
        nleft -= nread;
        ptr += nread;
    }
    return (n-nleft);
}

int uart_open(int fd,const char *pathname) {
    assert(pathname);

    fd = open(pathname,O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == -1) {
        perror("Open UART failed!");
        return -1;
    }

    if(fcntl(fd,F_SETFL,0) < 0) {
        fprintf(stderr,"fcntl failed!\n");
        return -1;
    }

    return fd;
}

int uart_set(int fd,int baude,int c_flow,int bits,char parity,int stop) {
    struct termios options;

    if(tcgetattr(fd,&options) < 0) {
        perror("tcgetattr error");
        return -1;
    }

    switch(baude) {
    case 4800:
        cfsetispeed(&options,B4800);
        cfsetospeed(&options,B4800);
        break;
    case 19200:
        cfsetispeed(&options,B19200);
        cfsetospeed(&options,B19200);
        break;
    case 38400:
        cfsetispeed(&options,B38400);
        cfsetospeed(&options,B38400);
        break;
    case 57600:
        cfsetispeed(&options,B57600);
        cfsetospeed(&options,B57600);
        break;
    case 115200:
        cfsetispeed(&options,B115200);
        cfsetospeed(&options,B115200);
        break;
    case 9600:
    default:
        cfsetispeed(&options,B9600);
        cfsetospeed(&options,B9600);
        break;
    }

    switch(c_flow) {
    case 1://进行硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2://进行软件流控制
        options.c_cflag |= IXON|IXOFF|IXANY;
        break;
    case 0://不进行流控制
    default:
        options.c_cflag &= ~CRTSCTS;
        break;
    }

    options.c_cflag &= ~CSIZE;//屏蔽其它标志位
    switch(bits) {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    default:
        options.c_cflag |= CS8;
        break;
    }

    /*设置校验位*/
    switch(parity){
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            break;
        default:
            options.c_cflag &= ~PARENB;
            break;
    }


    switch(stop) {
    case 2:
        options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
        break;
    default:
        options.c_cflag &= ~CSTOPB;
        break;
    }

    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    options.c_iflag &= ~( IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR );

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN]  = 1;//最少读取一个字符

    options.c_cflag |= CLOCAL;//保证程序不占用串口
    options.c_cflag |= CREAD;//允许程序可以从串口中读取数据

    tcflush(fd,TCIFLUSH);

    if(tcsetattr(fd,TCSANOW,&options) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

int uart_read(int fd,char *r_buf,size_t len,size_t tmo) {
    ssize_t cnt = 0;
    fd_set rfds;
    struct timeval time;

    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);

    time.tv_sec  = tmo / 1000000;
    time.tv_usec = tmo % 1000000;

    ret = select(fd+1,&rfds,NULL,NULL,&time);
    switch(ret) {
    case -1:
        printf("uart read error!\n");
        return -1;
    case 0:
        return -2;
    default:
        cnt = safe_read(fd,r_buf,len);
        if(cnt == -1) {
            fprintf(stderr,"read error!\n");
            return -1;
        }
        return cnt;
    }
}

int uart_write(int fd,const char *w_buf,size_t len) {
    ssize_t cnt = 0;

    cnt = safe_write(fd,w_buf,len);
    if(cnt == -1) {
        fprintf(stderr,"write error!\n");
        return -1;
    }

    return cnt;
}

int uart_wait_write(uint32_t tmo) {
    fd_set wfds;
    struct timeval time;

    FD_ZERO(&wfds);
    FD_SET(fd,&wfds);

    time.tv_sec  = tmo /1000000;
    time.tv_usec = tmo %1000000;

    return select(fd+1,NULL,&wfds,NULL,&time);
}

int uart_close(int fd) {
    assert(fd);
    close(fd);

    return 0;
}

uint16_t CRC16Table[] = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

uint16_t crc16(char *buf, uint16_t len) {
    uint16_t CRC = 0xFFFF;
    uint16_t  i;
    unsigned char ch;

    for (i = 0; i < len; i++) {
        ch = *buf++;
        CRC = CRC16Table[(ch ^ CRC) & 15] ^ (CRC >> 4);
        CRC = CRC16Table[((ch >> 4) ^ CRC) & 15] ^ (CRC >> 4);
    }
    return CRC;
}

/*
void setbit(char* buf, int idx, uint16_t status) {
    int inbyte = idx / 8;
    int bitInByte = idx % 8;
    printf("buf[%d].(%d) = %d\n",inbyte,bitInByte,status!=0);
    uint8_t b = ((buf[inbyte] & ((1<<bitInByte)))>0);
    if(b != (status>0)) {
        //状态发生变化，通知IEC61850更新
        if(status) {
            buf[inbyte] |=  (1<<bitInByte);
        } else {
            buf[inbyte] &= ~(1<<bitInByte);
        }
    }
}
// */
void setbit(char* d,const char* s, int di, int si)
{
	d[di/8] &= (~(1<<(di%8)));
	if (s[si/8] & (1<<(si%8))){
		d[di/8] |=(1<<(di%8));
	}
}


/**
int modslv(char* buf, int len) {
    uint16_t addr=0,tmpi=0,slen=0;
    addr=buf[2]<<8 | buf[3];
    slen=buf[4]<<8 | buf[5];

    switch(buf[1]) {
    case 1:
    case 2: //开关量
        slen = (slen+7)/8;
        buf[2]=slen;
        for(tmpi=0; tmpi<slen; tmpi++) {
            buf[tmpi+3]=(bits[addr/8+tmpi]>>(addr%8)) | (bits[addr/8+tmpi+1]<<(8-(addr%8)));
        }
        slen += 3;
        break;
    case 3:
    case 4: //模拟量
        for(tmpi=0; tmpi<slen; tmpi++) {
            buf[tmpi*2+3]=anas[addr+tmpi]>>8;
            buf[tmpi*2+4]=anas[addr+tmpi]&0xFF;
        }
        slen = slen*2;
        buf[2]=slen;
        slen += 3;
        break;
    case 5: //设置开关量
        setbit(bits,addr,slen);
        slen = 6;
        break;
    case 6: //设置摸拟量
        anas[addr] = slen;
        slen = 6;
        break;
    case 15://批量设置开关量
        break;
    case 16://批量设置摸拟量
        break;
    }

    tmpi=crc16(buf, slen);
    buf[slen++]=tmpi & 0xFF;
    buf[slen++]=tmpi>>8;

    return slen;
}
// */
//*
static char* modmst_rtn = 0;
static uint16_t modmst_adr=0;
int modmst(char* buf, uint8_t sa, uint8_t fn, uint16_t da, uint16_t lv, char* dptr) {
    int crc=0;
    int scnt=0;

    buf[0]=sa;
    buf[1]=fn;
    buf[2]=da>>8;
    buf[3]=da & 0xFF;
    buf[4]=lv>>8;
    buf[5]=lv & 0xFF;
    modmst_adr = da;

    scnt = 6;
    if(fn==1){
        modmst_rtn = (char*)&dptr[da/8];
    }else{
        modmst_rtn = (char*)&dptr[da<<1];
    }

    if(fn==15 || fn==16) {
        if(fn==15) {
            buf[scnt++]=(lv+7)/8;
        } else {
            buf[scnt++]=lv*2;
        }
        for(int i=0; i<buf[6]; i++) {
            buf[scnt++]=*(dptr+i);
        }
    }

    crc=crc16(buf,scnt);
    buf[scnt++]=crc & 0xFF;
    buf[scnt++]=crc>>8;
    return scnt;
}
// */

char* delstr(char* str, char* s2del) {
    int l=strlen(s2del);
    if(!str || !s2del) return str;
    char* s = NULL;
    if((s=strstr(str,s2del))!=NULL) {
        while(*s) {
            *s = *(s+l);
            s++;
        }
        *s = 0;
    }
    return str;
}

static char _buf4strleft[256];
char* strleft(const char* str, const char c, int nm) {
    int i=0;
    while(str[i]!=0) {
        if(str[i]==c)nm--;
        if(!nm)break;
        i++;
    }
    if (nm==0 && str[i]==c) {
        strncpy(_buf4strleft,str,i);
        _buf4strleft[i]=0;
        return _buf4strleft;
    }
    return NULL;
}

static char _buf4strright[256];
char* strright(const char* str, const char c) {
    char*s = strchr(str,c);
    return s+1;
}

struct LN_s {
    char* name;
    char* addr;
    struct LN_s* next;
};

struct LD_s {
    char* name;
    int kv_size;
    struct LN_s* Map;
    struct LD_s* next;
};

typedef struct LN_s ln_t;
typedef struct LD_s ld_t;

ld_t* LD_List=NULL;

struct LD_s* NewLD() {
    struct LD_s* ld = (struct LD_s*)malloc(sizeof(struct LD_s));
    ld->name = NULL;
    ld->kv_size = 0;
    ld->Map=NULL;
    ld->next=NULL;
    return ld;
}

struct LN_s* NewLN() {
    struct LN_s* ln = (struct LN_s*)malloc(sizeof(struct LN_s));
    ln->name = NULL;
    ln->addr = NULL;
    ln->next = NULL;
    return ln;
}

struct LD_s* GetLD_ex(struct LD_s* LD, char* s, bool create) {
    struct LD_s* ld = LD;
    while(ld) {
        if(!ld->name) {
            ld->name = strdup(s);
            return ld;
        }else if (strcmp(ld->name,s)==0) {
            return ld;
        }
        if(ld->next==NULL && create) {
            ld->next = NewLD();
            ld=ld->next;
            ld->name = strdup(s);
            ld->Map = NULL;
            ld->kv_size=0;
            ld->next = NULL;
            return ld;
        }
        ld=ld->next;
    }
    return NULL;
}

struct LD_s* GetLD(struct LD_s* LD, char* path) {
    char* ld = strleft(path,'/',1);
    return GetLD_ex(LD, ld, true);
}

struct LN_s* GetLN_ex(struct LD_s* LD, char* name, char* map, bool create) {
    struct LN_s* ln = LD->Map;
    if(LD->Map==NULL && create) {
        LD->Map = NewLN();
        LD->kv_size++;
        ln = LD->Map;
        ln->name = strdup(name);
        ln->addr = (map?strdup(map):NULL);
        ln->next = NULL;
        return ln;
    }
    while(ln) {
        if(ln->name) {
            if (strcmp(ln->name, name)==0) {
                if(!ln->addr && map)ln->addr=strdup(map);
                return ln;
            }
        }
        if(ln->next==NULL && create) {
            ln->next = NewLN();
            ln = ln->next;
            ln->name = strdup(name);
            ln->addr = (map?strdup(map):NULL);
            ln->next = NULL;
            LD->kv_size++;
            return ln;
        }
        ln=ln->next;
    }
    return NULL;
}

struct LN_s* GetLN(struct LD_s* LD, char* Path, char* map) {
    if(strchr(Path,'/')) {
        char* lds = strleft(Path,'/',1);
        struct LD_s* ld = GetLD_ex(LD,lds,true);
        if(ld) {
            char* ln = strright(Path,'/');
            if(ln) {
                return GetLN_ex(ld,ln,map,true);
            }
        } else {
            return NULL;
        }
    }
    return GetLN_ex(LD, Path,map,false);
}


#define MAX_BUF_SIZE 128
int datamap_init(struct LD_s* LD_List) {

    FILE *p = fopen("datamap.cfg", "r");
    if(p == NULL) {
        printf("the file is close!");
        return 0;
    }

    char LDName[MAX_BUF_SIZE];
    char LNPath[MAX_BUF_SIZE];
    char LNMap[MAX_BUF_SIZE];
    char LNType[MAX_BUF_SIZE];
    char LNComment[MAX_BUF_SIZE];

    struct LD_s* ld = NULL;
    struct LN_s* ln = NULL;

    char c;
    char i = 0;
    char idx=  0;

    while(!feof(p)) {
        c = getc(p);
        if (c == '\t' || c == '\n') {
            idx++;
            i=0;
            if(c == '\n') {
                if(!ld || (strcmp(ld->name, LDName)!=0)) {	//列表中的设备名与当前设备名不一致
                    ld = GetLD_ex(LD_List,LDName,false);	//取下列表中的一个设备名,不存在时不新建
                }

                if(ld) {
                    if(LNMap[strlen(LNMap)-1]=='v') {
                        delstr(LNPath,"ST.");delstr(LNPath,"MX.");
						delstr(LNMap,"DI$0$");delstr(LNMap,"AI$0$");delstr(LNMap, "$v");
                        ln = GetLN_ex(ld,LNPath, LNMap, false);
                    }
                    ld = NULL;
                }

                idx=0;
            }
            continue;
        }

        switch(idx) {
        case 1:
            LNPath[i++]=(c=='$')?'.':c;
            LNPath[i]=0;
            break;
        case 2:
            LNMap[i++]=c;
            LNMap[i]=0;
            break;
        case 3:
            LNType[i++]=c;
            LNType[i]=0;
            break;
        case 4:
            LNComment[i++]=c;
            LNComment[i]=0;
            break;
        default:
            LDName[i++]=c;
            LDName[i]=0;
            break;
        }
    }

    fclose(p);
    return 0;
}
//=================================================================

char * fc_type[] = {"ST","MX","SP","SV","CF","DC","SG","SE","SR","OR","BL","EX","CO","US","MS","RP","BR","LG","GO"};

char * model_type[] = {"LogicalDeviceModelType","LogicalNodeModelType","DataObjectModelType","DataAttributeModelType"};

char * iec_type[]= {
    "IEC61850_BOOLEAN",
    "IEC61850_INT8",
    "IEC61850_INT16",
    "IEC61850_INT32",
    "IEC61850_INT128",
    "IEC61850_INT64",
    "IEC61850_INT8U",
    "IEC61850_INT16U",
    "IEC61850_INT24U",
    "IEC61850_INT32U",
    "IEC61850_FLOAT32",
    "IEC61850_FLOAT64",
    "IEC61850_ENUMERATED",
    "IEC61850_OCTET_STRING_64",
    "IEC61850_OCTET_STRING_6",
    "IEC61850_OCTET_STRING_8",
    "IEC61850_VISIBLE_STRING_32",
    "IEC61850_VISIBLE_STRING_64",
    "IEC61850_VISIBLE_STRING_65",
    "IEC61850_VISIBLE_STRING_129",
    "IEC61850_VISIBLE_STRING_255",
    "IEC61850_UNICODE_STRING_255",
    "IEC61850_TIMESTAMP",
    "IEC61850_QUALITY",
    "IEC61850_CHECK",
    "IEC61850_CODEDENUM",
    "IEC61850_GENERIC_BITSTRING",
    "IEC61850_CONSTRUCTED",
    "IEC61850_ENTRY_TIME",
    "IEC61850_PHYCOMADDR",
    "IEC61850_CURRENCY",
    "IEC61850_OPTFLDS",
    "IEC61850_TRGOPS"
};

void sigint_handler(int signalId) {
    running = 0;
}

static void
goCbEventHandler(MmsGooseControlBlock goCb, int event, void* parameter) {
    printf("Access to GoCB: %s\n", MmsGooseControlBlock_getName(goCb));
    printf("         GoEna: %i\n", MmsGooseControlBlock_getGoEna(goCb));
}

char ___bufx___[128];
void
controlHandlerForBinaryOutput(ControlAction action, void* parameter, MmsValue* value) {
    IedServer_lockDataModel(iedServer);
    ModelNode* p = (ModelNode*)parameter;
    if (p) {
        ModelNode* sp = p;
        ModelNode* ssp=NULL;
        ModelNode* sssp=NULL;
        ModelNode* ssssp=NULL;
        ssp = sp->parent;
        if (ssp && ssp->name) {
            sssp = ssp->parent;
            if (sssp && sssp->modelType!=LogicalDeviceModelType) {
                ssssp = sssp->parent;
                if(ssssp && ssssp->modelType!=LogicalDeviceModelType) {
                    printf("ssssp: %s\n",model_type[ssssp->modelType]);
                } else {
                    sprintf(___bufx___,"%s%s/%s.%s.%s",_iedName,ssssp->name,sssp->name,ssp->name,sp->name);
                }
            } else {
                sprintf(___bufx___,"%s%s/%s.%s",_iedName,sssp->name,ssp->name,sp->name);

            }
        } else {
            sprintf(___bufx___,"%s%s/%s",_iedName,ssp->name,sp->name);
        }

        DataAttribute* s_val = (DataAttribute*)ModelNode_getChild(p,"stVal");
        DataAttribute* s_t = (DataAttribute*)ModelNode_getChild(p,"t");
        DataAttribute* s_q = (DataAttribute*)ModelNode_getChild(p,"q");
        uint64_t timestamp = Hal_getTimeInMs();
        if(s_val) {
            //printf("controlHandlerForBinaryOutput...[ %s.%s ][ %s ]\n", ___bufx___, s_val->name,MmsValue_getBoolean(value)?"true":"false");
            //Direct send control command to modbus...
            //modmst(&mst,02,5,0,MmsValue_getBoolean(value),NULL);

            if(s_t) {
                IedServer_updateUTCTimeAttributeValue(iedServer, s_t, timestamp);
            }
            if(s_q) {
                IedServer_updateUTCTimeAttributeValue(iedServer, s_q, QUALITY_VALIDITY_INVALID);
            }
            IedServer_updateAttributeValue(iedServer, s_val, value);

        }
    }
    IedServer_unlockDataModel(iedServer);
}


void FetchDA(const char* pathString) {
    DataAttribute* da = (DataAttribute*)
                        IedModel_getModelNodeByObjectReference(model, pathString);
    if(!da) return;
    if(da->fc == IEC61850_FC_ST || da->fc == IEC61850_FC_MX) {
        int len = strlen(pathString);
        //printf("%-48s\n", pathString);
        if (pathString[len-1]!='q' && pathString[len-1]!='t') {
            ln_t* ln = GetLN(LD_List, (char*)pathString, NULL);
        }
    }

    if (da->fc == IEC61850_FC_CO) return;
    if (!init_spc && strstr(pathString,"SPC")!=NULL &&  da->fc == IEC61850_FC_CF) {
        char* s = strleft(pathString,'.',2);
        ModelNode* mn = IedModel_getModelNodeByObjectReference(model, s);
        if (mn) {
            IedServer_setControlHandler(iedServer, (DataObject*)mn, (ControlHandler) controlHandlerForBinaryOutput,mn);
            //printf("|================== _set control handler: [ %s ] ==================\n", s);
        }

        return; //break function execution
    }
}

void ViewDA(ModelNode* da, const char* pathString) {
    char buf[256];
    ModelNode* pda = da;
    while(pda) {
        DataAttribute* da_da = (DataAttribute*)pda;
        sprintf(buf, "%s.%s", pathString, da_da->name);
        if (da_da->firstChild) {
            ViewDA(pda->firstChild, buf);
        } else {
            FetchDA(buf);
        }
        pda = pda->sibling;
    }
}

void ViewDS(IedModel* model) {
    DataSet* ds = model->dataSets;
    DataSetEntry* dse = NULL;
    char buf[65];
    while(ds) {
        dse = ds->fcdas;
        while(dse) {
            sprintf(buf,"%s%s/%s", model->name, dse->logicalDeviceName, dse->variableName);
            StringUtils_replace(buf, '$', '.');
            delstr(buf,"ST.");
            delstr(buf,"MX.");
            ModelNode* mn = IedModel_getModelNodeByObjectReference(model, buf);
            if(mn) {
                ViewDA(mn->firstChild,buf);
            }
            dse = dse->sibling;
        }
        ds = ds->sibling;
    }
}

void update_data(struct LD_s* LD) {
    bool isSupported;
    char buf[256];
    static uint32_t ind = 0;
    struct LN_s* ln = LD->Map;

    IedServer_lockDataModel(iedServer);
    while(ln) {
        sprintf(buf,"%s/%s",LD->name, ln->name);
        DataAttribute* da = (DataAttribute *) IedModel_getModelNodeByObjectReference(model, (const char*)buf);
        if (da == NULL) {
            printf("%s can't found!\n", buf);
            continue;
        }

        MmsValue* value;
        isSupported = false;
        if(ln->addr) {
            int address = atoi(ln->addr);
            switch(da->type) {
                case IEC61850_BOOLEAN: {
                    bool temp  = ((bits[address/8]>>(address%8))&1)>0;
                    bool temp_b= ((bits_b[address/8]>>(address%8))&1)>0;
                    if(temp!=temp_b){
                        value = MmsValue_newBoolean(temp);
                        if(temp){
                            bits_b[address/8] |=  (1<<(address % 8));
                        }else{
                            bits_b[address/8] &= ~(1<<(address % 8));
                        }

                        isSupported = true;
                    }
                    break;
                }
                case IEC61850_INT16:
                {
                    int16_t temp  = *(int16_t*)&anas[address];
                    int16_t temp_b= *(int16_t*)&anas_b[address];
                    if(temp!=temp_b){
                        value = MmsValue_newIntegerFromInt16(temp);
                        *(int16_t*)&anas_b[address] = *(int16_t*)&anas[address];
                        isSupported = true;
                    }
                    break;
                }
                case IEC61850_INT16U:{
                    uint16_t temp  = *(uint16_t*)&anas[address];
                    uint16_t temp_b= *(uint16_t*)&anas_b[address];
                    if(temp!=temp_b){
                        value = MmsValue_newUnsignedFromUint32(temp);
                        *(uint16_t*)&anas_b[address] = *(uint16_t*)&anas[address];
                        isSupported = true;
                    }

                    break;
                }
                case IEC61850_INT32:{
                    int32_t temp = *(int32_t*)&anas[address];
                    int32_t temp_b= *(int32_t*)&anas_b[address];
                    if(temp!=temp_b){
                        value = MmsValue_newIntegerFromInt32(temp);
                        *(int32_t*)&anas_b[address] = *(int32_t*)&anas[address];
                        isSupported = true;
                    }
                    break;
                }
                case IEC61850_INT32U:{
                    uint32_t temp  = *(uint32_t*)&anas[address];
                    uint32_t temp_b= *(uint32_t*)&anas_b[address];
                    if(temp!=temp_b){
                        value = MmsValue_newUnsignedFromUint32(temp);
                        *(uint32_t*)&anas_b[address] = *(uint32_t*)&anas[address];
                        isSupported = true;
                    }
                    break;
                }
                case IEC61850_FLOAT32:{
                    float temp  = *(float*)&anas[address];
                    float temp_b= *(float*)&anas_b[address];
                    if(temp-temp_b>0.0001 || temp_b-temp>0.0001){
                        value = MmsValue_newFloat(temp);
                        *(float*)&anas_b[address]=*(float*)&anas[address];
                        isSupported = true;
                    }

                    break;
                }
                case IEC61850_FLOAT64: {
                    double temp  = *(double*)&anas[address];
                    double temp_b= *(double*)&anas_b[address];
                    if(temp-temp_b>0.0001 || temp_b-temp>0.0001){
                        value = MmsValue_newDouble(temp);
                        *(double*)&anas_b[address] = *(double*)&anas[address];
                        isSupported = true;
                    }
                    break;
                }
                default: {
                    isSupported = false;
                    break;
                }
            }

            if (isSupported) {
                //printf("[ %5d ] update data ...\n", ind++);
                IedServer_updateAttributeValue(iedServer, da, value);
            }
        }
        ln = ln->next;
    }

    IedServer_unlockDataModel(iedServer);
}

//******modbus config file**********
typedef struct _Buffer_s{
    char ch;
    int pos;
    int curpos;
    int length;
    char* buf;
}Buffer_t;

typedef struct modmst_cfg_s{
    unsigned char sa;
    unsigned char fn;
    unsigned short adr;
	unsigned short lv;
    int dptr;
	struct modmst_cfg_s* next;
}modmst_cfg_t;

Buffer_t* InitBuffer_t(){
	Buffer_t* buf = (Buffer_t*)calloc(1,sizeof(Buffer_t));
	if(buf){
		buf->curpos  = 0;
		buf->pos	 = 0;
		buf->length	 = 0 ;
		buf->buf	 = 0;
		buf->ch		 = -1;
	}
	return buf;
}

bool ReadFile(char* file, Buffer_t* buf){

    FILE *fp = fopen(file, "r");
    if(fp == NULL)
    {
        printf("the file [ %s ] isn't exact opened!\n", file);
        return false;
    }
    fseek(fp, 0, SEEK_END);
    buf->length = ftell(fp);
    buf->buf = (char*)malloc(buf->length+1);
    fseek(fp, 0, SEEK_SET);
    fread(buf->buf,buf->length,1,fp);
	buf->buf[buf->length]=0;
    fclose(fp);
	return true;
}

char nextch(Buffer_t* buf){
    if(buf->curpos < buf->length){
        return buf->buf[buf->curpos++];
    }
    printf("nextch\n");
    return -1;
}

char peekch_ex(Buffer_t* buf, int n){
    if(buf->curpos < buf->length){
        return buf->buf[buf->curpos+n];
    }
    return -1;
}

char peekch(Buffer_t* buf){
	return peekch_ex(buf,0);
}

bool In(unsigned char ch, char* buf){
	int len = strlen(buf);
	for(int i=0;i<len;i++){
		if(ch == buf[i]) 
			return true;
	}
	return false;
}

char* upstr(char* s){
	char* p = s;
	while(*p){
		if(*p>='a' && *p<='z'){
			*p=*p-'a'+'A';
		}
		p++;
	}
	return s;
}

long htol(char* s){
	long l = 0;
	int slen = strlen(s);
    int tmpc = 0;
    if(s[0]=='0' && (s[1]=='x' || s[1]=='X')){
        for(int i=2;i<slen;i++){
            if(s[i]>='a' && s[i]<='f'){
                tmpc = s[i]-'a'+10; 
            }else if(s[i]>='A' && s[i]<='F'){ 
                tmpc = s[i]-'A'+10;
            }else{ 
                tmpc = s[i]-'0'; 
            }
            l = l*16 + tmpc;
        }
    }else{
        l = (long)atol(s);
    }

    return l;
}

bool isEnd(Buffer_t* buf){
	return peekch(buf)==0xFF || peekch(buf)==-1;
}

bool isNL(Buffer_t* buf){
	return In(peekch(buf),"\r\n");
}

bool wantNL(Buffer_t* buf){
	char ch;
	while(!isNL(buf) && !isEnd(buf)){
		ch = nextch(buf);
	}
	ch = nextch(buf);
	return isEnd(buf) || isNL(buf);
}

bool isNumber(Buffer_t* buf){
    return (In(peekch(buf),"0123456789"));
}

bool wantNumber(Buffer_t* buf){
    while(!isNumber(buf) && !isEnd(buf)){
		nextch(buf);
	}
	return isNumber(buf);
}

long number(Buffer_t* buf){
	int i=0;
	char bf[16]={0};
    
    wantNumber(buf);
    while(In(peekch(buf),"0123456789xabcdefABCDEF")){
        bf[i++]=nextch(buf);
        bf[i]=0;
    }
	return htol(bf);
}

/******************************************
SA FN DA   LENGTH/DATA ...
02 01 0    10
FA 03 1000 128
01 05 0    0xFF00
// ***************************************/
modmst_cfg_t* Modbus_Config_Parse(Buffer_t* buf){//, unsigned short* ai, unsigned char* di){
    unsigned char idx=0;
    char bf[256];
    int bflen=0;
	long n = 0;
	unsigned short da;
	unsigned short lv;
	modmst_cfg_t* mst_s = 0;
	modmst_cfg_t* p_mst_s = 0;
    while(!isEnd(buf) && wantNumber(buf)){
		if(!mst_s){
			mst_s = (modmst_cfg_t*)malloc(sizeof(modmst_cfg_t));
			p_mst_s=mst_s;
		}else{
			p_mst_s->next = (modmst_cfg_t*)malloc(sizeof(modmst_cfg_t));
			p_mst_s=p_mst_s->next;
		}
		p_mst_s->dptr = 0;
        p_mst_s->next=0;

		p_mst_s->sa=number(buf)&0xFF;
		p_mst_s->fn=number(buf)&0xFF;

		p_mst_s->adr = number(buf);
		p_mst_s->lv = (unsigned short)number(buf);

        p_mst_s->dptr = isNL(buf)?p_mst_s->adr:(unsigned short)number(buf);
		if(!wantNL(buf))printf("It isn't a new line! but wanted!\n");
    }
    return mst_s;
}

modmst_cfg_t* get_modmst_cfg(char* file){
    Buffer_t* buf=InitBuffer_t();
    if(ReadFile(file,buf)){
        modmst_cfg_t* t = Modbus_Config_Parse(buf);
        free(buf->buf);
        free(buf);
        return t;
    }
    return NULL;
}

int
main(int argc, char** argv) {
    bool dispinfo = true;

    int tcpPort = 102;

    srand((unsigned)time(NULL));

    /* parse the configuration file and create the data model */
    model = ConfigFileParser_createModelFromConfigFileEx("model.cfg");

    if (model == NULL) {
        printf("Error parsing config file!\n");
        return 1;
    }

    iedServer = IedServer_create(model);

    if (argc>1 && strcmp(argv[1],"run")==0){
        dispinfo=false;
    }

    IedServer_setGoCBHandler(iedServer, goCbEventHandler, NULL);

    IedServer_start(iedServer, tcpPort);

    if(dispinfo) printf("=========================================================================================\n");

    LD_List = NewLD();
    ViewDS(model);				//将icd模型文件的路径输入到LD_List中
    ld_t* ld = LD_List;
    datamap_init(ld);		//将LD_List列表中的路径对应到datamap.cfg指定的地址中.
    init_spc = true;

    if (!IedServer_isRunning(iedServer)) {
        printf("Starting server failed! Exit.\n");
        IedServer_destroy(iedServer);
        exit(-1);
    }

    /* Start GOOSE publishing */
    IedServer_enableGoosePublishing(iedServer);

    running = 1;

    signal(SIGINT, sigint_handler);


    fd = uart_open(fd,"/dev/ttyS0");  // 串口号/dev/ttySn,USB口号/dev/ttyUSBn
    if(fd == -1) {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,9600,0,8,'N',1) == -1) {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

    if(dispinfo){
        printf("IEC61850 server initialized ! \n");
        printf("running ......\n");
    }

    init_keyboard();

    uint16_t scnt=0;
    uint16_t rcnt=0;
    uint8_t status = IDLE;
    size_t tmo =  500000;
    uint8_t queue = 0;
    uint16_t retry = 0;
    struct timeval time;

    char r_ch[2];
    char buf[1024];
    char sbuf[264];

    bzero(r_ch,2);
    bzero(buf,1024);
    bzero(sbuf,264);

    time.tv_sec  = tmo / 1000000;
    time.tv_usec = tmo % 1000000;

    modmst_cfg_t* p_mst_s = 0;
    modmst_cfg_t* mst_s = get_modmst_cfg("modbus.cfg");

    p_mst_s = mst_s;

    while (running) {
        if(_kbhit()) {
            if (_getch()=='q') {
                running = false;
            }
        }

        ret = uart_read(fd,r_ch,1,tmo);

        if(ret == -1) {
            printf("uart read error!\n");
            status = IDLE;
        }

        if(ret > 0) {
            if(rcnt>0 && rcnt<260) {
                buf[rcnt++] = r_ch[0];
            }else if(rcnt==0) {
                if(r_ch[0]==0x02) {
                    buf[0] = r_ch[0];
                    rcnt=1;
                    tmo = 10000;
                }
            }
        }

        if(ret==-2) {
            if(p_mst_s && crc16(buf,rcnt)==0) {
                if(dispinfo){ printf("[%-3d]<<<", rcnt);Hex(buf,rcnt); }
                IedServer_lockDataModel(iedServer);
                
                switch(buf[1]) {
                    case 1:
                    case 2:
                        for(uint16_t i=0; i < p_mst_s->lv ; i++) {
                            setbit((char*)bits,(char*)&buf[3],p_mst_s->adr+i,i);
                        }
                        break;
                case 3:
                case 4:
                    for(int i=0; i < (buf[2]>>1); i++) {
                        anas[p_mst_s->dptr+i] = (buf[3+(i<<1)]<<8) | (buf[4+(i<<1)] & 0xFF);
                    }
                    break;
                }
                IedServer_unlockDataModel(iedServer);

                if(ld){
                     update_data(ld);
                     ld=ld->next;
                }else{
                    ld = LD_List;
                }

            }

            rcnt=0;
            status = IDLE;
        }

        if(status == IDLE) {
            tmo = 1000000;
            
            if(!p_mst_s)continue;
            
            p_mst_s=p_mst_s->next;
            if(!p_mst_s) p_mst_s = mst_s;

            if(p_mst_s->fn==1 || p_mst_s->fn==2 || p_mst_s->fn==15){
                scnt = modmst(sbuf,p_mst_s->sa,p_mst_s->fn,p_mst_s->adr,p_mst_s->lv,
                            (char*)&bits[p_mst_s->dptr/8]);
            }else{ // if(p_mst_s->fn==3 || p_mst_s->fn==4 || p_mst_s->fn==16)
                scnt = modmst(sbuf,p_mst_s->sa,p_mst_s->fn,p_mst_s->adr,p_mst_s->lv,
                            (char*)&anas[p_mst_s->dptr]);
            }
                
            if(scnt){
                if(dispinfo){printf("[%-3d]>>>", scnt);Hex(sbuf, scnt);}
                if(uart_write(fd,sbuf, scnt)==scnt){
                    status = SENT;
                }
            }
        }
        Thread_sleep(1);
        
    }

    scnt = 0;
    for(int i=0;i<16540;i++){
        if(anas[i]>0){
            printf("%5d",anas[i]);
            if(scnt%16==15)printf("\n");
            scnt++;
        }
    }
    printf("\n");
    
    close_keyboard();

    ret = uart_close(fd);
    if(ret == -1) {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    IedServer_stop(iedServer);
    IedServer_destroy(iedServer);
    IedModel_destroy(model);
    
    return 0;
} /* main() */

