#ifndef __AOLEXTRA_RING_BUFFER_H__
#define __AOLEXTRA_RING_BUFFER_H__

#include <linux/spinlock.h>
#include <linux/completion.h>

#define CORE_OP_SZ          8
#define MAX_FENCE_SIZE 10
#define MAX_WIFI_CLUSTER_SIZE 5
#define MAX_CELL_CLUSTER_SIZE 3

typedef struct wifi_info {
    unsigned char bssid[6];
    unsigned char ssid[32];
    int level;
    int band;
} WifiInfo;

struct wifi_fence_report_info {
    int id;
    uint32_t transitionEvent;
    WifiInfo cluster[MAX_WIFI_CLUSTER_SIZE];
};

typedef struct cell_info {
    uint8_t rat;
    uint8_t level;
    uint32_t cellId;
} CellInfo;

struct cell_fence_report_info {
    int id;
    uint32_t transitionEvent;
    CellInfo cluster[MAX_CELL_CLUSTER_SIZE];
};

struct wifi_fence_info {
    int id;
    int transitionTypes;
    int loiteringDelay;
    int notifyResponsiveness;
    WifiInfo cluster[MAX_WIFI_CLUSTER_SIZE];
};

struct cell_fence_info {
    int id;
    int transitionTypes;
    int loiteringDelay;
    int notifyResponsiveness;
    CellInfo cluster[MAX_CELL_CLUSTER_SIZE];
};

struct gnss_gfnc_location {
    uint32_t flags;
    double lat;
    double lng;
    double alt;
    float speed;
    float bearing;
    float h_accuracy;  //horizontal
    float v_accuracy;  //vertical
    float s_accuracy;  //speed
    float b_accuracy;  //bearing
    long long timestamp;    //may since boot
    uint32_t fix_type;
    long long utc_time; //Milliseconds since January 1, 1970
};

struct scp_loc_cnt {
    int total_req_cnt;
    int fix_cnt;
    int std_sta_cnt;
    int lp_sta_cnt;
    int near_fence_cnt;
    int medium_fence_cnt;
    int long_fence_cnt;
};

struct loc_duration {
    int gps_work_total_dur;
    int gps_work_std_dur;
    int gps_work_lp_dur;
    int gps_stationary_dur;
    int dr_work_total_dur;
    int dr_work_std_dur;
    int dr_work_lp_dur;
};

struct loc_accuracy {
    int mnl_fix_distr[7];
    int dr_fix_distr[7];
};

struct wifi_fence_stat {
    int scan_cnt[5];
    int event_cnt[5];
};

struct cell_fence_stat {
    //int scan_cnt[5];
    int event_cnt[5];
};

struct aol_statistics_data {
    struct scp_loc_cnt locCnt;
    struct loc_duration locDur;
    struct loc_accuracy locAcc;
    struct wifi_fence_stat wfStat;
    struct cell_fence_stat cfStat;
};

union aolextra_raw_data {
    struct wifi_fence_report_info wifi_fence_raw;
    struct cell_fence_report_info cell_fence_raw;
    struct gnss_gfnc_location loc_raw;
    struct aol_statistics_data stat_raw;
    struct wifi_fence_info wifiInfo2Scp;
    struct cell_fence_info cellInfo2Scp;
    int para2Scp;
};

struct aol_rb_data {
    unsigned int type;
    union aolextra_raw_data raw_data;
    struct completion comp;
};

struct aol_core_rb_q {
    uint32_t write;
    uint32_t read;
    uint32_t size;
    spinlock_t lock;
    struct aol_rb_data *queue[CORE_OP_SZ];
};

struct aol_core_rb {
    spinlock_t lock;
    struct aol_rb_data queue[CORE_OP_SZ];
    struct aol_core_rb_q freeQ;
    struct aol_core_rb_q activeQ;
};

int aol_core_rb_init(struct aol_core_rb *rb);
int aol_core_rb_deinit(struct aol_core_rb *rb);

struct aol_rb_data *aol_core_rb_pop_free(struct aol_core_rb *rb);
struct aol_rb_data *aol_core_rb_pop_active(struct aol_core_rb *rb);
void aol_core_rb_push_free(struct aol_core_rb *rb, struct aol_rb_data *data);
void aol_core_rb_push_active(struct aol_core_rb *rb, struct aol_rb_data *data);

int aol_core_rb_has_pending_data(struct aol_core_rb *rb);
#endif //__AOLEXTRA_RING_BUFFER_H__