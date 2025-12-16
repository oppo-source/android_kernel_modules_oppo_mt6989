// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include "../oplus/oplus_display_mtk_debug.h"
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus_24921_novatek_mtk_data_hw_roundedpattern_l.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus_24921_novatek_mtk_data_hw_roundedpattern_r.h"
#endif

#include "ktz8866.h"

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include "../mediatek/mediatek_v2/mtk_disp_notify.h"
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_CS_ON  0x19
#endif

#define MAX_NORMAL_BRIGHTNESS           2047
#define PHYSICAL_WIDTH_MM               240
#define PHYSICAL_HEIGHT_MM              171

#define REGFLAG_CMD                     0xFFFA
#define REGFLAG_DELAY                   0xFFFC
#define REGFLAG_UDELAY                  0xFFFB
#define REGFLAG_END_OF_TABLE            0xFFFD

static int g_fps_current = 120;

extern unsigned long esd_flag;
extern bool g_dp_support;
static bool is_pd_with_guesture = false;
extern unsigned int oplus_enhance_mipi_strength;

extern unsigned int silence_mode;
extern bool g_shutdown;

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif

#define  M_DELAY(n) usleep_range(n*1000, n*1000+100)
#define  U_DELAY(n) usleep_range(n, n+10)

#define HFP_144HZ (54)
#define HFP_90_50_48 (169)
#define HFP_120_60_30 (170)
#define HSA (28)
#define HBP (26)
#define HBP_90HZ (120)
#define VFP (26)
#define VFP_90HZ (768)
#define VFP_50HZ (3140)
#define VFP_48HZ (3360)
#define VFP_144HZ (26)
#define VFP_60HZ (2250)
#define VFP_120HZ (26)
#define VFP_30HZ (6698)
#define VSA (2)
#define VBP (196)
#define VAC_FHD (2000)
#define HAC_FHD (2800)

unsigned int level_backup = 0;
extern unsigned int oplus_display_brightness;
static bool lcd_bl_set_led_flag = false;
static const struct drm_display_mode hand_display_mode_90hz = {
	//fhd_sdc_90_mode
	.clock = 835522, //3130*2966*90 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_90_50_48,
	.hsync_end = HAC_FHD + HFP_90_50_48 + HSA,
	.htotal = HAC_FHD + HFP_90_50_48 + HSA + HBP_90HZ,//3120
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_90HZ,
	.vsync_end = VAC_FHD + VFP_90HZ + VSA,
	.vtotal = VAC_FHD + VFP_90HZ + VSA + VBP, //2966
	.hskew = 1,
};

static const struct drm_display_mode hand_display_mode_50hz = {
	//fhd_sdc_50_mode
	.clock = 835522, //3024*8896*30 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_90_50_48,
	.hsync_end = HAC_FHD + HFP_90_50_48 + HSA,
	.htotal = HAC_FHD + HFP_90_50_48 + HSA + HBP_90HZ, //3120
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_50HZ,
	.vsync_end = VAC_FHD + VFP_50HZ + VSA,
	.vtotal = VAC_FHD + VFP_50HZ + VSA + VBP, //
	.hskew = 1,
};

static const struct drm_display_mode hand_display_mode_48hz = {
	//fhd_sdc_48_mode
	.clock = 835522, //3024*8896*30 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_90_50_48,
	.hsync_end = HAC_FHD + HFP_90_50_48 + HSA,
	.htotal = HAC_FHD + HFP_90_50_48 + HSA + HBP_90HZ, //3120
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_48HZ,
	.vsync_end = VAC_FHD + VFP_48HZ + VSA,
	.vtotal = VAC_FHD + VFP_48HZ + VSA + VBP, //
	.hskew = 1,
};


static const struct drm_display_mode hand_display_mode_144hz = {
	//fhd_sdc_144_mode
	.clock = 931104, //2908*2224*144  htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_144HZ,
	.hsync_end = HAC_FHD + HFP_144HZ + HSA,
	.htotal = HAC_FHD + HFP_144HZ + HSA + HBP,//2908
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_144HZ,
	.vsync_end = VAC_FHD + VFP_144HZ + VSA,
	.vtotal = VAC_FHD + VFP_144HZ + VSA + VBP,//2224
	.hskew = 1,
};

static const struct drm_display_mode pan_display_mode_60hz = {
	//fhd_sdc_60_mode
	.clock = 807045, //3024*4448*60 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_120_60_30,
	.hsync_end = HAC_FHD + HFP_120_60_30 + HSA,
	.htotal = HAC_FHD + HFP_120_60_30 + HSA + HBP, //3024
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_60HZ,
	.vsync_end = VAC_FHD + VFP_60HZ + VSA,
	.vtotal = VAC_FHD + VFP_60HZ + VSA + VBP, //4448
	.hskew = 1,
};
static const struct drm_display_mode pan_display_mode_120hz = {
	//fhd_sdc_120_mode
	.clock = 807045, //2894*2224*120 htotal*vtotal*fps  3024*2224*120=807045
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_120_60_30,
	.hsync_end = HAC_FHD + HFP_120_60_30 + HSA,
	.htotal = HAC_FHD + HFP_120_60_30 + HSA + HBP, //3024
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_120HZ,
	.vsync_end = VAC_FHD + VFP_120HZ + VSA,
	.vtotal = VAC_FHD + VFP_120HZ + VSA + VBP, //2224
	.hskew = 1,
};

static const struct drm_display_mode pan_display_mode_30hz = {
	//fhd_sdc_30_mode
	.clock = 807045, //3024*8896*30 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_120_60_30,
	.hsync_end = HAC_FHD + HFP_120_60_30 + HSA,
	.htotal = HAC_FHD + HFP_120_60_30 + HSA + HBP, //3024
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_30HZ,
	.vsync_end = VAC_FHD + VFP_30HZ + VSA,
	.vtotal = VAC_FHD + VFP_30HZ + VSA + VBP, //8896
	.hskew = 1,
};

enum CMD_TYPE {
	TYPE_GERNERIC,
	TYPE_PPS,
	TYPE_DCS,
	TYPE_MDELAY,
	TYPE_UDELAY
};

struct LCM_setting_table_by_type {
	unsigned int type;
	unsigned char count;
	unsigned char para_list[64];
};

//void lcdinfo_notify(unsigned long val, void *v);
// struct lcm_pmic_info {
// 	struct regulator *reg_vrf18_aif;
// };

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *master_esd_gpio;
	struct gpio_desc *slave_esd_gpio;
	struct gpio_desc *bias_enp_en;
	struct gpio_desc *bias_enn_en;
	struct gpio_desc *bias_en;
	struct regulator *reg_vrf18_aif;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	int error;
};

/* rc_buf_thresh will right shift 6bits (which means the values here will be divided by 64)
 * when setting to PPS8~PPS11 registers in dsc_config() function, so the original values need
 * left sihft 6bit (which means the original values are multiplied by 64), so that PPS8~PPS11
 * registers can get right setting
 */
static unsigned int rc_buf_thresh[14] = {
//The original values VS values multiplied by 64
//14, 28,  42,   56,   70,   84,   98,   105,  112,  119,  121,  123,  125,  126
896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
unsigned int range_min_qp[15] = {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 9, 12};
unsigned int range_max_qp[15] = {4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13};
int range_bpg_ofs[15] = {2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

static unsigned int backlight_mapping_buf[] = {
	   0,  165,  175,  178,  182,  186,  190,  194,  200,  205,  210,  213,  216,  220,  225,  230,  235,  240,  245,  250,
	 255,  260,  265,  270,  275,  280,  285,  297,  305,  311,  317,  323,  329,  335,  341,  350,  360,  370,  375,  383,
	 391,  400,  408,  415,  425,  433,  443,  448,  453,  463,  473,  483,  493,  503,  514,  524,  534,  545,  555,  565,
	 576,  580,  586,  593,  600,  606,  614,  622,  630,  637,  644,  650,  656,  661,  667,  670,  673,  674,  675,  676,
	 677,  678,  679,  680,  681,  682,  686,  690,  695,  701,  704,  705,  706,  708,  709,  712,  713,  714,  716,  717,
	 720,  721,  722,  724,  726,  727,  729,  730,  731,  734,  735,  736,  738,  739,  741,  743,  744,  745,  747,  749,
	 750,  752,  753,  754,  755,  758,  759,  760,  761,  764,  765,  766,  767,  769,  770,  772,  773,  775,  776,  777,
	 778,  781,  782,  783,  784,  785,  788,  789,  790,  791,  792,  795,  796,  797,  798,  799,  800,  803,  804,  805,
	 807,  808,  810,  812,  813,  814,  815,  816,  818,  819,  821,  822,  823,  824,  826,  827,  828,  829,  830,  833,
	 834,  835,  836,  837,  838,  839,  841,  842,  843,  844,  846,  847,  849,  850,  851,  852,  853,  854,  856,  857,
	 858,  859,  860,  861,  862,  864,  865,  866,  867,  868,  869,  871,  873,  874,  875,  876,  877,  879,  880,  881,
	 882,  883,  884,  885,  887,  888,  889,  890,  891,  892,  893,  895,  896,  897,  898,  899,  899,  900,  902,  903,
	 904,  905,  906,  907,  908,  910,  911,  912,  913,  914,  915,  916,  918,  919,  920,  921,  922,  923,  923,  925,
	 926,  927,  928,  929,  930,  931,  933,  934,  935,  936,  937,  937,  938,  939,  941,  942,  943,  944,  945,  946,
	 948,  949,  949,  950,  951,  952,  953,  954,  956,  957,  958,  958,  959,  960,  961,  962,  964,  965,  966,  966,
	 967,  968,  969,  971,  972,  973,  974,  974,  975,  976,  977,  979,  980,  981,  981,  982,  983,  984,  985,  987,
	 988,  988,  989,  990,  991,  992,  994,  994,  995,  996,  997,  998,  999,  999, 1000, 1002, 1003, 1004, 1005, 1006,
	1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023, 1024, 1025, 1026,
	1027, 1028, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1037, 1038, 1039, 1040, 1041, 1042, 1043, 1044, 1045, 1046,
	1047, 1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066,
	1067, 1068, 1069, 1070, 1071, 1072, 1073, 1074, 1075, 1075, 1076, 1077, 1078, 1079, 1080, 1081, 1082, 1083, 1084, 1085,
	1086, 1087, 1088, 1089, 1090, 1091, 1092, 1093, 1094, 1095, 1096, 1097, 1097, 1098, 1099, 1100, 1101, 1102, 1103, 1104,
	1104, 1105, 1106, 1106, 1107, 1108, 1109, 1110, 1111, 1112, 1112, 1113, 1114, 1115, 1116, 1117, 1118, 1118, 1119, 1120,
	1120, 1121, 1122, 1123, 1124, 1125, 1126, 1126, 1127, 1128, 1128, 1129, 1130, 1131, 1132, 1133, 1134, 1134, 1135, 1136,
	1136, 1137, 1138, 1139, 1140, 1141, 1142, 1142, 1143, 1144, 1144, 1145, 1146, 1147, 1148, 1149, 1149, 1150, 1151, 1151,
	1152, 1153, 1154, 1155, 1156, 1156, 1157, 1158, 1159, 1159, 1160, 1161, 1162, 1163, 1164, 1164, 1165, 1166, 1166, 1167,
	1168, 1168, 1169, 1170, 1171, 1172, 1173, 1173, 1174, 1175, 1175, 1176, 1177, 1178, 1179, 1179, 1180, 1181, 1181, 1182,
	1183, 1183, 1184, 1185, 1186, 1187, 1188, 1188, 1189, 1190, 1190, 1191, 1192, 1193, 1194, 1195, 1195, 1196, 1196, 1197,
	1198, 1198, 1199, 1200, 1201, 1202, 1203, 1203, 1204, 1205, 1205, 1206, 1206, 1207, 1208, 1209, 1210, 1211, 1211, 1212,
	1213, 1213, 1214, 1214, 1215, 1216, 1217, 1218, 1219, 1219, 1220, 1221, 1221, 1222, 1223, 1224, 1225, 1225, 1226, 1227,
	1227, 1228, 1228, 1229, 1230, 1231, 1232, 1233, 1233, 1234, 1234, 1235, 1236, 1236, 1237, 1238, 1238, 1239, 1240, 1241,
	1242, 1242, 1243, 1243, 1244, 1245, 1246, 1247, 1248, 1248, 1249, 1249, 1250, 1251, 1251, 1252, 1252, 1253, 1254, 1255,
	1256, 1256, 1257, 1258, 1258, 1259, 1260, 1260, 1261, 1262, 1263, 1264, 1264, 1265, 1265, 1266, 1267, 1267, 1268, 1269,
	1270, 1271, 1271, 1272, 1272, 1273, 1274, 1274, 1275, 1275, 1276, 1277, 1278, 1279, 1279, 1280, 1281, 1281, 1282, 1282,
	1283, 1284, 1284, 1285, 1286, 1287, 1288, 1288, 1289, 1289, 1290, 1290, 1291, 1292, 1293, 1294, 1294, 1295, 1296, 1296,
	1297, 1297, 1298, 1299, 1299, 1300, 1301, 1302, 1302, 1303, 1304, 1304, 1305, 1305, 1306, 1307, 1307, 1308, 1309, 1310,
	1310, 1311, 1312, 1312, 1313, 1313, 1314, 1315, 1316, 1317, 1317, 1318, 1318, 1319, 1320, 1320, 1321, 1321, 1322, 1323,
	1324, 1325, 1325, 1326, 1326, 1327, 1327, 1328, 1329, 1329, 1330, 1331, 1332, 1332, 1333, 1334, 1334, 1335, 1335, 1336,
	1336, 1337, 1338, 1339, 1340, 1340, 1341, 1341, 1342, 1343, 1343, 1344, 1344, 1345, 1345, 1346, 1347, 1348, 1349, 1349,
	1350, 1350, 1351, 1351, 1352, 1353, 1353, 1354, 1355, 1356, 1356, 1357, 1357, 1358, 1359, 1359, 1360, 1360, 1361, 1362,
	1363, 1363, 1364, 1365, 1365, 1366, 1366, 1367, 1367, 1368, 1369, 1370, 1371, 1371, 1372, 1372, 1373, 1373, 1374, 1374,
	1375, 1376, 1376, 1377, 1378, 1379, 1379, 1380, 1380, 1381, 1382, 1382, 1383, 1383, 1384, 1385, 1386, 1386, 1387, 1387,
	1388, 1389, 1389, 1390, 1390, 1391, 1391, 1392, 1393, 1394, 1394, 1395, 1396, 1396, 1397, 1397, 1398, 1398, 1409, 1400,
	1401, 1401, 1402, 1403, 1403, 1404, 1404, 1405, 1405, 1406, 1406, 1407, 1408, 1409, 1409, 1410, 1411, 1411, 1412, 1412,
	1413, 1413, 1414, 1414, 1415, 1416, 1417, 1417, 1418, 1419, 1419, 1420, 1420, 1421, 1421, 1422, 1423, 1424, 1424, 1425,
	1425, 1426, 1427, 1427, 1428, 1428, 1429, 1429, 1430, 1431, 1432, 1432, 1433, 1433, 1434, 1434, 1435, 1435, 1436, 1437,
	1437, 1438, 1439, 1440, 1440, 1441, 1441, 1442, 1442, 1443, 1443, 1444, 1444, 1445, 1446, 1447, 1448, 1448, 1449, 1449,
	1450, 1450, 1451, 1451, 1452, 1452, 1453, 1454, 1455, 1455, 1456, 1456, 1457, 1457, 1458, 1458, 1459, 1460, 1461, 1462,
	1462, 1463, 1463, 1464, 1464, 1465, 1465, 1466, 1466, 1467, 1467, 1468, 1469, 1470, 1470, 1471, 1471, 1472, 1472, 1473,
	1474, 1474, 1475, 1475, 1476, 1477, 1478, 1478, 1479, 1479, 1480, 1480, 1481, 1481, 1482, 1482, 1483, 1483, 1484, 1485,
	1486, 1486, 1487, 1487, 1488, 1488, 1489, 1489, 1490, 1491, 1492, 1493, 1493, 1494, 1494, 1495, 1495, 1496, 1496, 1497,
	1497, 1498, 1499, 1500, 1500, 1501, 1501, 1502, 1502, 1503, 1503, 1504, 1504, 1505, 1505, 1506, 1506, 1507, 1508, 1509,
	1509, 1510, 1510, 1511, 1511, 1512, 1512, 1513, 1513, 1514, 1515, 1516, 1516, 1517, 1517, 1518, 1518, 1519, 1519, 1520,
	1520, 1521, 1521, 1522, 1523, 1524, 1525, 1525, 1526, 1526, 1527, 1527, 1528, 1528, 1529, 1529, 1530, 1531, 1532, 1532,
	1533, 1533, 1534, 1534, 1535, 1535, 1536, 1536, 1537, 1538, 1539, 1539, 1540, 1540, 1541, 1541, 1542, 1542, 1543, 1543,
	1544, 1544, 1545, 1546, 1547, 1547, 1548, 1548, 1549, 1549, 1550, 1550, 1551, 1551, 1552, 1552, 1553, 1554, 1555, 1555,
	1556, 1556, 1557, 1557, 1558, 1558, 1559, 1559, 1560, 1561, 1562, 1562, 1563, 1563, 1564, 1564, 1564, 1565, 1565, 1566,
	1566, 1567, 1567, 1568, 1569, 1570, 1570, 1571, 1571, 1572, 1572, 1573, 1573, 1574, 1574, 1575, 1575, 1576, 1577, 1578,
	1578, 1579, 1579, 1580, 1580, 1581, 1581, 1582, 1582, 1583, 1584, 1585, 1585, 1586, 1586, 1587, 1587, 1588, 1588, 1589,
	1589, 1590, 1590, 1591, 1592, 1593, 1593, 1594, 1594, 1595, 1595, 1596, 1596, 1597, 1597, 1598, 1598, 1599, 1600, 1600,
	1601, 1601, 1602, 1602, 1603, 1603, 1604, 1604, 1605, 1605, 1606, 1607, 1608, 1608, 1609, 1609, 1610, 1610, 1611, 1611,
	1612, 1612, 1613, 1613, 1614, 1615, 1616, 1616, 1617, 1617, 1618, 1618, 1618, 1619, 1619, 1620, 1620, 1621, 1621, 1622,
	1623, 1624, 1624, 1625, 1625, 1626, 1626, 1627, 1627, 1628, 1629, 1630, 1630, 1631, 1631, 1632, 1632, 1633, 1633, 1634,
	1634, 1634, 1635, 1635, 1636, 1636, 1637, 1638, 1639, 1639, 1640, 1640, 1641, 1641, 1642, 1642, 1643, 1643, 1644, 1644,
	1645, 1646, 1647, 1647, 1648, 1648, 1648, 1649, 1649, 1650, 1650, 1651, 1651, 1652, 1653, 1654, 1654, 1655, 1655, 1656,
	1656, 1657, 1657, 1658, 1658, 1659, 1659, 1660, 1661, 1661, 1662, 1662, 1663, 1663, 1664, 1664, 1665, 1665, 1666, 1666,
	1667, 1668, 1669, 1669, 1670, 1670, 1671, 1671, 1672, 1672, 1672, 1673, 1673, 1674, 1674, 1675, 1676, 1677, 1677, 1678,
	1678, 1679, 1679, 1680, 1680, 1681, 1681, 1682, 1682, 1683, 1684, 1684, 1685, 1685, 1686, 1686, 1687, 1687, 1688, 1688,
	1689, 1689, 1690, 1690, 1691, 1692, 1692, 1693, 1693, 1694, 1694, 1695, 1695, 1696, 1696, 1697, 1698, 1699, 1699, 1700,
	1700, 1701, 1701, 1702, 1702, 1702, 1703, 1703, 1704, 1704, 1705, 1705, 1706, 1707, 1708, 1708, 1709, 1709, 1710, 1710,
	1710, 1711, 1711, 1712, 1712, 1713, 1714, 1715, 1715, 1716, 1716, 1717, 1717, 1718, 1718, 1719, 1719, 1720, 1720, 1721,
	1722, 1722, 1723, 1723, 1724, 1724, 1725, 1725, 1726, 1726, 1727, 1727, 1727, 1728, 1729, 1730, 1730, 1731, 1731, 1732,
	1732, 1733, 1733, 1734, 1734, 1735, 1735, 1736, 1736, 1737, 1737, 1738, 1739, 1739, 1740, 1740, 1741, 1741, 1742, 1742,
	1742, 1743, 1743, 1744, 1745, 1746, 1746, 1747, 1747, 1748, 1748, 1749, 1749, 1750, 1750, 1750, 1751, 1751, 1752, 1753,
	1754, 1754, 1755, 1755, 1756, 1756, 1757, 1757, 1757, 1758, 1758, 1759, 1759, 1760, 1761, 1762, 1762, 1763, 1763, 1764,
	1764, 1764, 1765, 1765, 1766, 1767, 1768, 1768, 1769, 1769, 1770, 1770, 1771, 1771, 1771, 1772, 1772, 1773, 1773, 1774,
	1774, 1775, 1776, 1777, 1777, 1778, 1778, 1778, 1779, 1779, 1780, 1780, 1781, 1781, 1782, 1782, 1783, 1784, 1785, 1785,
	1785, 1786, 1786, 1787, 1787, 1788, 1788, 1789, 1789, 1790, 1791, 1791, 1792, 1792, 1793, 1793, 1794, 1794, 1795, 1795,
	1796, 1796, 1797, 1797, 1798, 1799, 1799, 1800, 1800, 1801, 1801, 1802, 1802, 1803, 1803, 1803, 1804, 1804, 1805, 1805,
	1806, 1807, 1808, 1808, 1809, 1809, 1810, 1810, 1810, 1811, 1811, 1812, 1812, 1813, 1814, 1815, 1815, 1816, 1816, 1816,
	1817, 1817, 1818, 1818, 1819, 1819, 1820, 1820, 1821, 1822, 1822, 1823, 1823, 1824, 1824, 1825, 1825, 1826, 1826, 1827,
	1827, 1827, 1828, 1828, 1829, 1830, 1831, 1831, 1832, 1832, 1833, 1833, 1833, 1834, 1834, 1835, 1835, 1836, 1837, 1837,
	1838, 1838, 1839, 1839, 1840, 1840, 1841, 1841, 1842, 1842, 1843, 1843, 1844, 1845, 1845, 1846, 1846, 1847, 1847, 1848,
	1848, 1849, 1849, 1850, 1850, 1851, 1851, 1852, 1852, 1853, 1853, 1854, 1854, 1855, 1855, 1856, 1856, 1857, 1857, 1858,
	1858, 1859, 1860, 1860, 1861, 1861, 1862, 1862, 1863, 1863, 1864, 1864, 1865, 1865, 1866, 1866, 1867, 1868, 1868, 1869,
	1869, 1870, 1870, 1871, 1871, 1872, 1872, 1873, 1873, 1874, 1874, 1875, 1875, 1876, 1876, 1877, 1877, 1878, 1878, 1879,
	1879, 1880, 1880, 1881, 1881, 1882, 1883, 1883, 1884, 1884, 1885, 1885, 1886, 1886, 1887, 1887, 1887, 1888, 1888, 1889,
	1889, 1890, 1891, 1891, 1892, 1892, 1893, 1893, 1894, 1894, 1895, 1895, 1896, 1896, 1897, 1897, 1898, 1899, 1899, 1900,
	1900, 1901, 1901, 1902, 1902, 1902, 1903, 1903, 1904, 1904, 1905, 1906, 1906, 1907, 1907, 1908, 1908, 1909, 1909, 1910,
	1910, 1911, 1911, 1912, 1912, 1913, 1914, 1914, 1915, 1915, 1916, 1916, 1917, 1917, 1917, 1918, 1918, 1919, 1919, 1920,
	1920, 1921, 1921, 1922, 1923, 1923, 1924, 1924, 1925, 1925, 1926, 1926, 1926, 1927, 1927, 1928, 1929, 1930, 1930, 1931,
	1931, 1932, 1932, 1932, 1933, 1933, 1934, 1934, 1935, 1935, 1936, 1937, 1937, 1938, 1938, 1939, 1939, 1940, 1940, 1941,
	1941, 1941, 1942, 1942, 1943, 1943, 1944, 1944, 1945, 1946, 1946, 1947, 1947, 1948, 1948, 1949, 1949, 1950, 1950, 1951,
	1952, 1952, 1953, 1953, 1954, 1954, 1955, 1955, 1955, 1956, 1956, 1957, 1957, 1958, 1958, 1959, 1960, 1960, 1961, 1961,
	1962, 1962, 1963, 1963, 1964, 1964, 1964, 1965, 1965, 1966, 1966, 1967, 1968, 1968, 1969, 1969, 1970, 1970, 1971, 1971,
	1972, 1972, 1973, 1973, 1974, 1975, 1975, 1976, 1976, 1977, 1977, 1977, 1978, 1978, 1979, 1979, 1980, 1980, 1981, 1981,
	1982, 1983, 1983, 1984, 1984, 1985, 1985, 1986, 1986, 1986, 1987, 1987, 1988, 1988, 1989, 1989, 1990, 1991, 1991, 1992,
	1992, 1993, 1993, 1994, 1994, 1995, 1995, 1996, 1996, 1997, 1997, 1998, 1998, 1999, 1999, 2000, 2000, 2001, 2001, 2002,
	2002, 2003, 2003, 2004, 2004, 2005, 2006, 2006, 2007, 2007, 2008, 2008, 2008, 2009, 2009, 2010, 2010, 2011, 2011, 2012,
	2012, 2013, 2014, 2014, 2015, 2015, 2016, 2016, 2017, 2017, 2017, 2018, 2018, 2019, 2019, 2020, 2021, 2021, 2022, 2022,
	2023, 2023, 2024, 2024, 2025, 2025, 2025, 2026, 2026, 2027, 2027, 2028, 2029, 2029, 2030, 2030, 2031, 2031, 2032, 2032,
	2033, 2033, 2033, 2034, 2034, 2035, 2035, 2036, 2037, 2037, 2038, 2038, 2039, 2039, 2040, 2040, 2041, 2041, 2042, 2042,
	2043, 2044, 2042, 2045, 2045, 2046, 2046, 2047
};

struct LCM_setting_table_by_type lcd_init_cmd[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x23}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x00, 0x60}},
	{TYPE_DCS, 0x02, {0x07, 0x20}},
	{TYPE_DCS, 0x02, {0x08, 0x01}},
	{TYPE_DCS, 0x02, {0x09, 0x5A}},
	{TYPE_DCS, 0x02, {0x10, 0x0C}},
	{TYPE_DCS, 0x02, {0x11, 0x03}},
	{TYPE_DCS, 0x02, {0x12, 0x6B}},
	{TYPE_DCS, 0x02, {0x15, 0x03}},
	{TYPE_DCS, 0x02, {0x16, 0x10}},
	{TYPE_DCS, 0x02, {0x0A, 0x8E}},
	{TYPE_DCS, 0x02, {0x0B, 0x8E}},
	{TYPE_DCS, 0x02, {0x0C, 0x8E}},
	{TYPE_DCS, 0x02, {0x0D, 0x00}},
	{TYPE_DCS, 0x02, {0x19, 0x00}},
	{TYPE_DCS, 0x02, {0x1A, 0x04}},
	{TYPE_DCS, 0x02, {0x1B, 0x08}},
	{TYPE_DCS, 0x02, {0x1C, 0x0C}},
	{TYPE_DCS, 0x02, {0x1D, 0x10}},
	{TYPE_DCS, 0x02, {0x1E, 0x14}},
	{TYPE_DCS, 0x02, {0x1F, 0x18}},
	{TYPE_DCS, 0x02, {0x20, 0x1C}},
	{TYPE_DCS, 0x02, {0x21, 0x20}},
	{TYPE_DCS, 0x02, {0x22, 0x24}},
	{TYPE_DCS, 0x02, {0x23, 0x28}},
	{TYPE_DCS, 0x02, {0x24, 0x2C}},
	{TYPE_DCS, 0x02, {0x25, 0x30}},
	{TYPE_DCS, 0x02, {0x26, 0x34}},
	{TYPE_DCS, 0x02, {0x27, 0x38}},
	{TYPE_DCS, 0x02, {0x28, 0x3C}},
	{TYPE_DCS, 0x02, {0x2A, 0x20}},
	{TYPE_DCS, 0x02, {0x2B, 0x20}},
	/* CABC_PWM_UI 55=01*/
	{TYPE_DCS, 0x02, {0x30, 0xFF}},
	{TYPE_DCS, 0x02, {0x31, 0xFD}},
	{TYPE_DCS, 0x02, {0x32, 0xFC}},
	{TYPE_DCS, 0x02, {0x33, 0xFA}},
	{TYPE_DCS, 0x02, {0x34, 0xF8}},
	{TYPE_DCS, 0x02, {0x35, 0xF6}},
	{TYPE_DCS, 0x02, {0x36, 0xF4}},
	{TYPE_DCS, 0x02, {0x37, 0xF2}},
	{TYPE_DCS, 0x02, {0x38, 0xF0}},
	{TYPE_DCS, 0x02, {0x39, 0xEE}},
	{TYPE_DCS, 0x02, {0x3A, 0xEC}},
	{TYPE_DCS, 0x02, {0x3B, 0xEA}},
	{TYPE_DCS, 0x02, {0x3D, 0xE9}},
	{TYPE_DCS, 0x02, {0x3F, 0xE8}},
	{TYPE_DCS, 0x02, {0x40, 0xE7}},
	{TYPE_DCS, 0x02, {0x41, 0xE6}},
	/* CABC_PWM_STILL 55=02*/
	{TYPE_DCS, 0x02, {0x45, 0xFF}},
	{TYPE_DCS, 0x02, {0x46, 0xF9}},
	{TYPE_DCS, 0x02, {0x47, 0xF6}},
	{TYPE_DCS, 0x02, {0x48, 0xF2}},
	{TYPE_DCS, 0x02, {0x49, 0xF0}},
	{TYPE_DCS, 0x02, {0x4A, 0xEC}},
	{TYPE_DCS, 0x02, {0x4B, 0xE8}},
	{TYPE_DCS, 0x02, {0x4C, 0xE4}},
	{TYPE_DCS, 0x02, {0x4D, 0xE0}},
	{TYPE_DCS, 0x02, {0x4E, 0xDE}},
	{TYPE_DCS, 0x02, {0x4F, 0xD9}},
	{TYPE_DCS, 0x02, {0x50, 0xD6}},
	{TYPE_DCS, 0x02, {0x51, 0xD4}},
	{TYPE_DCS, 0x02, {0x52, 0xC2}},
	{TYPE_DCS, 0x02, {0x53, 0xD0}},
	{TYPE_DCS, 0x02, {0x54, 0xCD}},
	/* CABC_PWM_MOV 55=03 */
	{TYPE_DCS, 0x02, {0x58, 0xFF}},
	{TYPE_DCS, 0x02, {0x59, 0xF6}},
	{TYPE_DCS, 0x02, {0x5A, 0xF0}},
	{TYPE_DCS, 0x02, {0x5B, 0xEB}},
	{TYPE_DCS, 0x02, {0x5C, 0xE8}},
	{TYPE_DCS, 0x02, {0x5D, 0xE5}},
	{TYPE_DCS, 0x02, {0x5E, 0xE3}},
	{TYPE_DCS, 0x02, {0x5F, 0xE0}},
	{TYPE_DCS, 0x02, {0x60, 0xDE}},
	{TYPE_DCS, 0x02, {0x61, 0xDA}},
	{TYPE_DCS, 0x02, {0x62, 0xD7}},
	{TYPE_DCS, 0x02, {0x63, 0xD4}},
	{TYPE_DCS, 0x02, {0x64, 0xD2}},
	{TYPE_DCS, 0x02, {0x65, 0xD0}},
	{TYPE_DCS, 0x02, {0x66, 0xCC}},
	{TYPE_DCS, 0x02, {0x67, 0xC8}},

	{TYPE_DCS, 0x02, {0xFF, 0x2A}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xC5, 0x09}},

	{TYPE_DCS, 0x02, {0xFF, 0x25}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x23, 0x09}},
	{TYPE_DCS, 0x02, {0x2A, 0x09}},

	{TYPE_DCS, 0x02, {0xFF, 0x2A}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x14, 0x09}},
	{TYPE_DCS, 0x02, {0x1E, 0x09}},
	{TYPE_DCS, 0x02, {0x1F, 0x09}},

	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xB2, 0x91}},
	{TYPE_DCS, 0x02, {0xB3, 0x40}},

	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x07, {0x3B, 0x03, 0xC6, 0x1A, 0x0A, 0x0A, 0x00}},

	/* cabc config */
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x03, {0x51, 0x07, 0xFF}},
	{TYPE_DCS, 0x02, {0x53, 0x2C}},
	{TYPE_DCS, 0x02, {0xB9, 0x00}},
	{TYPE_DCS, 0x02, {0x55, 0x01}},
	{TYPE_DCS, 0x02, {0xB9, 0x02}},

	/* esd config */
	{TYPE_DCS, 0x02, {0xFF, 0x27}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xD0, 0x31}},
	{TYPE_DCS, 0x02, {0xD1, 0x84}},
	{TYPE_DCS, 0x02, {0xD2, 0x38}},
	{TYPE_DCS, 0x02, {0xDE, 0x41}},
	{TYPE_DCS, 0x02, {0xDF, 0x02}},
};

struct LCM_setting_table_by_type set_60Hz_120Hz_init[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xB2, 0x91}},
	{TYPE_DCS, 0x02, {0xB3, 0x40}}
};

struct LCM_setting_table_by_type set_144Hz_init[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xB2, 0x00}},
	{TYPE_DCS, 0x02, {0xB3, 0x00}}
};

struct LCM_setting_table_by_type set_48_50_90Hz_init[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xB2, 0x00}},
	{TYPE_DCS, 0x02, {0xB3, 0x80}}
};

struct LCM_setting_table_by_type lcd_sleep_out[] = {
	{TYPE_DCS, 0x01, {0x11}},
	{TYPE_MDELAY, 120, {}},
	{TYPE_DCS, 0x01, {0x29}},
	{TYPE_MDELAY, 10, {}}
};

struct LCM_setting_table_by_type lcd_sleep_in[] = {
	{TYPE_DCS, 0x01, {0x28}},
	{TYPE_MDELAY, 10, {}},
	{TYPE_DCS, 0x01, {0x10}},
	{TYPE_MDELAY, 65, {}}
};

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_init_write_by_type(struct lcm *ctx, struct LCM_setting_table_by_type setting)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret = 0;

	if (ctx->error < 0) {
		pr_err("[lcd_info] %s: ctx->error=%d, LINE=%d\n",__func__, ctx->error, __LINE__);
		return;
	}

	switch(setting.type) {
		case TYPE_GERNERIC:
			ret = mipi_dsi_generic_write(dsi, setting.para_list, setting.count);
			break;
		case TYPE_PPS:
			ret = mipi_dsi_picture_parameter_set(dsi, (struct drm_dsc_picture_parameter_set *)(setting.para_list));
			break;
		case TYPE_DCS:
			ret = mipi_dsi_dcs_write_buffer(dsi, setting.para_list, setting.count);
			break;
		case TYPE_MDELAY:
			M_DELAY(setting.count);
			break;
		case TYPE_UDELAY:
			U_DELAY(setting.count);
			break;
		default:
			pr_err("[lcd_info] %s: type=%d is invalid\n",__func__, setting.type);
			break;
	}
	if (ret < 0) {
		pr_err("[lcd_info] error %zd writing type=%dh\n", ret, setting.type);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret = 0;

	if (ctx->error < 0) {
		printk("[lcd_info]%s:ctx->error=%d LINE=%d\n",__func__, ctx->error, __LINE__);
		return ret;
	}

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		printk("[lcd_info]error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		printk("%s return %d data(0x%08x) to dsi engine\n", __func__, ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static unsigned int lcm_enable_reg_vrf18_aif(struct lcm *ctx, int en)
{
	unsigned int ret = 0;
	static bool vddio_enable_flag = false;

	printk("lqb [lcd_info]%s +\n", __func__);
	if(!ctx->reg_vrf18_aif) {
		printk("lqb %s error return -1\n", __func__);
		return -1;
	}

	if(en) {
		if (!vddio_enable_flag) {
			if (is_pd_with_guesture) {
				printk("[lcd_info]%s:[ERROE]tp guesture enable, but set vddio disable when panel off\n", __func__);
			}
			ret = regulator_set_voltage(ctx->reg_vrf18_aif, 1900000, 1900000);
			ret = regulator_enable(ctx->reg_vrf18_aif);
			vddio_enable_flag = true;
			printk("[lcd_info]%s vddio enable\n", __func__);
		} else {
			if (is_pd_with_guesture)
				printk("[lcd_info]%s:tp guesture enable, vddio not set disable when panel off\n", __func__);
		}
	} else {
		if (vddio_enable_flag) {
			if (is_pd_with_guesture) {
				printk("[lcd_info]%s: tp guesture enable, not set vddio disable\n", __func__);
				return 0;
			}
			ret = regulator_disable(ctx->reg_vrf18_aif);
			vddio_enable_flag = false;
			printk("[lcd_info]%s vddio disable\n", __func__);
		}
	}
	printk("lqb [lcd_info]%s -\n", __func__);

	return ret;
}

static void lcm_panel_init_on(struct lcm *ctx)
{
	int i= 0;

	printk("[lcd_info]%s +\n", __func__);

	for (i = 0; i < sizeof(lcd_init_cmd) / sizeof(struct LCM_setting_table_by_type); i++) {
		lcm_init_write_by_type(ctx, lcd_init_cmd[i]);
	}

	if ((g_fps_current == 60) || (g_fps_current == 120) || (g_fps_current == 30)) {
		for (i = 0; i < sizeof(set_60Hz_120Hz_init) / sizeof(struct LCM_setting_table_by_type); i++) {
			lcm_init_write_by_type(ctx, set_60Hz_120Hz_init[i]);
		}
	} else if ((g_fps_current == 48) || (g_fps_current == 50) || (g_fps_current == 90)) {
		for (i = 0; i < sizeof(set_48_50_90Hz_init) / sizeof(struct LCM_setting_table_by_type); i++) {
			lcm_init_write_by_type(ctx, set_48_50_90Hz_init[i]);
		}
	} else if (g_fps_current == 144) {
		for (i = 0; i < sizeof(set_144Hz_init) / sizeof(struct LCM_setting_table_by_type); i++) {
			lcm_init_write_by_type(ctx, set_144Hz_init[i]);
		}
	}

	for (i = 0; i < sizeof(lcd_sleep_out) / sizeof(struct LCM_setting_table_by_type); i++) {
		lcm_init_write_by_type(ctx, lcd_sleep_out[i]);
	}
}

static void lcm_panel_init_off(struct lcm *ctx)
{
	int i= 0;

	printk("[lcd_info]%s +\n", __func__);

	for (i = 0; i < sizeof(lcd_sleep_in) / sizeof(struct LCM_setting_table_by_type); i++) {
		lcm_init_write_by_type(ctx, lcd_sleep_in[i]);
	}

	printk("[lcd_info]%s -\n", __func__);
}

/* ktz8866 lcd base config */
static void ktz8866_lcd_bias_config(struct drm_panel *panel, int enable)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->bias_enp_en = devm_gpiod_get(ctx->dev, "bias-enp-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enp_en))
		pr_err("could not get bias_enp_en gpio\n");

	ctx->bias_enn_en = devm_gpiod_get(ctx->dev, "bias-enn-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enn_en))
		pr_err("could not get bias_enn_en gpio\n");

	printk("[lcd_info]%s base enable = %d.\n", __func__, enable);
	if(!enable){
		/* Disable ENN */
		gpiod_set_value(ctx->bias_enn_en, 0);
		printk("[lcd_info]%s bias_enn_en 0\n", __func__);
		usleep_range(5000, 5010);
		/* Disable ENP */
		gpiod_set_value(ctx->bias_enp_en, 0);
		printk("[lcd_info]%s bias_enp_en 0\n", __func__);
	} else {
		/* only config i2c0*/
		/* LCD_BOOST_CFG */
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x0C, 0x30, NULL, 0, 0);
		/* OUTP_CFG，OUTP = 6.0V */
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x0D, 0x28, NULL, 0, 0);
		/* OUTN_CFG，OUTN = -6.0V */
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x0E, 0x28, NULL, 0, 0);
		/* enable OUTP */
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x09, 0x99, NULL, 0, 0);
		/* enable ENP */
		gpiod_set_value(ctx->bias_enp_en, 1);
		printk("[lcd_info]%s bias_enp_en 1\n", __func__);
		usleep_range(5000, 5010);
		/* enable ENN */
		gpiod_set_value(ctx->bias_enn_en, 1);
		printk("[lcd_info]%s bias_enn_en 1\n", __func__);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enp_en);
	devm_gpiod_put(ctx->dev, ctx->bias_enn_en);
}


/* backlight ic is ktz8866 */
static int lcm_backlight_ic_config(struct drm_panel *panel, int enable)
{
	struct lcm *ctx = panel_to_lcm(panel);
	static bool backlight_ic_enable_flag = true;

	printk("[lcd_info]%s +\n", __func__);
	ctx->bias_en = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_en)) {
		pr_err("[lcd_info]%s:could not get pm-enable gpio\n", __func__);
		return 0;
	}

	if (enable) {
		if (!backlight_ic_enable_flag) {
			if (is_pd_with_guesture) {
				printk("[lcd_info]%s:[ERROR]tp guesture enable, but disable backlight ic when panel off\n", __func__);
			}
			gpiod_set_value(ctx->bias_en, 1);
			usleep_range(125, 130);

			/* lcd base enable config */
			ktz8866_lcd_bias_config(panel, enable);

			/* BL_CFG1；OVP=31.5V，非线性调光，PWM Disabled */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x02, 0X53, lcd_bl_i2c_s_client, 0x02, 0X53);//caba 0ff 0x5A, cabc on 0x5B
			/* Current ramp 256ms pwm_hyst 10lsb */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x03, 0XCD, lcd_bl_i2c_s_client, 0x03, 0XCD);
			/* BL_OPTION2；电感4.7uH，BL_CURRENT_LIMIT 2.5A */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x11, 0x37, lcd_bl_i2c_s_client, 0x11, 0x37);
			/* turn on-off ramp */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x14, 0x44, lcd_bl_i2c_s_client, 0x14, 0x44);
			/* Backlight Full-scale LED Current 26.8mA/CH */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x15, 0xD8, lcd_bl_i2c_s_client, 0x15, 0xD8);

			backlight_ic_enable_flag = true;
		} else {
			if (is_pd_with_guesture)
				printk("[lcd_info]%s:tp guesture enable, backlight ic not disable when panel off\n", __func__);
		}
	} else {
		if (backlight_ic_enable_flag) {
			if (is_pd_with_guesture) {
				printk("[lcd_info]%s: tp guesture enable, not disable backlight ic\n", __func__);
				lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x08, 0x00, lcd_bl_i2c_s_client, 0x08, 0x00);
				devm_gpiod_put(ctx->dev, ctx->bias_en);
				return 0;
			}
			/* lcd base disable config */
			ktz8866_lcd_bias_config(panel, enable);

			/* turn on-off ramp */
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x14, 0xFF, lcd_bl_i2c_s_client, 0x14, 0xFF);
			usleep_range(500, 1000);
			/* BL disabled and Current sink 1/2/3/4 /5 enabled；*/
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x08, 0x00, lcd_bl_i2c_s_client, 0x08, 0x00);
			usleep_range(5000, 6000);

			gpiod_set_value(ctx->bias_en, 0);

			backlight_ic_enable_flag = false;
		}
	}
	devm_gpiod_put(ctx->dev, ctx->bias_en);

	printk("[lcd_info]%s -\n", __func__);
	return 0;
}

static int lcd_bl_set_led_brightness_dual(unsigned int bl_level)//for set bringhtness
{
	uint8_t level_lsb = 0, level_msb = 0;
	// unsigned char bl_ic0_status = 0;
	// unsigned char bl_ic1_status = 0;

	level_lsb = bl_level & 0x07;
	level_msb = (bl_level >> 3) & 0xFF;

	if (silence_mode == 1)
		bl_level = 0;

	if (bl_level > 0) {
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x04, level_lsb, lcd_bl_i2c_s_client, 0x04, level_lsb);
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x05, level_msb, lcd_bl_i2c_s_client, 0x05, level_msb);
		if (!lcd_bl_set_led_flag) {
			usleep_range(5000, 5010);
			lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x08, 0x4F, lcd_bl_i2c_s_client, 0x08, 0x4F);
			lcd_bl_set_led_flag = true;
		}
	} else {
		/* Current ramp 640ms pwm_hyst 10lsb */
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x03, 0xFD, lcd_bl_i2c_s_client, 0x03, 0xFD);
		usleep_range(500, 1000);
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x04, 0, lcd_bl_i2c_s_client, 0x04, 0);
		lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x05, 0, lcd_bl_i2c_s_client, 0x05, 0);
		/* BL disabled and Current sink 1/2/3/4 enabled；*/
		// lcd_bl_i2c_write_dual(lcd_bl_i2c_m_client, 0x08, 0, lcd_bl_i2c_s_client, 0x08, 0);
		lcd_bl_set_led_flag = false;
	}
	pr_err("%s: bl_level:%d\n", __func__, bl_level);

	return 0;
}

int oplus_i2c_set_backlight(unsigned int level)
{
	if (level > MAX_NORMAL_BRIGHTNESS)
		level = MAX_NORMAL_BRIGHTNESS;

	printk("[lcd_info]%s: bl_level:%d, mapping value = %d\n", __func__, level, backlight_mapping_buf[level]);
	level_backup = level;
	oplus_display_brightness = level;
	level = backlight_mapping_buf[level];

	lcd_bl_set_led_brightness_dual(level);

	return 0;
}
EXPORT_SYMBOL(oplus_i2c_set_backlight);

static int lcm_panel_reset(struct drm_panel *panel, int en)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s + en:%d\n", __func__, en);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev,
		"reset", GPIOD_OUT_HIGH);

	if (en) {
		if (!is_pd_with_guesture) {
			gpiod_set_value(ctx->reset_gpio, 1);
			M_DELAY(10);
			gpiod_set_value(ctx->reset_gpio, 0);
			M_DELAY(5);
			gpiod_set_value(ctx->reset_gpio, 1);
			M_DELAY(10);
		} else {
			gpiod_set_value(ctx->reset_gpio, 0);
			M_DELAY(5);
			gpiod_set_value(ctx->reset_gpio, 1);
			M_DELAY(5);
			gpiod_set_value(ctx->reset_gpio, 0);
			M_DELAY(5);
			gpiod_set_value(ctx->reset_gpio, 1);
			M_DELAY(10);
		}
	} else {
		if (!is_pd_with_guesture) {
			printk("[lcd_info]%s:tp guesture disable, lcm reset off\n", __func__);
			gpiod_set_value(ctx->reset_gpio, 0);
			M_DELAY(2);
		} else {
			printk("[lcd_info]%s:tp guesture enable, lcm reset not off\n", __func__);
		}
	}

	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s +\n", __func__);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	int flag_poweroff = 1;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	int blank = 0;
#endif

	printk("[lcd_info]%s +\n", __func__);
	if (!ctx->prepared) {
		printk("[lcd_info]%s ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown == 0) && (esd_flag == 0)) {
		is_pd_with_guesture = true;
	} else {
		is_pd_with_guesture = false;
	}

	lcm_panel_init_off(ctx);

	ret = lcm_panel_reset(panel, 0);
	if(ret) {
		printk("[lcd_info]%s: panel reset off failed! ret=%d\n", __func__, ret);
	}

	ret = lcm_backlight_ic_config(panel, 0);
	if(ret) {
		printk("[lcd_info]%s: set bl_bias disable failed! ret=%d\n", __func__, ret);
	}

	printk("[lcd_info][TP] g_shutdown=%d\n", g_shutdown);

	if (is_pd_with_guesture) {
		flag_poweroff = 0;
		pr_err("[TP] tp gesture is enable, Display not to poweroff\n");
	} else {
		flag_poweroff = 1;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
		blank = LCD_CTL_RST_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_info("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
		blank = LCD_CTL_CS_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_info("[TP]TP CS will change to gpio mode and low\n");
#endif
	}

	//disable 1.8V
	ret = lcm_enable_reg_vrf18_aif(ctx, 0);
	if(ret) {
		printk("[lcd_info]%s: set vddio off failed! ret=%d\n", __func__, ret);
	}

	ctx->error = 0;
	ctx->prepared = false;
	lcd_bl_set_led_flag = false;
	printk("[lcd_info]%s -\n", __func__);

	return ret;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	int blank = 0;
#endif
	//int mode = 0;

	printk("[lcd_info]%s +\n", __func__);

	if (ctx->prepared){
		printk("[lcd_info]%s frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	ret = lcm_backlight_ic_config(panel, 1);
	if(ret) {
		printk("[lcd_info]%s: set bl_bias enable failed! ret=%d\n", __func__, ret);
	}
	M_DELAY(10);

	// lcd reset
	ret = lcm_panel_reset(panel, 1);
	if(ret) {
		printk("[lcd_info]%s: set panel_reset failed! ret=%d\n", __func__, ret);
	}

	lcm_panel_init_on(ctx);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	//mode = get_boot_mode();
	//pr_info("[TP] in dis_panel_power_on, mode = %d\n", mode);

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP] spi CS set to high\n");
	usleep_range(5000, 5100);
	blank = LCD_CTL_TP_LOAD_FW;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP] start to load fw!\n");
#endif

	if(esd_flag)
		oplus_i2c_set_backlight(level_backup);

	printk("[lcd_info]%s -\n", __func__);

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s +\n", __func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;
	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params_pan_60hz= {
	//fhd_sdc_60_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x91} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x40} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_120_60_30,
		.vfp = VFP_60HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

static struct mtk_panel_params ext_params_pan_120hz = {
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.vfp_low_power = VFP_60HZ,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x91} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x40} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_120_60_30,
		.vfp = VFP_120HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

static struct mtk_panel_params ext_params_pan_30hz= {
	//fhd_sdc_30_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x91} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x40} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_120_60_30,
		.vfp = VFP_30HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

static struct mtk_panel_params ext_params_hand_90hz= {
	//fhd_sdc_90_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x00} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x80} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_90_50_48,
		.vfp = VFP_90HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

static struct mtk_panel_params ext_params_hand_50hz= {
	//fhd_sdc_144_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x00} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x80} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_90_50_48,
		.vfp = VFP_50HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

static struct mtk_panel_params ext_params_hand_48hz= {
	//fhd_sdc_144_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x00} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x80} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_90_50_48,
		.vfp = VFP_48HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};


static struct mtk_panel_params ext_params_hand_144hz= {
	//fhd_sdc_144_mode
	.data_rate = 1108,//(92+20+96+2800/2)*(26+2+196+2000)*8*144/4
	.vdo_per_frame_lp_enable = 1,
	.data_rate_khz = 1108090,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.ssc_enable = 0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.change_fps_by_vfp_send_cmd = 0,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 144,
		.dfps_cmd_table[0] = {0, 2, {0xFF, 0x10} },
		.dfps_cmd_table[1] = {0, 2, {0xFB, 0x01} },
		.dfps_cmd_table[2] = {0, 2, {0xB2, 0x00} },
		.dfps_cmd_table[3] = {0, 2, {0xB3, 0x00} },
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_144HZ,
		.vfp = VFP_144HZ,
	},
	.phy_timcon = {
		.hs_trail = 16,
		.clk_trail = 15,
	},
	.dsc_params = {
		.enable 				 = 1,
		.dual_dsc_enable		 = 1,
		.ver					 = 18,
		.slice_mode 			 = 1,
		.rgb_swap				 = 0,
		.dsc_cfg				 = 34,
		.rct_on 				 = 1,
		.bit_per_channel		 = 8,
		.dsc_line_buf_depth		 = 9,
		.bp_enable				 = 1,
		.bit_per_pixel			 = 128,
		.pic_height 			 = 2000,
		.pic_width				 = 1400,
		.slice_height			 = 20,
		.slice_width			 = 700,
		.chunk_size 			 = 700,
		.xmit_delay 			 = 512,
		.dec_delay				 = 637,
		.scale_value			 = 32,
		.increment_interval		 = 526,
		.decrement_interval		 = 9,
		.line_bpg_offset		 = 13,
		.nfl_bpg_offset			 = 1402,
		.slice_bpg_offset		 = 988,
		.initial_offset			 = 6144,
		.final_offset			 = 4304,
		.flatness_minqp			 = 3,
		.flatness_maxqp			 = 12,
		.rc_model_size			 = 8192,
		.rc_edge_factor			 = 6,
		.rc_quant_incr_limit0	 = 11,
		.rc_quant_incr_limit1	 = 11,
		.rc_tgt_offset_hi		 = 3,
		.rc_tgt_offset_lo		 = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.vendor = "NT36532W",
	.manufacture = "novatek",
};

/*
static void lcm_dcs_write_ext(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0) {
		printk("[lcd_info]%s:ctx->error=%d\n",__func__,ctx->error);
		return;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);

	if (ret < 0) {
		printk("[lcd_info]error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}
*/
static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;

	printk("[lcd_info]%s +\n", __func__);

	if (ctx->prepared){
		printk("[lcd_info]%s frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown == 0) && (esd_flag == 0)) {
		is_pd_with_guesture = true;
	} else {
		is_pd_with_guesture = false;
	}

	//set vddi 1.8v
	ret = lcm_enable_reg_vrf18_aif(ctx, 1);
	if(ret) {
		printk("[lcd_info]%s: set vddio on failed! ret=%d\n", __func__, ret);
	}

	printk("[lcd_info]%s -\n", __func__);
	return ret;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int bl_level)
{
	if (bl_level > MAX_NORMAL_BRIGHTNESS)
		bl_level = MAX_NORMAL_BRIGHTNESS;

	printk("[lcd_info]%s: bl_level:%d, mapping value = %d\n", __func__, bl_level, backlight_mapping_buf[bl_level]);
	level_backup = bl_level;
	oplus_display_brightness = bl_level;
	bl_level = backlight_mapping_buf[bl_level];
	lcd_bl_set_led_brightness_dual(bl_level);

	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s on=%d\n", __func__, on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static void lcm_pack_modes(unsigned int type,
			struct drm_display_mode * pack_mode,
			struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	mode = drm_mode_duplicate(connector->dev, pack_mode);
	if (!mode) {
		printk("[lcd_info]failed to add mode %ux%ux@%u\n",
			pack_mode->hdisplay, pack_mode->vdisplay,
			drm_mode_vrefresh(pack_mode));
	}

	drm_mode_set_name(mode);
	mode->type = type;
	printk("lcm_pack_modes:mode->name[%s] mode->type[%u] htotal=%u vtotal =%u\n",
		mode->name, mode->type, mode->htotal, mode->vtotal);
	drm_mode_probed_add(connector, mode);
}

static int lcm_get_modes(struct drm_panel *panel,
			struct drm_connector *connector) {

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		(struct drm_display_mode *)&pan_display_mode_120hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&pan_display_mode_60hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&pan_display_mode_30hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&hand_display_mode_90hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&hand_display_mode_50hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&hand_display_mode_48hz, connector);

	lcm_pack_modes(DRM_MODE_TYPE_DRIVER,
		(struct drm_display_mode *)&hand_display_mode_144hz, connector);

	connector->display_info.width_mm = PHYSICAL_WIDTH_MM;
	connector->display_info.height_mm = PHYSICAL_HEIGHT_MM;

	printk("lqb lcm_get_modes-");

	return 1;
}

struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int lcm_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int target_fps;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);

	target_fps = drm_mode_vrefresh(m);

	printk("%s %d target_fps=%d, mode=%u\n", __func__, __LINE__, target_fps, mode);

	if (target_fps == 60){
		ext->params = &ext_params_pan_60hz;
	} else if (target_fps == 120){
		ext->params = &ext_params_pan_120hz;
	} else if (target_fps == 30){
		ext->params = &ext_params_pan_30hz;
	} else if (target_fps == 90){
		ext->params = &ext_params_hand_90hz;
	} else if (target_fps == 50){
		ext->params = &ext_params_hand_50hz;
	} else if (target_fps == 48){
		ext->params = &ext_params_hand_48hz;
	} else if (target_fps == 144){
		ext->params = &ext_params_hand_144hz;
	} else {
		printk("lcm_ext_param_set: No mode to set fps = %d \n", target_fps);
		ret = 1;
	}

	g_fps_current = target_fps;

	return ret;
}

static void mode_switch_to_120(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x91);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x40);
	}
}

static void mode_switch_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x91);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x40);
	}
}

static void mode_switch_to_30(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x91);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x40);
	}
}

static void mode_switch_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x80);
	}
}

static void mode_switch_to_50(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x80);
	}
}

static void mode_switch_to_48(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x80);
	}
}

static void mode_switch_to_144(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	if (stage == BEFORE_DSI_POWERDOWN) {
		struct lcm *ctx = panel_to_lcm(panel);

		lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
		lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xB2, 0x00);
		lcm_dcs_write_seq_static(ctx, 0xB3, 0x00);
	}
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, dst_mode);

	if (cur_mode == dst_mode)
		return ret;

	if (drm_mode_vrefresh(m) == 60) { /*switch to 60 */
		mode_switch_to_60(panel, stage);
		printk("[%d]%s zzq mode_switch_to_60\n", __LINE__, __func__);
	} else if (drm_mode_vrefresh(m) == 120) { /*switch to 120 */
		mode_switch_to_120(panel, stage);
	} else if (drm_mode_vrefresh(m) == 30) { /*switch to 30 */
		mode_switch_to_30(panel, stage);
	} else if (drm_mode_vrefresh(m) == 90) { /*switch to 90 */
		mode_switch_to_90(panel, stage);
	}else if (drm_mode_vrefresh(m) == 50) { /*switch to 50 */
		mode_switch_to_50(panel, stage);
	} else if (drm_mode_vrefresh(m) == 48) { /*switch to 48 */
		mode_switch_to_48(panel, stage);
	} else if (drm_mode_vrefresh(m) == 144) { /*switch to 144 */
		mode_switch_to_144(panel, stage);
	} else
		ret = 1;

	return ret;
}

static int lcd_esd_gpio_read(struct drm_panel *panel)
{
	struct lcm *ctx = container_of(panel, struct lcm, panel);
	int master_read_value = 0, slave_read_value = 0;
	int ret = 0;

	master_read_value = gpiod_get_value(ctx->master_esd_gpio);
	slave_read_value = gpiod_get_value(ctx->slave_esd_gpio);
	printk("[lcd_info][ESD]%s: frist time master:%d slave:%d\n", __func__, master_read_value, slave_read_value);
	if( master_read_value || slave_read_value) {
		msleep(100);
		master_read_value = gpiod_get_value(ctx->master_esd_gpio);
		slave_read_value = gpiod_get_value(ctx->slave_esd_gpio);
		printk("[lcd_info][ESD]%s second time master:%d slave:%d\n", __func__, master_read_value, slave_read_value);
		if(master_read_value || slave_read_value) {
			pr_err("[ESD]%s: triger esd to recovery\n", __func__);
			ret = 1;
		}
	} else {
		ret = 0;
	}

	if (1 == ret) {
		char payload[200] = "";
		int cnt = 0;

		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "DisplayDriverID@@507$$");
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "ESD:");
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "master_0x%x, slave_0x%x",
			master_read_value, slave_read_value);
		pr_err("ESD check failed: %s\n", payload);
		mm_fb_display_kevent(payload, MM_FB_KEY_RATELIMIT_1H, "ESD check failed");
	}

	printk("[lcd_info][ESD]%s:ret=%d\n", __func__, ret);
	return ret;
}

#define CABC_MODE_CMD_SIZE 7
static void cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	unsigned char cabc_mode_para = 0;
	int i = 0;
	unsigned char cabc_mode_cmd[CABC_MODE_CMD_SIZE][3] = {
		{0xFF, 0x10},
		{0xFB, 0x01},
		{0x51, 0x07, 0xFF},
		{0x53, 0x2C},
		{0xB9, 0x00},
		{0x55, 0x00},
		{0xB9, 0x02},
	};

	if (cabc_mode == 0) {
		cabc_mode_para = 0;
	} else if (cabc_mode == 1) {
		cabc_mode_para = 1;
	} else if (cabc_mode == 2) {
		cabc_mode_para = 2;
	} else if (cabc_mode == 3) {
		cabc_mode_para = 3;
	} else {
		printk("[lcd_info]%s: cabc_mode=%d is not support, close cabc !\n", __func__, cabc_mode);
		cabc_mode_para = 0;
	}

	cabc_mode_cmd[5][1] = cabc_mode_para;
	for (i = 0; i < CABC_MODE_CMD_SIZE; i++) {
		cb(dsi, handle, cabc_mode_cmd[i], ARRAY_SIZE(cabc_mode_cmd[i]));
	}
	printk("[lcd_info]%s:cabc mode_%d, set cabc_para=%d\n", __func__, cabc_mode, cabc_mode_para);
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.panel_poweron = lcm_panel_poweron,
	.ext_param_set = lcm_ext_param_set,
	.mode_switch = mode_switch,
	.esd_read_gpio = lcd_esd_gpio_read,
	.cabc_switch = cabc_mode_switch,
};
#endif

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret = 0;

	printk("[lcd_info] lqb %s +\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				printk("[lcd_info]No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			printk("[lcd_info]device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		printk("[lcd_info]skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	g_dp_support = true;
	printk("[lcd_info]g_dp_support is %d\n", g_dp_support);

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		printk("[lcd_info]cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ctx->bias_en = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_en)) {
		printk("[lcd_info]cannot get bias_en %ld\n",
			PTR_ERR(ctx->bias_en));
		return PTR_ERR(ctx->bias_en);
	}
	// gpiod_set_value(ctx->bias_en, 1);
	// printk("%s set GPIO15 to high befor devm_gpiod_put\n", __func__);
	devm_gpiod_put(ctx->dev, ctx->bias_en);

	ctx->bias_enp_en = devm_gpiod_get(ctx->dev, "bias-enp-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enp_en)) {
		printk("[lcd_info]cannot getbias-enp-en-gpios %ld\n",
			 PTR_ERR(ctx->bias_enp_en));
		return PTR_ERR(ctx->bias_enp_en);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enp_en);

	ctx->bias_enn_en = devm_gpiod_get(ctx->dev, "bias-enn-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enn_en)) {
		printk("[lcd_info]cannot getbias-enp-en-gpios %ld\n",
			 PTR_ERR(ctx->bias_enn_en));
		return PTR_ERR(ctx->bias_enn_en);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enn_en);

	ctx->reg_vrf18_aif = regulator_get(ctx->dev, "1p8");
	if (IS_ERR(ctx->reg_vrf18_aif)) {
		printk("[lcd_info]cannot get reg_vrf18_aif %ld\n",
			PTR_ERR(ctx->reg_vrf18_aif));
		return -517;
	} else {
		printk("[lcd_info]get reg_vrf18_aif success\n");
	}

	printk("lqb [lcd_info]%s lcm_enable_reg_vrf18_aif LINE=%d\n", __func__, __LINE__);
	ret = lcm_enable_reg_vrf18_aif(ctx, 1);

	printk("[lcd_info][ESD]%s master gpio LINE=%d\n", __func__, __LINE__);
	ctx->master_esd_gpio = devm_gpiod_get_optional(ctx->dev, "master-esd", GPIOD_IN);
	if (IS_ERR(ctx->master_esd_gpio)) {
		DISP_ERR("[lcd_info]cannot get master_esd_gpio %ld\n",
			PTR_ERR(ctx->master_esd_gpio));
		return PTR_ERR(ctx->master_esd_gpio);
	} else {
		gpiod_direction_input(ctx->master_esd_gpio);
	}

	printk("[lcd_info][ESD]%s slave gpio LINE=%d\n", __func__, __LINE__);
	ctx->slave_esd_gpio = devm_gpiod_get_optional(ctx->dev, "slave-esd", GPIOD_IN);
	if (IS_ERR(ctx->slave_esd_gpio)) {
		DISP_ERR("[lcd_info]cannot get slave_esd_gpio %ld\n",
			PTR_ERR(ctx->slave_esd_gpio));
		return PTR_ERR(ctx->slave_esd_gpio);
	} else {
		gpiod_direction_input(ctx->slave_esd_gpio);
	}
	ctx->prepared = true;
	ctx->enabled = true;

	printk("lqb [lcd_info]%s drm_panel_init LINE=%d\n", __func__, __LINE__);

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	printk("lqb [lcd_info]%s drm_panel_add LINE=%d\n", __func__, __LINE__);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		printk("lqb [lcd_info]%s mipi_dsi_attach fail LINE=%d ret=%d\n", __func__, __LINE__, ret);
		drm_panel_remove(&ctx->panel);
		return -EPROBE_DEFER;
	}
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_pan_120hz, &ext_funcs, &ctx->panel);
	if (ret < 0) {
		printk("lqb [lcd_info]%s mtk_panel_ext_create fail LINE=%d ret=%d\n", __func__, __LINE__, ret);
		return ret;
	}
#endif
	//add proc/devinfo/lcd
	register_device_proc("lcd", "NT36532W", "novatek");
	oplus_enhance_mipi_strength = 2;

	printk("lqb [lcd_info]%s -\n", __func__);
	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
}

static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "oplus24921_nt36532w_2800_2000_dual_dsi_vdo_144hz_csot",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus24921_nt36532w_2800_2000_dual_dsi_vdo_144hz_csot",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("liuwenqi");
MODULE_DESCRIPTION("lcm novatek nt36532w Panel Driver");
MODULE_LICENSE("GPL v2");
