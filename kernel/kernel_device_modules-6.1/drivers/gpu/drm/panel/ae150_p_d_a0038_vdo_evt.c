// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#include <linux/regmap.h>
#include <linux/mfd/mt6363/registers.h>
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#include <soc/oplus/device_info.h>
#include "ktz8868.h"
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include "../mediatek/mediatek_v2/mtk_disp_notify.h"
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_CS_ON  0x19
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int bpc;

	/**
	 * @width_mm: width of the panel's active display area
	 * @height_mm: height of the panel's active display area
	 */
	struct {
		unsigned int width_mm;
		unsigned int height_mm;
	} size;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	const struct panel_init_cmd *init_cmds;
	unsigned int lanes;
};

enum led_mode {
	LED_MODE_BLS_NONE = 0,
	LED_MODE_BLS_VIRTUAL,
	LED_MODE_BLS_CABC
};

struct lcm {
	struct device *dev;
	struct mipi_dsi_device *dsi;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *master_esd_gpio;
	struct gpio_desc *slave_esd_gpio;
	struct gpio_desc *bias_enp_en;
	struct gpio_desc *bias_enn_en;
	struct gpio_desc *bias_en;
	struct gpio_desc *lcm_vddi_en;
	struct regulator *reg;
	struct regmap *pmic_regmap;
	bool prepared;
	bool enabled;
	int error;
	unsigned int gate_ic;
	bool display_dual_swap;
	enum led_mode bl_mode;
};

#define lcm_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})
#define lcm_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define  M_DELAY(n) usleep_range(n*1000, n*1000+100)
#define  U_DELAY(n) usleep_range(n, n+10)

#define HSA                (4)
#define HBP_90_48HZ        (60)
#define HBP_120_60HZ       (45)

#define HFP_90_48HZ        (217)
#define HFP_120_60HZ       (45)

#define VSA                (2)
#define VBP                (230)

#define VFP_120HZ          (56)
#define VFP_90HZ           (56)
#define VFP_60HZ           (2344)
#define VFP_48HZ           (2058)

#define VAC_FHD            (2000)
#define HAC_FHD            (2800)

#define LCM_LDO_KEEP_AWAKE_BIT BIT(3)

static int g_fps_current = 120;

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}


#define MAX_NORMAL_BRIGHTNESS			1884
#define MAX_HW_BRIGHTNESS			2047
static bool ktz8868_set_bl_flag = false;
unsigned int level_backup = 0;
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif
static bool is_pd_with_guesture = false;
extern unsigned long esd_flag;
extern bool g_shutdown;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static unsigned int backlight_map[] = {
	   0,   39,   78,  116,  155,  194,  233,  271,  310,  312,  313,  314,  316,  317,  319,  320,  321,  323,  324,  326,
	 327,  328,  330,  331,  333,  334,  335,  337,  338,  340,  341,  342,  344,  345,  347,  348,  349,  351,  352,  354,
	 355,  356,  358,  359,  361,  362,  363,  365,  366,  368,  369,  370,  372,  373,  375,  376,  377,  379,  380,  382,
	 383,  384,  386,  387,  388,  390,  391,  392,  394,  395,  396,  398,  399,  401,  402,  403,  405,  406,  407,  409,
	 410,  411,  413,  414,  416,  417,  418,  420,  421,  422,  424,  425,  426,  428,  429,  430,  432,  433,  435,  436,
	 437,  439,  440,  442,  443,  445,  447,  448,  449,  450,  451,  452,  454,  455,  456,  458,  459,  460,  462,  463,
	 464,  466,  467,  469,  470,  471,  473,  474,  475,  477,  478,  479,  481,  482,  484,  485,  486,  488,  489,  490,
	 492,  493,  494,  496,  497,  498,  500,  501,  503,  504,  505,  507,  508,  509,  511,  512,  513,  515,  516,  517,
	 519,  520,  521,  523,  524,  525,  527,  528,  529,  531,  532,  533,  535,  536,  537,  539,  540,  541,  543,  544,
	 545,  547,  548,  549,  550,  552,  553,  554,  556,  557,  558,  560,  561,  562,  564,  565,  566,  568,  569,  570,
	 572,  573,  574,  576,  577,  578,  580,  581,  582,  583,  585,  586,  587,  589,  590,  591,  592,  594,  595,  596,
	 598,  599,  600,  601,  603,  604,  605,  606,  608,  609,  610,  612,  613,  614,  615,  617,  618,  619,  621,  622,
	 623,  624,  626,  627,  628,  630,  631,  632,  633,  635,  636,  637,  638,  640,  641,  642,  644,  645,  646,  647,
	 649,  650,  651,  653,  654,  655,  656,  658,  659,  660,  662,  663,  664,  665,  667,  668,  669,  670,  672,  673,
	 674,  676,  677,  678,  679,  681,  682,  683,  685,  686,  687,  688,  690,  691,  692,  694,  695,  696,  697,  699,
	 700,  701,  702,  704,  705,  706,  708,  709,  710,  711,  713,  714,  715,  716,  718,  719,  720,  721,  723,  724,
	 725,  726,  728,  729,  730,  731,  732,  734,  735,  736,  737,  739,  740,  741,  742,  744,  745,  746,  747,  749,
	 750,  751,  752,  754,  755,  756,  757,  759,  760,  761,  762,  763,  765,  766,  767,  768,  770,  771,  772,  773,
	 775,  776,  777,  778,  779,  781,  782,  783,  784,  785,  787,  788,  789,  790,  791,  793,  794,  795,  796,  797,
	 799,  800,  801,  802,  803,  805,  806,  807,  808,  809,  811,  812,  813,  814,  815,  817,  818,  819,  820,  821,
	 823,  824,  825,  826,  827,  829,  830,  831,  832,  833,  835,  836,  837,  838,  839,  841,  842,  843,  844,  845,
	 847,  848,  849,  850,  851,  853,  854,  855,  856,  857,  859,  860,  861,  862,  863,  865,  866,  867,  868,  869,
	 871,  872,  873,  874,  875,  877,  878,  879,  880,  881,  883,  884,  885,  886,  887,  889,  890,  891,  892,  893,
	 894,  896,  897,  898,  899,  900,  901,  903,  904,  905,  906,  907,  908,  910,  911,  912,  913,  914,  915,  917,
	 918,  919,  920,  921,  922,  923,  925,  926,  927,  928,  929,  930,  932,  933,  934,  935,  936,  937,  939,  940,
	 941,  942,  943,  944,  946,  947,  948,  949,  950,  951,  952,  954,  955,  956,  957,  958,  959,  960,  961,  963,
	 964,  965,  966,  967,  968,  969,  970,  971,  973,  974,  975,  976,  977,  978,  979,  980,  982,  983,  984,  985,
	 986,  987,  988,  989,  991,  992,  993,  994,  995,  996,  997,  998,  999, 1001, 1002, 1003, 1004, 1005, 1006, 1007,
	1008, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1019, 1020, 1021, 1022, 1023, 1024, 1025, 1026, 1027, 1029, 1030,
	1031, 1032, 1033, 1034, 1035, 1036, 1038, 1039, 1040, 1041, 1042, 1043, 1044, 1045, 1047, 1048, 1049, 1050, 1051, 1052,
	1053, 1054, 1055, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1074,
	1075, 1076, 1077, 1078, 1079, 1081, 1082, 1083, 1084, 1085, 1086, 1087, 1088, 1089, 1090, 1091, 1092, 1094, 1095, 1096,
	1097, 1098, 1099, 1100, 1101, 1102, 1103, 1104, 1105, 1106, 1108, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117,
	1118, 1119, 1120, 1121, 1122, 1123, 1125, 1126, 1127, 1128, 1129, 1130, 1131, 1132, 1133, 1134, 1135, 1136, 1137, 1138,
	1139, 1140, 1141, 1142, 1143, 1144, 1145, 1146, 1147, 1148, 1149, 1151, 1152, 1153, 1154, 1155, 1156, 1157, 1158, 1159,
	1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1177, 1178, 1179, 1180,
	1181, 1182, 1183, 1184, 1185, 1186, 1187, 1188, 1189, 1190, 1191, 1192, 1193, 1194, 1195, 1196, 1197, 1198, 1199, 1200,
	1201, 1203, 1204, 1205, 1206, 1207, 1208, 1209, 1210, 1211, 1212, 1213, 1214, 1215, 1216, 1217, 1218, 1219, 1220, 1221,
	1222, 1223, 1224, 1225, 1226, 1227, 1228, 1229, 1230, 1231, 1232, 1233, 1234, 1235, 1236, 1237, 1238, 1239, 1240, 1241,
	1242, 1243, 1244, 1245, 1246, 1247, 1248, 1249, 1250, 1251, 1252, 1253, 1254, 1255, 1256, 1257, 1258, 1259, 1260, 1261,
	1262, 1263, 1264, 1265, 1266, 1267, 1268, 1269, 1270, 1271, 1272, 1273, 1274, 1275, 1276, 1277, 1278, 1279, 1280, 1281,
	1282, 1283, 1284, 1285, 1286, 1287, 1288, 1288, 1289, 1290, 1291, 1292, 1293, 1294, 1295, 1296, 1297, 1298, 1299, 1300,
	1301, 1302, 1303, 1304, 1305, 1306, 1307, 1308, 1309, 1310, 1311, 1312, 1312, 1313, 1314, 1315, 1316, 1317, 1318, 1319,
	1320, 1321, 1322, 1323, 1324, 1325, 1326, 1327, 1327, 1328, 1329, 1330, 1331, 1332, 1333, 1334, 1335, 1336, 1337, 1338,
	1338, 1339, 1340, 1341, 1342, 1343, 1344, 1345, 1346, 1347, 1348, 1349, 1350, 1350, 1351, 1352, 1353, 1354, 1355, 1356,
	1357, 1358, 1359, 1360, 1361, 1361, 1362, 1363, 1364, 1365, 1366, 1367, 1368, 1369, 1370, 1371, 1372, 1373, 1373, 1374,
	1375, 1376, 1377, 1378, 1379, 1380, 1381, 1382, 1383, 1384, 1384, 1385, 1386, 1387, 1388, 1389, 1390, 1391, 1392, 1393,
	1394, 1395, 1396, 1396, 1397, 1398, 1399, 1400, 1401, 1402, 1403, 1404, 1405, 1406, 1407, 1407, 1408, 1409, 1410, 1411,
	1412, 1413, 1414, 1415, 1416, 1416, 1417, 1418, 1419, 1420, 1421, 1422, 1423, 1423, 1424, 1425, 1426, 1427, 1428, 1429,
	1430, 1431, 1431, 1432, 1433, 1434, 1435, 1436, 1437, 1438, 1438, 1439, 1440, 1441, 1442, 1443, 1444, 1445, 1445, 1446,
	1447, 1448, 1449, 1450, 1451, 1452, 1453, 1453, 1454, 1455, 1456, 1457, 1458, 1458, 1459, 1460, 1461, 1462, 1463, 1464,
	1464, 1465, 1466, 1467, 1468, 1469, 1469, 1470, 1471, 1472, 1473, 1474, 1474, 1475, 1476, 1477, 1478, 1479, 1479, 1480,
	1481, 1482, 1483, 1484, 1485, 1485, 1486, 1487, 1488, 1489, 1490, 1490, 1491, 1492, 1493, 1494, 1495, 1495, 1496, 1497,
	1498, 1499, 1500, 1500, 1501, 1502, 1503, 1504, 1505, 1506, 1506, 1507, 1508, 1509, 1510, 1511, 1511, 1512, 1513, 1514,
	1515, 1516, 1516, 1517, 1518, 1519, 1520, 1521, 1521, 1522, 1523, 1524, 1525, 1526, 1527, 1527, 1528, 1529, 1530, 1531,
	1532, 1532, 1533, 1534, 1535, 1536, 1537, 1537, 1538, 1539, 1540, 1541, 1542, 1542, 1543, 1544, 1545, 1546, 1546, 1547,
	1548, 1549, 1550, 1550, 1551, 1552, 1553, 1554, 1554, 1555, 1556, 1557, 1558, 1558, 1559, 1560, 1561, 1562, 1562, 1563,
	1564, 1565, 1566, 1566, 1567, 1568, 1569, 1570, 1570, 1571, 1572, 1573, 1574, 1574, 1575, 1576, 1577, 1578, 1578, 1579,
	1580, 1581, 1581, 1582, 1583, 1584, 1584, 1585, 1586, 1587, 1587, 1588, 1589, 1590, 1591, 1591, 1592, 1593, 1594, 1594,
	1595, 1596, 1597, 1597, 1598, 1599, 1600, 1600, 1601, 1602, 1603, 1603, 1604, 1605, 1606, 1606, 1607, 1608, 1609, 1610,
	1610, 1611, 1612, 1613, 1613, 1614, 1615, 1616, 1616, 1617, 1618, 1619, 1619, 1620, 1621, 1622, 1622, 1623, 1624, 1625,
	1625, 1626, 1627, 1628, 1629, 1629, 1630, 1631, 1632, 1632, 1633, 1634, 1635, 1635, 1636, 1637, 1638, 1638, 1639, 1640,
	1641, 1641, 1642, 1643, 1644, 1644, 1645, 1646, 1647, 1648, 1648, 1649, 1650, 1651, 1651, 1652, 1653, 1654, 1654, 1655,
	1656, 1657, 1657, 1658, 1659, 1659, 1660, 1661, 1662, 1662, 1663, 1664, 1664, 1665, 1666, 1667, 1667, 1668, 1669, 1669,
	1670, 1671, 1672, 1672, 1673, 1674, 1675, 1675, 1676, 1677, 1677, 1678, 1679, 1680, 1680, 1681, 1682, 1682, 1683, 1684,
	1685, 1685, 1686, 1687, 1687, 1688, 1689, 1690, 1690, 1691, 1692, 1692, 1693, 1694, 1695, 1695, 1696, 1697, 1697, 1698,
	1699, 1700, 1700, 1701, 1702, 1702, 1703, 1704, 1705, 1705, 1706, 1707, 1707, 1708, 1709, 1709, 1710, 1711, 1712, 1712,
	1713, 1714, 1714, 1715, 1716, 1717, 1717, 1718, 1719, 1719, 1720, 1721, 1722, 1722, 1723, 1724, 1724, 1725, 1726, 1726,
	1727, 1728, 1729, 1729, 1730, 1731, 1731, 1732, 1733, 1734, 1734, 1735, 1736, 1736, 1737, 1738, 1739, 1739, 1740, 1741,
	1741, 1742, 1743, 1743, 1744, 1745, 1746, 1746, 1747, 1748, 1748, 1749, 1750, 1751, 1751, 1752, 1753, 1753, 1754, 1755,
	1756, 1756, 1757, 1758, 1758, 1759, 1760, 1760, 1761, 1762, 1762, 1763, 1764, 1764, 1765, 1766, 1766, 1767, 1768, 1768,
	1769, 1770, 1770, 1771, 1772, 1772, 1773, 1774, 1774, 1775, 1776, 1776, 1777, 1778, 1778, 1779, 1780, 1780, 1781, 1782,
	1782, 1783, 1784, 1784, 1785, 1786, 1786, 1787, 1788, 1788, 1789, 1790, 1790, 1791, 1792, 1792, 1793, 1794, 1794, 1795,
	1795, 1796, 1797, 1797, 1798, 1799, 1799, 1800, 1800, 1801, 1802, 1802, 1803, 1804, 1804, 1805, 1805, 1806, 1807, 1807,
	1808, 1809, 1809, 1810, 1810, 1811, 1812, 1812, 1813, 1814, 1814, 1815, 1815, 1816, 1817, 1817, 1818, 1819, 1819, 1820,
	1820, 1821, 1822, 1822, 1823, 1824, 1824, 1825, 1825, 1826, 1827, 1827, 1828, 1829, 1829, 1830, 1830, 1831, 1832, 1832,
	1833, 1834, 1834, 1835, 1835, 1836, 1837, 1837, 1838, 1839, 1839, 1840, 1840, 1841, 1842, 1842, 1843, 1844, 1844, 1845,
	1845, 1846, 1847, 1847, 1848, 1849, 1849, 1850, 1850, 1851, 1852, 1852, 1853, 1853, 1854, 1855, 1855, 1856, 1856, 1857,
	1857, 1858, 1859, 1859, 1860, 1860, 1861, 1862, 1862, 1863, 1863, 1864, 1864, 1865, 1866, 1866, 1867, 1867, 1868, 1869,
	1869, 1870, 1870, 1871, 1871, 1872, 1873, 1873, 1874, 1874, 1875, 1876, 1876, 1877, 1877, 1878, 1878, 1879, 1880, 1880,
	1881, 1881, 1882, 1882, 1883, 1883, 1884, 1884, 1885, 1886, 1886, 1887, 1887, 1888, 1888, 1889, 1889, 1890, 1890, 1891,
	1891, 1892, 1893, 1893, 1894, 1894, 1895, 1895, 1896, 1896, 1897, 1897, 1898, 1899, 1899, 1900, 1900, 1901, 1901, 1902,
	1902, 1903, 1903, 1904, 1904, 1905, 1906, 1906, 1907, 1907, 1908, 1908, 1909, 1909, 1910, 1910, 1911, 1912, 1912, 1913,
	1913, 1914, 1914, 1915, 1915, 1916, 1916, 1917, 1917, 1918, 1919, 1919, 1920, 1920, 1921, 1921, 1922, 1922, 1923, 1923,
	1924, 1925, 1925, 1926, 1926, 1927, 1927, 1928, 1928, 1929, 1929, 1930, 1930, 1931, 1932, 1932, 1933, 1933, 1934, 1934,
	1935, 1935, 1936, 1936, 1937, 1937, 1938, 1938, 1939, 1939, 1940, 1940, 1941, 1941, 1942, 1942, 1943, 1943, 1944, 1944,
	1945, 1945, 1946, 1946, 1947, 1947, 1948, 1948, 1949, 1949, 1950, 1950, 1951, 1951, 1952, 1952, 1953, 1953, 1954, 1954,
	1955, 1955, 1955, 1956, 1956, 1957, 1957, 1958, 1958, 1959, 1959, 1960, 1960, 1961, 1961, 1961, 1962, 1962, 1963, 1963,
	1964, 1964, 1965, 1965, 1966, 1966, 1966, 1967, 1967, 1968, 1968, 1969, 1969, 1970, 1970, 1971, 1971, 1972, 1972, 1972,
	1973, 1973, 1974, 1974, 1975, 1975, 1976, 1976, 1977, 1977, 1977, 1978, 1978, 1979, 1979, 1980, 1980, 1981, 1981, 1982,
	1982, 1983, 1983, 1983, 1984, 1984, 1985, 1985, 1986, 1986, 1987, 1987, 1988, 1988, 1988, 1989, 1989, 1990, 1990, 1991,
	1991, 1992, 1992, 1993, 1993, 1994, 1994, 1994, 1995, 1995, 1996, 1996, 1997, 1997, 1998, 1998, 1999, 1999, 1999, 2000,
	2000, 2001, 2001, 2002, 2002, 2002, 2003, 2003, 2004, 2004, 2004, 2005, 2005, 2006, 2006, 2007, 2007, 2007, 2008, 2008,
	2009, 2009, 2009, 2010, 2010, 2011, 2011, 2012, 2012, 2012, 2013, 2013, 2014, 2014, 2014, 2015, 2015, 2016, 2016, 2017,
	2017, 2017, 2018, 2018, 2019, 2019, 2019, 2019, 2020, 2020, 2020, 2020, 2020, 2020, 2021, 2021, 2021, 2021, 2021, 2021,
	2022, 2022, 2022, 2022, 2022, 2022, 2023, 2023, 2023, 2023, 2023, 2023, 2024, 2024, 2024, 2024, 2024, 2025, 2025, 2025,
	2025, 2025, 2025, 2026, 2026, 2026, 2026, 2026, 2026, 2027, 2027, 2027, 2027, 2027, 2027, 2028, 2028, 2028, 2028, 2028,
	2029, 2029, 2029, 2029, 2029, 2029, 2030, 2030, 2030, 2030, 2030, 2030, 2031, 2031, 2031, 2031, 2031, 2031, 2032, 2032,
	2032, 2032, 2032, 2032, 2033, 2033, 2033, 2033, 2033, 2034, 2034, 2034, 2034, 2034, 2034, 2035, 2035, 2035, 2035, 2035,
	2035, 2036, 2036, 2036, 2036, 2036, 2036, 2037, 2037, 2037, 2037, 2037, 2037, 2038, 2038, 2038, 2038, 2038, 2039, 2039,
	2039, 2039, 2039, 2039, 2040, 2040, 2040, 2040, 2040, 2040, 2041, 2041, 2041, 2041, 2041, 2041, 2042, 2042, 2042, 2042,
	2042, 2043, 2043, 2043, 2043, 2043, 2043, 2044, 2044, 2044, 2044, 2044, 2044, 2045, 2045, 2045, 2045, 2045, 2045, 2046,
	2046, 2046, 2046, 2046, 2046, 2047, 2047, 2047
};

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static int ktz8868_set_brightness_level(unsigned int bl_lvl)
{
	if (bl_lvl > 0) {
		ktz8868_write_byte(0x04, bl_lvl & 0x07);
		ktz8868_write_byte(0x05, (bl_lvl >> 3) & 0xFF);
		if (ktz8868_set_bl_flag == false) {
			mdelay(15);
			ktz8868_write_byte(0x01, 0x01);
			ktz8868_set_bl_flag = true;
		}
	}

	if (bl_lvl == 0) {
		ktz8868_write_byte(0x01, 0x00);
		mdelay(9);
		ktz8868_write_byte(0x04, 0x00);
		ktz8868_write_byte(0x05, 0x00);
		if(ktz8868_set_bl_flag == true) {
			ktz8868_set_bl_flag = false;
		}
	}
	pr_info("[lcd_info]%s: bl_lvl=%d\n", __func__, bl_lvl);

	return 0;
}

static int ktz8868_backlight_config(struct drm_panel *panel, bool enable)
{
	struct lcm *ctx = panel_to_lcm(panel);
	unsigned int i2c_maped_level;

	if (enable) {
		if (ctx->bl_mode == LED_MODE_BLS_VIRTUAL) {
			ktz8868_write_byte(0x02, 0xD2); //i2c mode 34v
		} else if (ctx->bl_mode == LED_MODE_BLS_CABC) {
			ktz8868_write_byte(0x02, 0xD3); //i2c & pwm mode 34v 0xD3(Exp. Mode) 0xDB(Lin. Mode)
		}
		ktz8868_write_byte(0x03, 0xCD);
		ktz8868_write_byte(0x11, 0x76);
		ktz8868_write_byte(0x15, 0x88); /* 18.8mA */
		ktz8868_write_byte(0x08, 0xFF);
		if(esd_flag){/* esd recovery write brightness to ktz8868*/
			i2c_maped_level = backlight_map[level_backup];
			ktz8868_set_brightness_level(i2c_maped_level);
			printk("[lcd_info]%s: -- i2c_maped_level = %d \n", __func__,i2c_maped_level);
		}
	} else {
		ktz8868_write_byte(0x08, 0x00);
	}

	pr_info("[lcd_info]%s: enable=%d bl_mode =%d\n", __func__, enable, ctx->bl_mode);

	return 0;
}


/* ktz8868 lcd base config */
static void ktz8868_lcd_bias_config(struct drm_panel *panel, int enable)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s: ++ enable=%d\n", __func__, enable);
	ctx->bias_enp_en = devm_gpiod_get(ctx->dev, "bias-enp-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enp_en)) {
		printk("[lcd_info][error]%s: could not get bias_enp_en gpio line=%d\n", __func__, __LINE__);
		return;
	}

	ctx->bias_enn_en = devm_gpiod_get(ctx->dev, "bias-enn-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enn_en)) {
		printk("[lcd_info][error]%s: could not get bias_enn_en gpio line=%d\n", __func__, __LINE__);
		return;
	}

	if(!enable){
		/* Disable ENN */
		ktz8868_write_byte(0x09, 0x9C);
		printk("[lcd_info]%s: bias_enn_en 0\n", __func__);
		usleep_range(5000, 5010);
		/* Disable ENP */
		ktz8868_write_byte(0x09, 0x98);
		printk("[lcd_info]%s: bias_enp_en 0\n", __func__);
	} else {
		/* only config i2c0*/
		/* LCD_BOOST_CFG */
		ktz8868_write_byte(0x0C, 0x32);
		/* OUTP_CFG，OUTP = 6.0V */
		ktz8868_write_byte(0x0D, 0x28);
		/* OUTN_CFG，OUTN = -6.0V */
		ktz8868_write_byte(0x0E, 0x28);
		/* enable OUTN and OUTP via I2C Ctrl */
		ktz8868_write_byte(0x09, 0x98);
		/* enable ENP */
		ktz8868_write_byte(0x09, 0x9C);
		printk("[lcd_info]%s: bias_enp_en 1\n", __func__);
		usleep_range(5000, 5010);
		/* enable ENN */
		ktz8868_write_byte(0x09, 0x9E);
		usleep_range(10 * 1000, 15 * 1000); /* 10ms */
		printk("[lcd_info]%s: bias_enn_en 1\n", __func__);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enp_en);
	devm_gpiod_put(ctx->dev, ctx->bias_enn_en);

	printk("[lcd_info]%s: --\n", __func__);
}

/* VDDI Ctrl */
static int lcm_enable_vddi(struct drm_panel *panel, int enable)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s: ++\n", __func__);
	#if 0
	ctx->lcm_vddi_en = devm_gpiod_get(ctx->dev, "lcm-vddi-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->lcm_vddi_en)) {
		pr_err("[lcd_info][error]%s: could not get lcm_vddi_en gpio line=%d\n", __func__, __LINE__);
		return -1;
	}
	#endif
	if (enable) {
		gpiod_set_value(ctx->lcm_vddi_en, 1);
	} else {
			if (is_pd_with_guesture) {
				pr_info("[lcd_info]%s: vddi tp guesture enable, not disable backlight ic\n", __func__);
				devm_gpiod_put(ctx->dev, ctx->bias_en);
				return 0;
			}
		gpiod_set_value(ctx->lcm_vddi_en, 0);
	}
	printk("[lcd_info]%s: --\n", __func__);
	return 0;
}

/* backlight ic is ktz8868 */
static int lcm_backlight_ic_config(struct drm_panel *panel, int enable)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s: ++\n", __func__);
	ctx->bias_en = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_en)) {
		pr_err("[lcd_info][error]%s: could not get pm-enable gpio line=%d\n", __func__, __LINE__);
		return -1;
	}

	if (enable) {
		gpiod_set_value(ctx->bias_en, 1);
		usleep_range(125, 130);

		if (!is_pd_with_guesture) {
			/* lcd bias config enable  */
			ktz8868_lcd_bias_config(panel, true);
		}
		/* lcd brightness config enable*/
		ktz8868_backlight_config(panel, true);

	} else {
			if (is_pd_with_guesture) {
				pr_info("[lcd_info]%s: tp guesture enable, not disable backlight ic\n", __func__);
				ktz8868_backlight_config(panel, false);
				devm_gpiod_put(ctx->dev, ctx->bias_en);
				return 0;
			}
		ktz8868_lcd_bias_config(panel, false);
		ktz8868_backlight_config(panel, false);
		gpiod_set_value(ctx->bias_en, 0);
		msleep(20);
		/* if ktz8868 will shutdown, we shoudle set bl flag to false */
		ktz8868_set_bl_flag = false;
	}

	devm_gpiod_put(ctx->dev, ctx->bias_en);

	printk("[lcd_info]%s: --\n", __func__);
	return 0;
}
/*
int oplus25682_i2c_set_backlight(unsigned int level)
{
	if (level > MAX_NORMAL_BRIGHTNESS)
		level = MAX_NORMAL_BRIGHTNESS;

	level_backup = level;
	oplus_display_brightness = level;
	printk("[lcd_info]%s: bl_level:%d, mapping value = %d\n", __func__, level, backlight_map[level]);

	level = backlight_map[level];
	ktz8868_set_brightness_level(level);

	return 0;
}
EXPORT_SYMBOL(oplus25682_i2c_set_backlight);
*/

/* rc_buf_thresh will right shift 6bits (which means the values here will be divided by 64)
 * when setting to PPS8~PPS11 registers in mtk_dsc_config() function, so the original values
 * need left sihft 6bit (which means the original values are multiplied by 64), so that
 * PPS8~PPS11 registers can get right setting
 */
static unsigned int rc_buf_thresh[14] = {
//The original values VS values multiplied by 64
//14, 28,  42,	 56,   70,	 84,   98,	 105,  112,  119,  121,  123,  125,  126
896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int range_min_qp[15] = {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 9, 12};
static unsigned int range_max_qp[15] = {4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13};
static int range_bpg_ofs[15] = {2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s +\n", __func__);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (is_pd_with_guesture) {
		gpiod_set_value(ctx->reset_gpio, 0);
		usleep_range(5 * 1000, 10 * 1000);
		gpiod_set_value(ctx->reset_gpio, 1);
		usleep_range(10 * 1000, 15 * 1000);
		gpiod_set_value(ctx->reset_gpio, 0);
		usleep_range(10 * 1000, 15 * 1000);
		gpiod_set_value(ctx->reset_gpio, 1);
		usleep_range(100 * 1000, 110 * 1000);
	} else {
		gpiod_set_value(ctx->reset_gpio, 1);
		usleep_range(10 * 1000, 15 * 1000);
		gpiod_set_value(ctx->reset_gpio, 0);
		usleep_range(10 * 1000, 15 * 1000);
		gpiod_set_value(ctx->reset_gpio, 1);
		usleep_range(100 * 1000, 110 * 1000);
	}
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	//CABC start
	lcm_dcs_write_seq_static(ctx,0xFF,0x23);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x00,0x60);
	lcm_dcs_write_seq_static(ctx,0x07,0x20);
	lcm_dcs_write_seq_static(ctx,0x08,0x01);
	lcm_dcs_write_seq_static(ctx,0x09,0x68);
	lcm_dcs_write_seq_static(ctx,0x11,0x03);
	lcm_dcs_write_seq_static(ctx,0x12,0x6B);
	lcm_dcs_write_seq_static(ctx,0x13,0x00);
	lcm_dcs_write_seq_static(ctx,0x15,0x03);
	lcm_dcs_write_seq_static(ctx,0x16,0x10);
	lcm_dcs_write_seq_static(ctx,0x0A,0xAA);
	lcm_dcs_write_seq_static(ctx,0x0B,0xAA);
	lcm_dcs_write_seq_static(ctx,0x0C,0xD5);
	lcm_dcs_write_seq_static(ctx,0x0D,0x05);
	lcm_dcs_write_seq_static(ctx,0x19,0x18);
	lcm_dcs_write_seq_static(ctx,0x1A,0x18);
	lcm_dcs_write_seq_static(ctx,0x1B,0x18);
	lcm_dcs_write_seq_static(ctx,0x1C,0x18);
	lcm_dcs_write_seq_static(ctx,0x1D,0x1A);
	lcm_dcs_write_seq_static(ctx,0x1E,0x1A);
	lcm_dcs_write_seq_static(ctx,0x1F,0x1B);
	lcm_dcs_write_seq_static(ctx,0x20,0x1D);
	lcm_dcs_write_seq_static(ctx,0x21,0x1E);
	lcm_dcs_write_seq_static(ctx,0x22,0x21);
	lcm_dcs_write_seq_static(ctx,0x23,0x27);
	lcm_dcs_write_seq_static(ctx,0x24,0x2B);
	lcm_dcs_write_seq_static(ctx,0x25,0x2F);
	lcm_dcs_write_seq_static(ctx,0x26,0x34);
	lcm_dcs_write_seq_static(ctx,0x27,0x39);
	lcm_dcs_write_seq_static(ctx,0x28,0x3E);
	lcm_dcs_write_seq_static(ctx,0x2A,0x32);
	lcm_dcs_write_seq_static(ctx,0x2B,0x3F);

	//UI MODE 55=01
	lcm_dcs_write_seq_static(ctx,0x30,0xFA);
	lcm_dcs_write_seq_static(ctx,0x31,0xF7);
	lcm_dcs_write_seq_static(ctx,0x32,0xF5);
	lcm_dcs_write_seq_static(ctx,0x33,0xF2);
	lcm_dcs_write_seq_static(ctx,0x34,0xF0);
	lcm_dcs_write_seq_static(ctx,0x35,0xED);
	lcm_dcs_write_seq_static(ctx,0x36,0xEB);
	lcm_dcs_write_seq_static(ctx,0x37,0xE8);
	lcm_dcs_write_seq_static(ctx,0x38,0xE6);
	lcm_dcs_write_seq_static(ctx,0x39,0xE2);
	lcm_dcs_write_seq_static(ctx,0x3A,0xDE);
	lcm_dcs_write_seq_static(ctx,0x3B,0xDB);
	lcm_dcs_write_seq_static(ctx,0x3D,0xD7);
	lcm_dcs_write_seq_static(ctx,0x3F,0xD3);
	lcm_dcs_write_seq_static(ctx,0x40,0xD0);
	lcm_dcs_write_seq_static(ctx,0x41,0xCC);

	//STILL MODE 55=02
	lcm_dcs_write_seq_static(ctx,0x45,0xF7);
	lcm_dcs_write_seq_static(ctx,0x46,0xEF);
	lcm_dcs_write_seq_static(ctx,0x47,0xE7);
	lcm_dcs_write_seq_static(ctx,0x48,0xDE);
	lcm_dcs_write_seq_static(ctx,0x49,0xD5);
	lcm_dcs_write_seq_static(ctx,0x4A,0xCD);
	lcm_dcs_write_seq_static(ctx,0x4B,0xC4);
	lcm_dcs_write_seq_static(ctx,0x4C,0xBB);
	lcm_dcs_write_seq_static(ctx,0x4D,0xB3);
	lcm_dcs_write_seq_static(ctx,0x4E,0xAF);
	lcm_dcs_write_seq_static(ctx,0x4F,0xAC);
	lcm_dcs_write_seq_static(ctx,0x50,0xA9);
	lcm_dcs_write_seq_static(ctx,0x51,0xA5);
	lcm_dcs_write_seq_static(ctx,0x52,0xA2);
	lcm_dcs_write_seq_static(ctx,0x53,0x9F);
	lcm_dcs_write_seq_static(ctx,0x54,0x9C);

	//MOVING MODE 55=03
	lcm_dcs_write_seq_static(ctx,0x58,0xF5);
	lcm_dcs_write_seq_static(ctx,0x59,0xEB);
	lcm_dcs_write_seq_static(ctx,0x5A,0xE1);
	lcm_dcs_write_seq_static(ctx,0x5B,0xD7);
	lcm_dcs_write_seq_static(ctx,0x5C,0xCC);
	lcm_dcs_write_seq_static(ctx,0x5D,0xC2);
	lcm_dcs_write_seq_static(ctx,0x5E,0xB8);
	lcm_dcs_write_seq_static(ctx,0x5F,0xAD);
	lcm_dcs_write_seq_static(ctx,0x60,0xA3);
	lcm_dcs_write_seq_static(ctx,0x61,0x9C);
	lcm_dcs_write_seq_static(ctx,0x62,0x92);
	lcm_dcs_write_seq_static(ctx,0x63,0x8B);
	lcm_dcs_write_seq_static(ctx,0x64,0x85);
	lcm_dcs_write_seq_static(ctx,0x65,0x7E);
	lcm_dcs_write_seq_static(ctx,0x66,0x77);
	lcm_dcs_write_seq_static(ctx,0x67,0x70);
	lcm_dcs_write_seq_static(ctx,0x6E,0x00);
	lcm_dcs_write_seq_static(ctx,0x6F,0x00);
	lcm_dcs_write_seq_static(ctx,0x70,0x00);
	lcm_dcs_write_seq_static(ctx,0x71,0x00);
	// CABC end


	lcm_dcs_write_seq_static(ctx,0xFF,0xE0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xCC,0x11);

	lcm_dcs_write_seq_static(ctx,0xFF,0xF0);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x75,0x33,0x01,0x03);
	lcm_dcs_write_seq_static(ctx,0xE4,0x10);
	lcm_dcs_write_seq_static(ctx,0xFF,0x20);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00,0x00,0x00,0x21,0x00,0x4E,0x00,0x6E,0x00,0x8C,0x00,0xA4,0x00,0xBC,0x00,0xCE);
	lcm_dcs_write_seq_static(ctx,0xB1,0x00,0xE1,0x01,0x1B,0x01,0x46,0x01,0x88,0x01,0xBB,0x02,0x08,0x02,0x44,0x02,0x45);
	lcm_dcs_write_seq_static(ctx,0xB2,0x02,0x7F,0x02,0xBC,0x02,0xE5,0x03,0x1C,0x03,0x3F,0x03,0x6A,0x03,0x79,0x03,0x88);
	lcm_dcs_write_seq_static(ctx,0xB3,0x03,0x99,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB4,0x00,0x00,0x00,0x20,0x00,0x4D,0x00,0x70,0x00,0x8C,0x00,0xA5,0x00,0xBA,0x00,0xCE);
	lcm_dcs_write_seq_static(ctx,0xB5,0x00,0xDF,0x01,0x18,0x01,0x44,0x01,0x86,0x01,0xB7,0x02,0x02,0x02,0x3D,0x02,0x3E);
	lcm_dcs_write_seq_static(ctx,0xB6,0x02,0x78,0x02,0xB7,0x02,0xE0,0x03,0x19,0x03,0x3C,0x03,0x67,0x03,0x79,0x03,0x88);
	lcm_dcs_write_seq_static(ctx,0xB7,0x03,0x99,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB8,0x00,0x00,0x00,0x21,0x00,0x4F,0x00,0x76,0x00,0x97,0x00,0xB2,0x00,0xC9,0x00,0xDE);
	lcm_dcs_write_seq_static(ctx,0xB9,0x00,0xF1,0x01,0x2A,0x01,0x55,0x01,0x96,0x01,0xC6,0x02,0x0F,0x02,0x49,0x02,0x4A);
	lcm_dcs_write_seq_static(ctx,0xBA,0x02,0x83,0x02,0xC0,0x02,0xE9,0x03,0x22,0x03,0x47,0x03,0x77,0x03,0x79,0x03,0x89);
	lcm_dcs_write_seq_static(ctx,0xBB,0x03,0x9A,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xC6,0x26);
	lcm_dcs_write_seq_static(ctx,0xC7,0x22);
	lcm_dcs_write_seq_static(ctx,0xC8,0x33);
	lcm_dcs_write_seq_static(ctx,0xC9,0x22);
	lcm_dcs_write_seq_static(ctx,0xCA,0x21);
	lcm_dcs_write_seq_static(ctx,0xCB,0x10);
	lcm_dcs_write_seq_static(ctx,0xCC,0x31);
	lcm_dcs_write_seq_static(ctx,0xCD,0x63);
	lcm_dcs_write_seq_static(ctx,0xCE,0xA2);
	lcm_dcs_write_seq_static(ctx,0xCF,0xB6);
	lcm_dcs_write_seq_static(ctx,0xD0,0xC4);
	lcm_dcs_write_seq_static(ctx,0xD1,0xE2);
	lcm_dcs_write_seq_static(ctx,0xD2,0x26);
	lcm_dcs_write_seq_static(ctx,0xD3,0x22);
	lcm_dcs_write_seq_static(ctx,0xD4,0x33);
	lcm_dcs_write_seq_static(ctx,0xD5,0x21);
	lcm_dcs_write_seq_static(ctx,0xD6,0x21);
	lcm_dcs_write_seq_static(ctx,0xD7,0x00);
	lcm_dcs_write_seq_static(ctx,0xD8,0x22);
	lcm_dcs_write_seq_static(ctx,0xD9,0x63);
	lcm_dcs_write_seq_static(ctx,0xDA,0x82);
	lcm_dcs_write_seq_static(ctx,0xDB,0xA7);
	lcm_dcs_write_seq_static(ctx,0xDC,0xA4);
	lcm_dcs_write_seq_static(ctx,0xDD,0xE2);
	lcm_dcs_write_seq_static(ctx,0xDE,0x26);
	lcm_dcs_write_seq_static(ctx,0xDF,0x22);
	lcm_dcs_write_seq_static(ctx,0xE0,0x33);
	lcm_dcs_write_seq_static(ctx,0xE1,0x22);
	lcm_dcs_write_seq_static(ctx,0xE2,0x21);
	lcm_dcs_write_seq_static(ctx,0xE3,0x00);
	lcm_dcs_write_seq_static(ctx,0xE4,0x22);
	lcm_dcs_write_seq_static(ctx,0xE5,0x63);
	lcm_dcs_write_seq_static(ctx,0xE6,0x82);
	lcm_dcs_write_seq_static(ctx,0xE7,0xA7);
	lcm_dcs_write_seq_static(ctx,0xE8,0xA4);
	lcm_dcs_write_seq_static(ctx,0xE9,0xE2);
	lcm_dcs_write_seq_static(ctx,0xFF,0x21);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00,0x00,0x00,0x21,0x00,0x4E,0x00,0x6E,0x00,0x8C,0x00,0xA4,0x00,0xBC,0x00,0xCE);
	lcm_dcs_write_seq_static(ctx,0xB1,0x00,0xE1,0x01,0x1B,0x01,0x46,0x01,0x88,0x01,0xBB,0x02,0x08,0x02,0x44,0x02,0x45);
	lcm_dcs_write_seq_static(ctx,0xB2,0x02,0x7F,0x02,0xBC,0x02,0xE5,0x03,0x1C,0x03,0x3F,0x03,0x6A,0x03,0x79,0x03,0x88);
	lcm_dcs_write_seq_static(ctx,0xB3,0x03,0x99,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB4,0x00,0x00,0x00,0x20,0x00,0x4D,0x00,0x70,0x00,0x8C,0x00,0xA5,0x00,0xBA,0x00,0xCE);
	lcm_dcs_write_seq_static(ctx,0xB5,0x00,0xDF,0x01,0x18,0x01,0x44,0x01,0x86,0x01,0xB7,0x02,0x02,0x02,0x3D,0x02,0x3E);
	lcm_dcs_write_seq_static(ctx,0xB6,0x02,0x78,0x02,0xB7,0x02,0xE0,0x03,0x19,0x03,0x3C,0x03,0x67,0x03,0x79,0x03,0x88);
	lcm_dcs_write_seq_static(ctx,0xB7,0x03,0x99,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB8,0x00,0x00,0x00,0x21,0x00,0x4F,0x00,0x76,0x00,0x97,0x00,0xB2,0x00,0xC9,0x00,0xDE);
	lcm_dcs_write_seq_static(ctx,0xB9,0x00,0xF1,0x01,0x2A,0x01,0x55,0x01,0x96,0x01,0xC6,0x02,0x0F,0x02,0x49,0x02,0x4A);
	lcm_dcs_write_seq_static(ctx,0xBA,0x02,0x83,0x02,0xC0,0x02,0xE9,0x03,0x22,0x03,0x47,0x03,0x77,0x03,0x79,0x03,0x89);
	lcm_dcs_write_seq_static(ctx,0xBB,0x03,0x9A,0x03,0xAB,0x03,0xC0,0x03,0xD6,0x03,0xE5,0x03,0xFF,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x3B,0x03,0xE8,0x38,0x04,0x04,0x00);
	lcm_dcs_write_seq_static(ctx,0x90,0x13);
	lcm_dcs_write_seq_static(ctx,0x91,0x89,0xA8,0x00,0x14,0xD2,0x00,0x00,0x00,0x02,0x98,0x00,0x13,0x05,0x7A,0x01,0xF3);
	lcm_dcs_write_seq_static(ctx,0x92,0x10,0xE0);
	lcm_dcs_write_seq_static(ctx,0x9D,0x01);
	lcm_dcs_write_seq_static(ctx,0x51,0x07,0xFF);
	if(esd_flag)
		lcm_dcs_write_seq_static(ctx,0x53,0x2c); /* esd recovery open CABC*/
	if(g_fps_current == 120 || g_fps_current == 60){
	    lcm_dcs_write_seq_static(ctx,0xB2,0x91);
	    lcm_dcs_write_seq_static(ctx,0xB3,0x00);
	}
	if(g_fps_current == 90 || g_fps_current == 48){
	    lcm_dcs_write_seq_static(ctx,0xB2,0x80);
	    lcm_dcs_write_seq_static(ctx,0xB3,0x40);
	}
	//cabc switch
	lcm_dcs_write_seq_static(ctx,0xB9,0x00);
	lcm_dcs_write_seq_static(ctx,0x55,0x01);  /*defualt UI mode*/
	lcm_dcs_write_seq_static(ctx,0xB9,0x02);

	lcm_dcs_write_seq_static(ctx,0x35,0x00);

	lcm_dcs_write_seq_static(ctx,0xFF,0x27);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0xD0,0x31);
	lcm_dcs_write_seq_static(ctx,0xD1,0x54);
	lcm_dcs_write_seq_static(ctx,0xDE,0x42);
	lcm_dcs_write_seq_static(ctx,0xDF,0x02);

	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0xFB,0x01);
	lcm_dcs_write_seq_static(ctx,0x11);
	M_DELAY(120);
	lcm_dcs_write_seq_static(ctx,0x29);
	M_DELAY(10);
	pr_info("%s -\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s+++\n", __func__);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	pr_info("%s---\n", __func__);

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	int blank = 0;
#endif

	pr_info("%s+++\n", __func__);

	if (!ctx->prepared)
		return 0;

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown == 0) && (esd_flag == 0)) {
		is_pd_with_guesture = true;
	} else {
		is_pd_with_guesture = false;
	}
#endif

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (is_pd_with_guesture) {
		pr_info("[TP] tp gesture is enable, Display not to poweroff vddi\n");
	} else {
		blank = LCD_CTL_CS_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_info("[TP]TP CS will change to gpio mode and low\n");
	}
#endif

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(5 * 1000, 10 * 1000);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(110 * 1000, 115 * 1000);

	if (is_pd_with_guesture) {

	} else {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		usleep_range(5 * 1000, 5 * 1000);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	}

	ret = lcm_backlight_ic_config(panel, 0);
	if(ret) {
		printk("[lcd_info][error]%s: set bl_bias disable failed! ret=%d line=%d\n", __func__, ret, __LINE__);
	}

	if (!IS_ERR_OR_NULL(ctx->pmic_regmap))
		regmap_set_bits(ctx->pmic_regmap, MT6363_BUCK_VS1_VOTER_CON1_CLR, LCM_LDO_KEEP_AWAKE_BIT);

	/* ret = lcm_enable_vddi(panel, 0); */
	if(ret) {
		pr_err("[lcd_info][error]%s: set vddio off failed! ret=%d line=%d\n", __func__, ret, __LINE__);
	}

	ctx->error = 0;
	ctx->prepared = false;

	pr_info("%s---\n", __func__);

	return 0;
}
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	int blank = 0;
#endif

	pr_info("%s+++\n", __func__);

	if (ctx->prepared)
		return 0;

	if (!IS_ERR_OR_NULL(ctx->pmic_regmap))
		regmap_set_bits(ctx->pmic_regmap, MT6363_BUCK_VS1_VOTER_CON1_SET, LCM_LDO_KEEP_AWAKE_BIT);

	ret = lcm_backlight_ic_config(panel, 1);
	if(ret) {
		printk("[lcd_info][error]%s: set bl_bias enable failed! ret=%d line=%d\n", __func__, ret, __LINE__);
	}

	lcm_panel_init(ctx);
	ret = ctx->error;
	if (ret < 0) {
		pr_info("Send initial code error!\n");
		lcm_unprepare(panel);
	}

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

#if IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_info("[TP] spi CS set to high\n");
#endif
	pr_info("%s---\n", __func__);

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s+++\n", __func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	pr_info("%s---\n", __func__);

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 794576, //2894*2288*120 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_120_60HZ,
	.hsync_end = HAC_FHD + HFP_120_60HZ + HSA,
	.htotal = HAC_FHD + HFP_120_60HZ + HSA + HBP_120_60HZ,//2894
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_120HZ,
	.vsync_end = VAC_FHD + VFP_120HZ + VSA,
	.vtotal = VAC_FHD + VFP_120HZ + VSA + VBP, //2288
	.hskew = 1,
};

static const struct drm_display_mode performance_mode_90hz = {
	.clock = 634439, //3081*2288*90 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_90_48HZ,
	.hsync_end = HAC_FHD + HFP_90_48HZ + HSA,
	.htotal = HAC_FHD + HFP_90_48HZ + HSA + HBP_90_48HZ,//3081
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_90HZ,
	.vsync_end = VAC_FHD + VFP_90HZ + VSA,
	.vtotal = VAC_FHD + VFP_90HZ + VSA + VBP, //2288
	.hskew = 1,
};

static const struct drm_display_mode performance_mode_60hz = {
	.clock = 794576, //2894*4576*60 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_120_60HZ,
	.hsync_end = HAC_FHD + HFP_120_60HZ + HSA,
	.htotal = HAC_FHD + HFP_120_60HZ + HSA + HBP_120_60HZ,//2894
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_60HZ,
	.vsync_end = VAC_FHD + VFP_60HZ + VSA,
	.vtotal = VAC_FHD + VFP_60HZ + VSA + VBP, //4576
	.hskew = 1,
};

static const struct drm_display_mode performance_mode_48hz = {
	.clock = 634439, //3081*4290*48 htotal*vtotal*fps
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP_90_48HZ,
	.hsync_end = HAC_FHD + HFP_90_48HZ + HSA,
	.htotal = HAC_FHD + HFP_90_48HZ + HSA + HBP_90_48HZ,//3081
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP_48HZ,
	.vsync_end = VAC_FHD + VFP_48HZ + VSA,
	.vtotal = VAC_FHD + VFP_48HZ + VSA + VBP, //4290
	.hskew = 1,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 463,
	.data_rate = 926,
	.data_rate_khz = 926924,
	.physical_width_um = 239988,
	.physical_height_um = 171420,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.dual_swap = false,
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.lcm_esd_check_table[0] = {
		.cmd = 0x53,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.dsc_params = {
		.enable = 1,
		.ver = 0x12, /* [7:4] major [3:0] minor */
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1400,
		.slice_height = 20,
		.slice_width = 1400,
		.chunk_size = 1400,
		.xmit_delay = 512,
		.dec_delay = 1015,
		.scale_value = 32,
		.increment_interval = 664,
		.decrement_interval = 19,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1402,
		.slice_bpg_offset = 499,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
 	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0xFF, 0x10}},
		.dfps_cmd_table[1] = {0, 2,  {0xFB, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xB2, 0x91}},
		.dfps_cmd_table[3] = {0, 2 , {0xB3, 0x00}},
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_120_60HZ,
		.vfp = VFP_120HZ,
		.hbp = HBP_120_60HZ,
	},
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 463,
	.data_rate = 926,
	.data_rate_khz = 926924,
	.physical_width_um = 239988,
	.physical_height_um = 171420,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.dual_swap = false,
	.vdo_per_frame_lp_enable = 1,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x53,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.dsc_params = {
		.enable = 1,
		.ver = 0x12, /* [7:4] major [3:0] minor */
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1400,
		.slice_height = 20,
		.slice_width = 1400,
		.chunk_size = 1400,
		.xmit_delay = 512,
		.dec_delay = 1015,
		.scale_value = 32,
		.increment_interval = 664,
		.decrement_interval = 19,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1402,
		.slice_bpg_offset = 499,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2 , {0xFF, 0x10}},
		.dfps_cmd_table[1] = {0, 2,  {0xFB, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xB2, 0x80}},
		.dfps_cmd_table[3] = {0, 2 , {0xB3, 0x40}},
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_90_48HZ,
		.vfp = VFP_90HZ,
		.hbp = HBP_90_48HZ,
	},
};

static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 463,
	.data_rate = 926,
	.data_rate_khz = 926924,
	.physical_width_um = 239988,
	.physical_height_um = 171420,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.dual_swap = false,
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x53,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.dsc_params = {
		.enable = 1,
		.ver = 0x12, /* [7:4] major [3:0] minor */
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1400,
		.slice_height = 20,
		.slice_width = 1400,
		.chunk_size = 1400,
		.xmit_delay = 512,
		.dec_delay = 1015,
		.scale_value = 32,
		.increment_interval = 664,
		.decrement_interval = 19,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1402,
		.slice_bpg_offset = 499,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2 , {0xFF, 0x10}},
		.dfps_cmd_table[1] = {0, 2,  {0xFB, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xB2, 0x91}},
		.dfps_cmd_table[3] = {0, 2 , {0xB3, 0x00}},
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_120_60HZ,
		.vfp = VFP_60HZ,
		.hbp = HBP_120_60HZ,
	},
};

static struct mtk_panel_params ext_params_48hz = {
	.pll_clk = 463,
	.data_rate = 926,
	.data_rate_khz = 926924,
	.physical_width_um = 239988,
	.physical_height_um = 171420,
	.output_mode = MTK_PANEL_DUAL_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.lcm_cmd_if = MTK_PANEL_DUAL_PORT,
	.dual_swap = false,
	.vdo_per_frame_lp_enable = 1,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x53,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.dsc_params = {
		.enable = 1,
		.ver = 0x12, /* [7:4] major [3:0] minor */
		.slice_mode = 0,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2000,
		.pic_width = 1400,
		.slice_height = 20,
		.slice_width = 1400,
		.chunk_size = 1400,
		.xmit_delay = 512,
		.dec_delay = 1015,
		.scale_value = 32,
		.increment_interval = 664,
		.decrement_interval = 19,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1402,
		.slice_bpg_offset = 499,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = rc_buf_thresh,
			.range_min_qp = range_min_qp,
			.range_max_qp = range_max_qp,
			.range_bpg_ofs = range_bpg_ofs,
		},
	},
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 48,
		.dfps_cmd_table[0] = {0, 2 , {0xFF, 0x10}},
		.dfps_cmd_table[1] = {0, 2,  {0xFB, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xB2, 0x80}},
		.dfps_cmd_table[3] = {0, 2 , {0xB3, 0x40}},
	},
	.dyn = {
		.switch_en = 1,
		.hfp = HFP_90_48HZ,
		.vfp = VFP_48HZ,
		.hbp = HBP_90_48HZ,
	},
};

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;

	pr_info("[lcd_info]%s: ++\n", __func__);

	if (ctx->prepared){
		pr_info("[lcd_info]%s: frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	//set vddi 1.8v
	ret = lcm_enable_vddi(panel, 1);
	if(ret) {
		pr_err("[lcd_info][error]%s: set vddio on failed! ret=%d line=%d\n", __func__, ret, __LINE__);
	}
	M_DELAY(10);

	pr_info("[lcd_info]%s: --\n", __func__);
	return ret;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret = 0;

	pr_info("[lcd_info]%s: ++\n", __func__);

	if (ctx->prepared){
		pr_info("[lcd_info]%s: frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	if (is_pd_with_guesture) {
		pr_info("[lcd_info]%s: tp guesture enable, not disable vddi\n", __func__);
		return 0;
	}
	//set vddi 1.8v
	usleep_range(20 * 1000, 25 * 1000);
	ret = lcm_enable_vddi(panel, 0);
	if(ret) {
		pr_err("[lcd_info][error]%s: set vddio on failed! ret=%d line=%d\n", __func__, ret, __LINE__);
	}
	M_DELAY(10);

	pr_info("[lcd_info]%s: --\n", __func__);
	return ret;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	unsigned int bl_level = 0;
	unsigned int i2c_maped_level = 0;
	unsigned char cabc_brightness_lvl[] = {0x51, 0x07, 0xff};
	unsigned char cabc_open[] = {0x53, 0x2C};
	unsigned char cabc_close[] = {0x53, 0x00};

	if (level > MAX_HW_BRIGHTNESS)
		bl_level = MAX_HW_BRIGHTNESS;
	else
		bl_level = level;

	level_backup = bl_level;
	oplus_display_brightness = bl_level;
	pr_info("[lcd_info]%s: bl_level:%d, mapping value = %d\n", __func__, bl_level, backlight_map[bl_level]);

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && bl_level > 0) {
		bl_level = 1024;
	}


	i2c_maped_level = backlight_map[bl_level];

	if (!cb)
		return -1;

	/*I2C & PWM mode set i2c level to max firstly,Only ctrl by pwm*/
	if (ktz8868_set_bl_flag == false) {
		cb(dsi, handle, cabc_brightness_lvl, ARRAY_SIZE(cabc_brightness_lvl));
		usleep_range(1 * 1000, 2 * 1000);
		cb(dsi, handle, cabc_open, ARRAY_SIZE(cabc_open));
	}

	pr_info("[lcd_info]%s level = %d,backlight = %d,cabc_brightness_lvl[1] = 0x%x,cabc_brightness_lvl[2] = 0x%x,%d\n",
		__func__, level, bl_level, cabc_brightness_lvl[1], cabc_brightness_lvl[2],i2c_maped_level);

	if(bl_level == 0) {
		cb(dsi, handle, cabc_close, ARRAY_SIZE(cabc_close));
		ktz8868_set_brightness_level(0);
		usleep_range(1 * 1000, 2 * 1000);
	} else {
		ktz8868_set_brightness_level(i2c_maped_level);
	}

	return 0;
}

#define CABC_MODE_CMD_SIZE 5
static void cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	unsigned char cabc_mode_para = 0;
	int i = 0;
	unsigned char cabc_mode_cmd[CABC_MODE_CMD_SIZE][2] = {
		{0xFF, 0x10},
		{0xFB, 0x01},
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
		pr_info("[lcd_info]%s: cabc_mode=%d is not support, close cabc !\n", __func__, cabc_mode);
		cabc_mode_para = 0;
	}

	cabc_mode_cmd[3][1] = cabc_mode_para;
	for (i = 0; i < CABC_MODE_CMD_SIZE; i++) {
		cb(dsi, handle, cabc_mode_cmd[i], ARRAY_SIZE(cabc_mode_cmd[i]));
	}

	pr_info("[lcd_info]%s:cabc mode_%d, set cabc_para=%d\n", __func__, cabc_mode, cabc_mode_para);
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int target_fps = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	target_fps = drm_mode_vrefresh(m);


	if (target_fps == 120)
		ext->params = &ext_params;
	else if (target_fps == 90)
		ext->params = &ext_params_90hz;
	else if (target_fps == 60)
		ext->params = &ext_params_60hz;
	else if (target_fps == 48)
		ext->params = &ext_params_48hz;
	else
		ret = 1;

    g_fps_current = target_fps;
	return ret;
}

static int lcd_esd_gpio_read(struct drm_panel *panel)
{
	struct lcm *ctx = container_of(panel, struct lcm, panel);
	int master_read_value = 0, slave_read_value = 0;
	int ret = 0;

	master_read_value = gpiod_get_value(ctx->master_esd_gpio);
	slave_read_value = gpiod_get_value(ctx->slave_esd_gpio);
	printk("[lcd_info][ESD]%s: master:%d slave:%d\n", __func__, master_read_value, slave_read_value);
	if( master_read_value || slave_read_value) {

		pr_err("[ESD]%s: triger esd to recovery\n", __func__);
		ret = 1;

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

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.cabc_switch = cabc_mode_switch,
	.esd_read_gpio = lcd_esd_gpio_read,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
	struct drm_display_mode *mode4;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		pr_info("failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_90hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_60hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);

	mode4 = drm_mode_duplicate(connector->dev, &performance_mode_48hz);
	if (!mode4) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_48hz.hdisplay, performance_mode_48hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_48hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode4);

	connector->display_info.width_mm = 273;
	connector->display_info.height_mm = 171;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct device *dev = &dsi->dev;
	struct device_node *backlight;
	struct device_node *led_node;
	struct lcm *ctx;
	struct device_node *pmic_np;
	struct platform_device *pmic_pdev;

	int ret;

	pr_info("%s+++\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}
	pr_info("It's oplus25682_nt36532c_2800_2000_dual_dsi_vdo_120hz_csot\n");

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	pmic_np = of_find_compatible_node(NULL, NULL, "mediatek,mt6363-pinctrl");
	if (pmic_np) {
		pmic_pdev = of_find_device_by_node(pmic_np);
		of_node_put(pmic_np);
		if (!pmic_pdev) {
			pr_err("Failed to get PMIC pdev\n");
			return -EPROBE_DEFER;
		} else {
			ctx->pmic_regmap = dev_get_regmap(pmic_pdev->dev.parent, NULL);
			if (IS_ERR_OR_NULL(ctx->pmic_regmap))
				pr_err("Failed to get PMIC regmap\n");
			else
				regmap_set_bits(ctx->pmic_regmap, MT6363_BUCK_VS1_VOTER_CON1_SET,
					LCM_LDO_KEEP_AWAKE_BIT);
			platform_device_put(pmic_pdev);
		}
	}

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);
		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	led_node = of_find_node_by_name(NULL, "mtk-leds");
	if (led_node) {
		const char *comp_str = NULL;
		of_property_read_string(led_node, "compatible", &comp_str);
		pr_info("Found mtk_leds node: %s\n", comp_str ? comp_str : "N/A");

		if (of_device_is_compatible(led_node, "mediatek,mtk-leds")) {
			ctx->bl_mode = LED_MODE_BLS_VIRTUAL;
			pr_info("Backlight mode set to VIRTUAL\n");
		} else if (of_device_is_compatible(led_node, "mediatek,disp-leds")) {
			ctx->bl_mode = LED_MODE_BLS_CABC;
			pr_info("Backlight mode set to CABC\n");
		} else {
			ctx->bl_mode = LED_MODE_BLS_NONE;
			pr_info("Unknown backlight mode: %s\n", comp_str);
		}
		of_node_put(led_node);
	} else {
		ctx->bl_mode = LED_MODE_BLS_NONE;
		pr_info("Failed to find mtk_leds node\n");
	}

	ctx->display_dual_swap = of_property_read_bool(dev->of_node,
					      "display-dual-swap");
	pr_notice("ctx->display_dual_swap=%d\n", ctx->display_dual_swap);
	if (ctx->display_dual_swap)
		ext_params.dual_swap = true;

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		printk("[lcd_info][error]%s: cannot get reset-gpios %ld\n",
			 __func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	ctx->bias_en = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_en)) {
		printk("[lcd_info][error]%s: cannot get bias_en %ld\n",
			 __func__, PTR_ERR(ctx->bias_en));
		return PTR_ERR(ctx->bias_en);
	}
	gpiod_set_value(ctx->bias_en, 1);
	printk("[lcd_info]%s: set BL_EN to high\n", __func__);
	devm_gpiod_put(ctx->dev, ctx->bias_en);

	ctx->lcm_vddi_en = devm_gpiod_get(ctx->dev, "lcm-vddi-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->lcm_vddi_en)) {
		printk("[lcd_info][error]%s: cannot getbias-enp-en-gpios %ld\n",
			 __func__, PTR_ERR(ctx->lcm_vddi_en));
		return PTR_ERR(ctx->lcm_vddi_en);
	}
	devm_gpiod_put(ctx->dev, ctx->lcm_vddi_en);

	ctx->bias_enp_en = devm_gpiod_get(ctx->dev, "bias-enp-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enp_en)) {
		printk("[lcd_info][error]%s: cannot getbias-enp-en-gpios %ld\n",
			 __func__, PTR_ERR(ctx->bias_enp_en));
		return PTR_ERR(ctx->bias_enp_en);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enp_en);

	ctx->bias_enn_en = devm_gpiod_get(ctx->dev, "bias-enn-en", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_enn_en)) {
		printk("[lcd_info][error]%s: cannot getbias-enp-en-gpios %ld\n",
			 __func__, PTR_ERR(ctx->bias_enn_en));
		return PTR_ERR(ctx->bias_enn_en);
	}
	devm_gpiod_put(ctx->dev, ctx->bias_enn_en);

	printk("[lcd_info][ESD]%s master_esd_gpio line=%d\n", __func__, __LINE__);
	ctx->master_esd_gpio = devm_gpiod_get_optional(ctx->dev, "master-esd", GPIOD_IN);
	if (IS_ERR(ctx->master_esd_gpio)) {
		printk("[lcd_info][error]%s: cannot get master_esd_gpio %ld\n",
			__func__, PTR_ERR(ctx->master_esd_gpio));
		return PTR_ERR(ctx->master_esd_gpio);
	} else {
		gpiod_direction_input(ctx->master_esd_gpio);
	}

	printk("[lcd_info][ESD]%s: slave_esd_gpio line=%d\n", __func__, __LINE__);
	ctx->slave_esd_gpio = devm_gpiod_get_optional(ctx->dev, "slave-esd", GPIOD_IN);
	if (IS_ERR(ctx->slave_esd_gpio)) {
		printk("[lcd_info][error]%s: cannot get slave_esd_gpio %ld\n",
			__func__, PTR_ERR(ctx->slave_esd_gpio));
		return PTR_ERR(ctx->slave_esd_gpio);
	} else {
		gpiod_direction_input(ctx->slave_esd_gpio);
	}

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		dev_err(dev, "mipi_dsi_attach fail, ret=%d\n", ret);
		return -EPROBE_DEFER;
	}

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;

	/* wanhang */
	register_device_proc("lcd", "nt36532c", "csot");
	pr_info("%s-\n", __func__);

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
	if (ext_ctx != NULL) {
		mtk_panel_detach(ext_ctx);
		mtk_panel_remove(ext_ctx);
	}
#endif

	if (!IS_ERR_OR_NULL(ctx->pmic_regmap))
		regmap_set_bits(ctx->pmic_regmap, MT6363_BUCK_VS1_VOTER_CON1_CLR, LCM_LDO_KEEP_AWAKE_BIT);
}
static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "ae150_p_d_a0038_vdo_evt",
	},
	{}
};
MODULE_DEVICE_TABLE(of, lcm_of_match);
static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
			.name = "ae150_p_d_a0038_vdo_evt",
			.owner = THIS_MODULE,
			.of_match_table = lcm_of_match,
		},
};
module_mipi_dsi_driver(lcm_driver);
MODULE_AUTHOR("xian.zhang <xian.zhang@tinno.com>");
MODULE_DESCRIPTION("csot nt36532c 2800*2000 120Hz DPHY sPanel Driver");
MODULE_LICENSE("GPL");

