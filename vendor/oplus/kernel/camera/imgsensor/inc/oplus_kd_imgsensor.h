// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */
#ifndef __OPLUS_KD_IMGSENSOR_H
#define __OPLUS_KD_IMGSENSOR_H

#define S5KJN1_SENSOR_ID                        0x38E1
#define SENSOR_ID_OFFSET_LUNA                       0x1000
#define IMX800LUNA_SENSOR_ID                        (0x0800 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX800LUNA_MIPI_RAW  "imx800luna_mipi_raw"
#define IMX709LUNA_SENSOR_ID                        (0x0709 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX709LUNA_MIPI_RAW  "imx709luna_mipi_raw"
#define S5KJN1LUNA_SENSOR_ID                        (0x38E1 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_S5KJN1LUNA_MIPI_RAW  "s5kjn1luna_mipi_raw"
#define IMX766LUNA_SENSOR_ID                        (0x0766 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX766LUNA_MIPI_RAW  "imx766luna_mipi_raw"
#define IMX890TELELUNA_SENSOR_ID                    (0x0890 + SENSOR_ID_OFFSET_LUNA + 0x1)
#define SENSOR_DRVNAME_IMX890TELELUNA_MIPI_RAW  "imx890teleluna_mipi_raw"
#define IMX890LUNA_SENSOR_ID                        (0x0890 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX890LUNA_MIPI_RAW      "imx890luna_mipi_raw"

#define SENSOR_ID_OFFSET_NVWA                        0x2113
#define NVWAFRONT_SENSOR_ID                          0x281C     /*(0x0709 + SENSOR_ID_OFFSET_NVWA) 10268*/
#define SENSOR_DRVNAME_NVWAFRONT_MIPI_RAW            "nvwafront_mipi_raw"
#define NVWAFRONT2_SENSOR_ID                         0x291C     /*(0x0809 + SENSOR_ID_OFFSET_NVWA) 10524*/
#define SENSOR_DRVNAME_NVWAFRONT2_MIPI_RAW           "nvwafront2_mipi_raw"
#define NVWAEARTH_SENSOR_ID                          0x29A4     /*(0x0890 + SENSOR_ID_OFFSET_NVWA + 0x1) 10660*/
#define SENSOR_DRVNAME_NVWAEARTH_MIPI_RAW            "nvwaearth_mipi_raw"
#define NVWATELE_SENSOR_ID                           0x7777     /*(0x5664 + SENSOR_ID_OFFSET_NVWA) 30583*/
#define SENSOR_DRVNAME_NVWATELE_MIPI_RAW             "nvwatele_mipi_raw"
#define NVWAMAIN_SENSOR_ID                           0x2A79     /*(0x0966 + SENSOR_ID_OFFSET_NVWA) 10873*/
#define SENSOR_DRVNAME_NVWAMAIN_MIPI_RAW             "nvwamain_mipi_raw"
#define NVWASUN2_SENSOR_ID                           0x29A3     /*(0x0890 + SENSOR_ID_OFFSET_NVWA) 10659*/
#define SENSOR_DRVNAME_NVWASUN2_MIPI_RAW             "nvwasun2_mipi_raw"
#define NVWAMARS_SENSOR_ID                           0x2995     /*(0x0882 + SENSOR_ID_OFFSET_NVWA) 10645*/
#define SENSOR_DRVNAME_NVWAMARS_MIPI_RAW             "nvwamars_mipi_raw"
#define NVWAMARS2_SENSOR_ID                          0x2694     /*(0x0581 + SENSOR_ID_OFFSET_NVWA) 9876*/
#define SENSOR_DRVNAME_NVWAMARS2_MIPI_RAW            "nvwamars2_mipi_raw"
#define NVWAUWIDE_SENSOR_ID                          0x59F4     /*(0x38E1 + SENSOR_ID_OFFSET_NVWA) 23038*/
#define SENSOR_DRVNAME_NVWAUWIDE_MIPI_RAW            "nvwauwide_mipi_raw"

#define SENSOR_ID_OFFSET_OMEGAS2                        0x3265
#define OMEGAS2MAIN_SENSOR_ID                           0x3AE7     /*(0x0882 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2MAIN_MIPI_RAW             "omegas2main_mipi_raw"
#define OMEGAS2WIDE_SENSOR_ID                           0x35BA     /*(0x0355 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2WIDE_MIPI_RAW             "omegas2wide_mipi_raw"
#define OMEGAS2TELE_SENSOR_ID                           0x6B4B     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS2 + 0x1) */
#define SENSOR_DRVNAME_OMEGAS2TELE_MIPI_RAW             "omegas2tele_mipi_raw"
#define OMEGAS2FRONT_SENSOR_ID                          0x6B4A     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2FRONT_MIPI_RAW            "omegas2front_mipi_raw"

#define SENSOR_ID_OFFSET_CASIO                        0x0001
#define CASIOMAIN_SENSOR_ID                           0x0883     /*(0x0882 + SENSOR_ID_OFFSET_CASIO) */
#define SENSOR_DRVNAME_CASIOMAIN_MIPI_RAW             "casiomain_mipi_raw"
#define CASIOMONO_SENSOR_ID                           0xeb53     /*(0xeb52 + SENSOR_ID_OFFSET_CASIO) */
#define SENSOR_DRVNAME_CASIOMONO_MIPI_RAW             "casiomono_mipi_raw"
#define CASIOFRONT_SENSOR_ID                          0x310A     /*(0x3109 + SENSOR_ID_OFFSET_CASIO) */
#define SENSOR_DRVNAME_CASIOFRONT_MIPI_RAW            "casiofront_mipi_raw"
#define CASIOWIDE_SENSOR_ID                           0x0356     /*(0x0355 + SENSOR_ID_OFFSET_CASIO) */
#define SENSOR_DRVNAME_CASIOWIDE_MIPI_RAW             "casiowide_mipi_raw"
#define CASIOMAIN2_SENSOR_ID                          0x565045     /*(0x565044 + SENSOR_ID_OFFSET_CASIO) */
#define SENSOR_DRVNAME_CASIOMAIN2_MIPI_RAW            "casiomain2_mipi_raw"

#define SENSOR_ID_OFFSET_ALPHAH                       0x4051
#define ALPHAHMAIN_SENSOR_ID                          0x9095     /*(0x5044 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHMAIN_MIPI_RAW            "alphahmain_mipi_raw"
#define ALPHAHFRONT_SENSOR_ID                         0x44C2     /*(0x0471 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHFRONT_MIPI_RAW           "alphahfront_mipi_raw"
#define ALPHAHMONO_SENSOR_ID                          0x4331     /*(0x02e0 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHMONO_MIPI_RAW            "alphahmono_mipi_raw"

#define SENSOR_ID_OFFSET_GREENLANDFRONT                  0x3
#define SENSOR_ID_OFFSET_GREENLANDMAIN                   0x4
#define GREENLANDFRONT_SENSOR_ID                         0xD157      /*(0xD154 + SENSOR_ID_OFFSET_GREENLANDFRONT)*/
#define SENSOR_DRVNAME_GREENLANDFRONT_MIPI_RAW          "greenlandfront_mipi_raw"
#define GREENLANDMAIN_SENSOR_ID                          0xD158      /*(0xD154 + SENSOR_ID_OFFSET_GREENLANDMAIN)*/
#define SENSOR_DRVNAME_GREENLANDMAIN_MIPI_RAW           "greenlandmain_mipi_raw"

#define SENSOR_ID_OFFSET_BAIKALM                         0x0111
#define BAIKALMMAIN_SENSOR_ID                            0x5155  /*(0x5044 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMMAIN_MIPI_RAW             "baikalmmain_mipi_raw"
#define BAIKALMFRONT_SENSOR_ID                           0x09B9  /*(0x08A8 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT_MIPI_RAW            "baikalmfront_mipi_raw"
#define BAIKALMFRONT2_SENSOR_ID                          0x321B  /*(0x3109 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT2_MIPI_RAW           "baikalmfront2_mipi_raw"
#define BAIKALMFRONT3_SENSOR_ID                          0x0582     /*(0x0471 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT3_MIPI_RAW           "baikalmfront3_mipi_raw"
#define BAIKALMMONO_SENSOR_ID                            0x013C  /*(0x002B + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMMONO_MIPI_RAW             "baikalmmono_mipi_raw"

#define SENSOR_ID_OFFSET_CASIOX                          0x0002
#define CASIOXMAIN_SENSOR_ID                             0x0884     /*(0x0882 + SENSOR_ID_OFFSET_CASIOX) */
#define SENSOR_DRVNAME_CASIOXMAIN_MIPI_RAW               "casioxmain_mipi_raw"
#define CASIOXFRONT_SENSOR_ID                            0x32E4     /*(0x32E2 + SENSOR_ID_OFFSET_CASIOX) */
#define SENSOR_DRVNAME_CASIOXFRONT_MIPI_RAW              "casioxfront_mipi_raw"
#define CASIOXFRONT2_SENSOR_ID                            0x0472     /*(0x0471 + SENSOR_ID_OFFSET_CASIOX) */
#define SENSOR_DRVNAME_CASIOXFRONT2_MIPI_RAW              "casioxfront2_mipi_raw"
#define CASIOXWIDE_SENSOR_ID                             0x0357     /*(0x0355 + SENSOR_ID_OFFSET_CASIOX) */
#define SENSOR_DRVNAME_CASIOXWIDE_MIPI_RAW               "casioxwide_mipi_raw"
#define CASIOXMONO_SENSOR_ID                             0xeb54     /*(0xeb52 + SENSOR_ID_OFFSET_CASIOX) */
#define SENSOR_DRVNAME_CASIOXMONO_MIPI_RAW               "casioxmono_mipi_raw"

#define SENSOR_ID_OFFSET_CASIOY                          0x0003
#define CASIOYMAIN_SENSOR_ID                             0x0885     /*(0x0882 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOYMAIN_MIPI_RAW               "casioymain_mipi_raw"
#define CASIOYFRONT_SENSOR_ID                            0x32E5     /*(0x32E2 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOYFRONT_MIPI_RAW              "casioyfront_mipi_raw"
#define CASIOYFRONT2_SENSOR_ID                           0x5047     /*(0x5044 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOYFRONT2_MIPI_RAW             "casioyfront2_mipi_raw"
#define CASIOPFRONT_SENSOR_ID                           0x0474     /*(0x0471 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOPFRONT_MIPI_RAW             "casiopfront_mipi_raw"
#define CASIOYUWIDE_SENSOR_ID                            0x560B     /*(0x5608 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOYUWIDE_MIPI_RAW              "casioyuwide_mipi_raw"
#define CASIOYMONO_SENSOR_ID                             0xeb55     /*(0xeb52 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOYMONO_MIPI_RAW               "casioymono_mipi_raw"

#define SENSOR_ID_OFFSET_CASIOM                          0x0006
#define CASIOMMAIN_SENSOR_ID                             0x0888     /*(0x0882 + SENSOR_ID_OFFSET_CASIOM) */
#define SENSOR_DRVNAME_CASIOMMAIN_MIPI_RAW               "casiommain_mipi_raw"
#define CASIOMFRONT_SENSOR_ID                            0x32E8     /*(0x32E2 + SENSOR_ID_OFFSET_CASIOM) */
#define SENSOR_DRVNAME_CASIOMFRONT_MIPI_RAW              "casiomfront_mipi_raw"
#define CASIOMUWIDE_SENSOR_ID                            0x560E     /*(0x5608 + SENSOR_ID_OFFSET_CASIOM) */
#define SENSOR_DRVNAME_CASIOMUWIDE_MIPI_RAW              "casiomuwide_mipi_raw"

#define SENSOR_ID_OFFSET_CASIOP                          0x0004
#define CASIOPMAIN_SENSOR_ID                             0x5048     /*(0x5044 + SENSOR_ID_OFFSET_CASIOY) */
#define SENSOR_DRVNAME_CASIOPMAIN_MIPI_RAW               "casiopmain_mipi_raw"

#define SENSOR_ID_OFFSET_SUZUKI                          0x0005
#define SUZUKIFRONT_SENSOR_ID                            0x84B      /*(0x846 + SENSOR_ID_OFFSET_SUZUKIFRONT)*/
#define SENSOR_DRVNAME_SUZUKIFRONT_MIPI_RAW           "suzukifront_mipi_raw"

#define SENSOR_ID_OFFSET_OMEGAS3                        0x3261
#define OMEGAS3MAIN_SENSOR_ID                           0x3AE3     /*(0x0882 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3MAIN_MIPI_RAW             "omegas3main_mipi_raw"
#define OMEGAS3WIDE_SENSOR_ID                           0x35B6     /*(0x0355 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3WIDE_MIPI_RAW             "omegas3wide_mipi_raw"
#define OMEGAS3MACRO_SENSOR_ID                           0x328D     /*(0x002B + SENSOR_ID_OFFSET_OMEGAS3 + 0x1) */
#define SENSOR_DRVNAME_OMEGAS3MACRO_MIPI_RAW            "omegas3macro_mipi_raw"
#define OMEGAS3FRONT_SENSOR_ID                          0x6B46     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3FRONT_MIPI_RAW            "omegas3front_mipi_raw"
#define OMEGAS3FRONT2_SENSOR_ID                          0x6543     /*(0x33E2 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3FRONT2_MIPI_RAW            "omegas3front2_mipi_raw"

/*dunhuang*/
#define SENSOR_ID_OFFSET_DUNHUANGFRONT                      0x1
#define SENSOR_ID_OFFSET_DUNHUANGMAIN                       0x2
#define DUNHUANGFRONT_SENSOR_ID                             0x847      /*(0x846 + SENSOR_ID_OFFSET_DUNHUANGFRONT)*/
#define SENSOR_DRVNAME_DUNHUANGFRONT_MIPI_RAW              "dunhuangfront_mipi_raw"
#define DUNHUANGMAIN_SENSOR_ID                              0x848      /*(0x846 + SENSOR_ID_OFFSET_DUNHUANGMAIN)*/
#define SENSOR_DRVNAME_DUNHUANGMAIN_MIPI_RAW               "dunhuangmain_mipi_raw"

#define SENSOR_ID_OFFSET_MILKYWAYC1                        0x4021
#define MILKYWAYC1MAIN_SENSOR_ID                           0x48B1     /*(0x0890 + SENSOR_ID_OFFSET_MILKYWAYC1) */
#define SENSOR_DRVNAME_MILKYWAYC1MAIN_MIPI_RAW             "milkywayc1main_mipi_raw"
#define MILKYWAYC1WIDE_SENSOR_ID                           0x9629     /*(0x5608 + SENSOR_ID_OFFSET_MILKYWAYC1) */
#define SENSOR_DRVNAME_MILKYWAYC1WIDE_MIPI_RAW             "milkywayc1wide_mipi_raw"
#define MILKYWAYC1TELE_SENSOR_ID                           0x7907     /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYC1 + 0x1) */
#define SENSOR_DRVNAME_MILKYWAYC1TELE_MIPI_RAW             "milkywayc1tele_mipi_raw"
#define MILKYWAYC1FRONT_SENSOR_ID                          0x7906     /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYC1) */
#define SENSOR_DRVNAME_MILKYWAYC1FRONT_MIPI_RAW            "milkywayc1front_mipi_raw"

#define SENSOR_ID_OFFSET_MILKYWAYC2                        0x4023
#define MILKYWAYC2MAIN_SENSOR_ID                           0x48A5     /*(0x0882 + SENSOR_ID_OFFSET_MILKYWAYC2) */
#define SENSOR_DRVNAME_MILKYWAYC2MAIN_MIPI_RAW             "milkywayc2main_mipi_raw"
#define MILKYWAYC2WIDE_SENSOR_ID                           0x962B     /*(0x5608 + SENSOR_ID_OFFSET_MILKYWAYC2) */
#define SENSOR_DRVNAME_MILKYWAYC2WIDE_MIPI_RAW             "milkywayc2wide_mipi_raw"
#define MILKYWAYC2FRONT_SENSOR_ID                          0x7908     /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYC2) */
#define SENSOR_DRVNAME_MILKYWAYC2FRONT_MIPI_RAW            "milkywayc2front_mipi_raw"

#define SENSOR_ID_OFFSET_MILKYWAYS1                        0x4261
#define MILKYWAYS1MAIN_SENSOR_ID                           0x4AF1     /*(0x0890 + SENSOR_ID_OFFSET_MILKYWAYS1) */
#define SENSOR_DRVNAME_MILKYWAYS1MAIN_MIPI_RAW             "milkyways1main_mipi_raw"
#define MILKYWAYS1WIDE_SENSOR_ID                           0x9869     /*(0x5608 + SENSOR_ID_OFFSET_MILKYWAYS1) */
#define SENSOR_DRVNAME_MILKYWAYS1WIDE_MIPI_RAW             "milkyways1wide_mipi_raw"
#define MILKYWAYS1TELE_SENSOR_ID                           0x7B46    /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYS1 + 0x1) */
#define SENSOR_DRVNAME_MILKYWAYS1TELE_MIPI_RAW             "milkyways1tele_mipi_raw"
#define MILKYWAYS1FRONT_SENSOR_ID                          0x7B47     /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYS1) */
#define SENSOR_DRVNAME_MILKYWAYS1FRONT_MIPI_RAW            "milkyways1front_mipi_raw"

#define SENSOR_ID_OFFSET_MILKYWAYS2                        0x4222
#define MILKYWAYS2MAIN_SENSOR_ID                           0x4AA4     /*(0x0882 + SENSOR_ID_OFFSET_MILKYWAYS2) */
#define SENSOR_DRVNAME_MILKYWAYS2MAIN_MIPI_RAW             "milkyways2main_mipi_raw"
#define MILKYWAYS2WIDE_SENSOR_ID                           0x982A     /*(0x5608 + SENSOR_ID_OFFSET_MILKYWAYS2) */
#define SENSOR_DRVNAME_MILKYWAYS2WIDE_MIPI_RAW             "milkyways2wide_mipi_raw"
#define MILKYWAYS2MONO_SENSOR_ID                           0x424D     /*(0x002B + SENSOR_ID_OFFSET_MILKYWAYS2) */
#define SENSOR_DRVNAME_MILKYWAYS2MONO_MIPI_RAW             "milkyways2mono_mipi_raw"
#define MILKYWAYS2FRONT_SENSOR_ID                          0x7B07     /*(0x38E5 + SENSOR_ID_OFFSET_MILKYWAYS2) */
#define SENSOR_DRVNAME_MILKYWAYS2FRONT_MIPI_RAW            "milkyways2front_mipi_raw"

#define SENSOR_ID_OFFSET_CAYMANA                           0x0002
#define CAYMANAMAIN_SENSOR_ID                              0x0884     /*(0x0882 + SENSOR_ID_OFFSET_CAYMANA) */
#define SENSOR_DRVNAME_CAYMANAMAIN_MIPI_RAW                "caymanamain_mipi_raw"
#define CAYMANAUWIDE_SENSOR_ID                              0x560A     /*(0x5608 + SENSOR_ID_OFFSET_CAYMANA) */
#define SENSOR_DRVNAME_CAYMANAUWIDE_MIPI_RAW                "caymanauwide_mipi_raw"
#define CAYMANAFRONT_SENSOR_ID                             0x0473    /*(0x0471 + SENSOR_ID_OFFSET_CAYMANA) */
#define SENSOR_DRVNAME_CAYMANAFRONT_MIPI_RAW               "caymanafront_mipi_raw"

#define SENSOR_ID_OFFSET_CAYMANB                           0x2001
#define CAYMANBMAIN_SENSOR_ID                              0x2897     /*(0x0896 + SENSOR_ID_OFFSET_CAYMANB) */
#define SENSOR_DRVNAME_CAYMANBMAIN_MIPI_RAW                "caymanbmain_mipi_raw"
#define CAYMANBTELE_SENSOR_ID                              0x58E6     /*(0x38E5 + SENSOR_ID_OFFSET_CAYMANB) */
#define SENSOR_DRVNAME_CAYMANBTELE_MIPI_RAW                "caymanbtele_mipi_raw"
#define CAYMANBUWIDE_SENSOR_ID                             0x7609    /*(0x5608 + SENSOR_ID_OFFSET_CAYMANB) */
#define SENSOR_DRVNAME_CAYMANBUWIDE_MIPI_RAW               "caymanbuwide_mipi_raw"
#define CAYMANBFRONT_SENSOR_ID                             0x2472    /*(0x0471 + SENSOR_ID_OFFSET_CAYMANB) */
#define SENSOR_DRVNAME_CAYMANBFRONT_MIPI_RAW               "caymanbfront_mipi_raw"

#define SENSOR_ID_OFFSET_CAYMANBN                       0x2002
#define CAYMANBNMAIN_SENSOR_ID                          0x2908     /*(0x0906 + SENSOR_ID_OFFSET_CAYMANBN) */
#define SENSOR_DRVNAME_CAYMANBNMAIN_MIPI_RAW            "caymanbnmain_mipi_raw"
#define CAYMANBNTELE_SENSOR_ID                          0x58E7     /*(0x38E5 + SENSOR_ID_OFFSET_CAYMANBN) */
#define SENSOR_DRVNAME_CAYMANBNTELE_MIPI_RAW            "caymanbntele_mipi_raw"
#define CAYMANBNUWIDE_SENSOR_ID                         0x760A    /*(0x5608 + SENSOR_ID_OFFSET_CAYMANBN) */
#define SENSOR_DRVNAME_CAYMANBNUWIDE_MIPI_RAW           "caymanbnuwide_mipi_raw"
#define CAYMANBNFRONT_SENSOR_ID                         0x2473    /*(0x0471 + SENSOR_ID_OFFSET_CAYMANBN) */
#define SENSOR_DRVNAME_CAYMANBNFRONT_MIPI_RAW           "caymanbnfront_mipi_raw"
#define CAYMANBNFRONT2_SENSOR_ID                        0x2617    /*(0x0615 + SENSOR_ID_OFFSET_CAYMANBN) */
#define SENSOR_DRVNAME_CAYMANBNFRONT2_MIPI_RAW          "caymanbnfront2_mipi_raw"

#define SENSOR_ID_OFFSET_YAMAHA                            0x3001
#define YAMAHAMAIN_SENSOR_ID                               0x3897     /*(0x0896 + SENSOR_ID_OFFSET_YAMAHA) */
#define SENSOR_DRVNAME_YAMAHAMAIN_MIPI_RAW                 "yamahamain_mipi_raw"
#define YAMAHAUWIDE_SENSOR_ID                              0x8609     /*(0x5608 + SENSOR_ID_OFFSET_YAMAHA) */
#define SENSOR_DRVNAME_YAMAHAUWIDE_MIPI_RAW                "yamahauwide_mipi_raw"
#define YAMAHAFRONT_SENSOR_ID                              0x3472    /*(0x0471 + SENSOR_ID_OFFSET_YAMAHA) */
#define SENSOR_DRVNAME_YAMAHAFRONT_MIPI_RAW                "yamahafront_mipi_raw"

#define SENSOR_ID_OFFSET_ZHUQUEC2                          0x4075
#define ZHUQUEC2MAIN_SENSOR_ID                             0x48F7     /*(0x0882 + SENSOR_ID_OFFSET_ZHUQUEC2) */
#define SENSOR_DRVNAME_ZHUQUEC2MAIN_MIPI_RAW               "zhuquec2main_mipi_raw"
#define ZHUQUEC2WIDE_SENSOR_ID                             0x967D     /*(0x5608 + SENSOR_ID_OFFSET_ZHUQUEC2) */
#define SENSOR_DRVNAME_ZHUQUEC2WIDE_MIPI_RAW               "zhuquec2wide_mipi_raw"
#define ZHUQUEC2TELE_SENSOR_ID                             0x795B     /*(0x38E5 + SENSOR_ID_OFFSET_ZHUQUEC2 + 0x1) */
#define SENSOR_DRVNAME_ZHUQUEC2TELE_MIPI_RAW               "zhuquec2tele_mipi_raw"
#define ZHUQUEC2FRONT_SENSOR_ID                            0x795A     /*(0x38E5 + SENSOR_ID_OFFSET_ZHUQUEC2) */
#define SENSOR_DRVNAME_ZHUQUEC2FRONT_MIPI_RAW              "zhuquec2front_mipi_raw"

#define SENSOR_ID_OFFSET_ZHUQUES2                          0x4322
#define ZHUQUES2MAIN_SENSOR_ID                             0x4BA4    /*(0x0882 + SENSOR_ID_OFFSET_ZHUQUES2) */
#define SENSOR_DRVNAME_ZHUQUES2MAIN_MIPI_RAW               "zhuques2main_mipi_raw"
#define ZHUQUES2WIDE_SENSOR_ID                             0x992A    /*(0x5608 + SENSOR_ID_OFFSET_ZHUQUES2) */
#define SENSOR_DRVNAME_ZHUQUES2WIDE_MIPI_RAW               "zhuques2wide_mipi_raw"
#define ZHUQUES2TELE_SENSOR_ID                             0x7C08     /*(0x38E5 + SENSOR_ID_OFFSET_ZHUQUES2 + 0x1) */
#define SENSOR_DRVNAME_ZHUQUES2TELE_MIPI_RAW               "zhuques2tele_mipi_raw"
#define ZHUQUES2FRONT_SENSOR_ID                            0x7C07     /*(0x38E5 + SENSOR_ID_OFFSET_ZHUQUES2) */
#define SENSOR_DRVNAME_ZHUQUES2FRONT_MIPI_RAW              "zhuques2front_mipi_raw"

#define SENSOR_ID_OFFSET_HONDA                             0x4891
#define HONDAMAIN_SENSOR_ID                                0x5113     /*(0x0882 + SENSOR_ID_OFFSET_HONDA) */
#define SENSOR_DRVNAME_HONDAMAIN_MIPI_RAW                  "hondamain_mipi_raw"
#define HONDAWIDE_SENSOR_ID                                0x9E99     /*(0x5608 + SENSOR_ID_OFFSET_HONDA) */
#define SENSOR_DRVNAME_HONDAWIDE_MIPI_RAW                  "hondawide_mipi_raw"
#define HONDAFRONT_SENSOR_ID                               0x4D02     /*(0x0471 + SENSOR_ID_OFFSET_HONDA) */
#define SENSOR_DRVNAME_HONDAFRONT_MIPI_RAW                 "hondafront_mipi_raw"

#define SENSOR_ID_OFFSET_SUBARU                         0x4890
#define SUBARUMAIN_SENSOR_ID                            0x5112     /*(0x0882 + SENSOR_ID_OFFSET_SUBARU) */
#define SENSOR_DRVNAME_SUBARUMAIN_MIPI_RAW             "subarumain_mipi_raw"
#define SUBARUMONO_SENSOR_ID                             0x48BB     /*(0x002B + SENSOR_ID_OFFSET_SUBARU) */
#define SENSOR_DRVNAME_SUBARUMONO_MIPI_RAW              "subarumono_mipi_raw"
#define SUBARUFRONT_SENSOR_ID                           0x4D01     /*(0x0471 + SENSOR_ID_OFFSET_SUBARU) */
#define SENSOR_DRVNAME_SUBARUFRONT_MIPI_RAW            "subarufront_mipi_raw"

#define SENSOR_ID_OFFSET_CAYMANP                       0x2003
#define CAYMANPMAIN_SENSOR_ID                          0x2909     /*(0x0906 + SENSOR_ID_OFFSET_CAYMANP) */
#define SENSOR_DRVNAME_CAYMANPMAIN_MIPI_RAW            "caymanpmain_mipi_raw"
#define CAYMANPUWIDE_SENSOR_ID                         0x760B    /*(0x5608 + SENSOR_ID_OFFSET_CAYMANP) */
#define SENSOR_DRVNAME_CAYMANPUWIDE_MIPI_RAW           "caymanpuwide_mipi_raw"
#define CAYMANPFRONT_SENSOR_ID                         0x2474    /*(0x0471 + SENSOR_ID_OFFSET_CAYMANP) */
#define SENSOR_DRVNAME_CAYMANPFRONT_MIPI_RAW           "caymanpfront_mipi_raw"
#define CAYMANPFRONT2_SENSOR_ID                        0x2618    /*(0x0615 + SENSOR_ID_OFFSET_CAYMANP) */
#define SENSOR_DRVNAME_CAYMANPFRONT2_MIPI_RAW          "caymanpfront2_mipi_raw"

#define SENSOR_ID_OFFSET_CAYMANC                        0x0011
#define CAYMANCMAIN_SENSOR_ID                           0x0893     /*(0x0882 + SENSOR_ID_OFFSET_CAYMAN) */
#define SENSOR_DRVNAME_CAYMANCMAIN_MIPI_RAW             "caymancmain_mipi_raw"
#define CAYMANCUWIDE_SENSOR_ID                           0x561A     /*(0x5609 + SENSOR_ID_OFFSET_CAYMAN) */
#define SENSOR_DRVNAME_CAYMANCUWIDE_MIPI_RAW             "caymancuwide_mipi_raw"
#define CAYMANCFRONT_SENSOR_ID                          0x0482    /*(0x0471 + SENSOR_ID_OFFSET_CAYMAN) */
#define SENSOR_DRVNAME_CAYMANCFRONT_MIPI_RAW            "caymancfront_mipi_raw"

#define SENSOR_ID_OFFSET_CHROMEFRONT                  0x1
#define SENSOR_ID_OFFSET_CHROMEMAIN                   0x2
#define CHROMEFRONT_SENSOR_ID                         0xD155      /*(0xd154 + SENSOR_ID_OFFSET_CHROMEFRONT)*/
#define SENSOR_DRVNAME_CHROMEFRONT_MIPI_RAW           "chromefront_mipi_raw"
#define CHROMEMAIN_SENSOR_ID                          0xD156      /*(0xd154 + SENSOR_ID_OFFSET_CHROMEMAIN)*/
#define SENSOR_DRVNAME_CHROMEMAIN_MIPI_RAW            "chromemain_mipi_raw"

#define SENSOR_ID_OFFSET_IWC                             0x0005
#define IWCMAIN_SENSOR_ID                                0x1B7A     /*(0x1B75 + SENSOR_ID_OFFSET_IWC) */
#define SENSOR_DRVNAME_IWCMAIN_MIPI_RAW                  "iwcmain_mipi_raw"
#define IWCFRONT_SENSOR_ID                              0x5049     /*(0x5044 + SENSOR_ID_OFFSET_IWC) */
#define SENSOR_DRVNAME_IWCFRONT_MIPI_RAW                "iwcfront_mipi_raw"
#define IWCUWIDE_SENSOR_ID                               0x560D     /*(0x5608 + SENSOR_ID_OFFSET_IWC) */
#define SENSOR_DRVNAME_IWCUWIDE_MIPI_RAW                 "iwcuwide_mipi_raw"

#endif    /* __OPLUS_KD_IMGSENSOR_H */
