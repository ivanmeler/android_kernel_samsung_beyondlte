struct IntrinToName {
  uint32_t Id;
  int32_t FullName;
  int32_t ShortName;
};
static const IntrinToName Map[] = {
  { ARM::BI__builtin_arm_mve_asrl, 0, -1},
  { ARM::BI__builtin_arm_mve_lsll, 5, -1},
  { ARM::BI__builtin_arm_mve_sqrshr, 10, -1},
  { ARM::BI__builtin_arm_mve_sqrshrl, 17, -1},
  { ARM::BI__builtin_arm_mve_sqrshrl_sat48, 25, -1},
  { ARM::BI__builtin_arm_mve_sqshl, 39, -1},
  { ARM::BI__builtin_arm_mve_sqshll, 45, -1},
  { ARM::BI__builtin_arm_mve_srshr, 52, -1},
  { ARM::BI__builtin_arm_mve_srshrl, 58, -1},
  { ARM::BI__builtin_arm_mve_uqrshl, 65, -1},
  { ARM::BI__builtin_arm_mve_uqrshll, 72, -1},
  { ARM::BI__builtin_arm_mve_uqrshll_sat48, 80, -1},
  { ARM::BI__builtin_arm_mve_uqshl, 94, -1},
  { ARM::BI__builtin_arm_mve_uqshll, 100, -1},
  { ARM::BI__builtin_arm_mve_urshr, 107, -1},
  { ARM::BI__builtin_arm_mve_urshrl, 113, -1},
  { ARM::BI__builtin_arm_mve_vabavq_p_s16, 129, 120},
  { ARM::BI__builtin_arm_mve_vabavq_p_s32, 142, 120},
  { ARM::BI__builtin_arm_mve_vabavq_p_s8, 155, 120},
  { ARM::BI__builtin_arm_mve_vabavq_p_u16, 167, 120},
  { ARM::BI__builtin_arm_mve_vabavq_p_u32, 180, 120},
  { ARM::BI__builtin_arm_mve_vabavq_p_u8, 193, 120},
  { ARM::BI__builtin_arm_mve_vabavq_s16, 212, 205},
  { ARM::BI__builtin_arm_mve_vabavq_s32, 223, 205},
  { ARM::BI__builtin_arm_mve_vabavq_s8, 234, 205},
  { ARM::BI__builtin_arm_mve_vabavq_u16, 244, 205},
  { ARM::BI__builtin_arm_mve_vabavq_u32, 255, 205},
  { ARM::BI__builtin_arm_mve_vabavq_u8, 266, 205},
  { ARM::BI__builtin_arm_mve_vabdq_f16, 282, 276},
  { ARM::BI__builtin_arm_mve_vabdq_f32, 292, 276},
  { ARM::BI__builtin_arm_mve_vabdq_m_f16, 310, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_f32, 322, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_s16, 334, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_s32, 346, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_s8, 358, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_u16, 369, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_u32, 381, 302},
  { ARM::BI__builtin_arm_mve_vabdq_m_u8, 393, 302},
  { ARM::BI__builtin_arm_mve_vabdq_s16, 404, 276},
  { ARM::BI__builtin_arm_mve_vabdq_s32, 414, 276},
  { ARM::BI__builtin_arm_mve_vabdq_s8, 424, 276},
  { ARM::BI__builtin_arm_mve_vabdq_u16, 433, 276},
  { ARM::BI__builtin_arm_mve_vabdq_u32, 443, 276},
  { ARM::BI__builtin_arm_mve_vabdq_u8, 453, 276},
  { ARM::BI__builtin_arm_mve_vabdq_x_f16, 470, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_f32, 482, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_s16, 494, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_s32, 506, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_s8, 518, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_u16, 529, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_u32, 541, 462},
  { ARM::BI__builtin_arm_mve_vabdq_x_u8, 553, 462},
  { ARM::BI__builtin_arm_mve_vadciq_m_s32, 573, 564},
  { ARM::BI__builtin_arm_mve_vadciq_m_u32, 586, 564},
  { ARM::BI__builtin_arm_mve_vadciq_s32, 606, 599},
  { ARM::BI__builtin_arm_mve_vadciq_u32, 617, 599},
  { ARM::BI__builtin_arm_mve_vadcq_m_s32, 636, 628},
  { ARM::BI__builtin_arm_mve_vadcq_m_u32, 648, 628},
  { ARM::BI__builtin_arm_mve_vadcq_s32, 666, 660},
  { ARM::BI__builtin_arm_mve_vadcq_u32, 676, 660},
  { ARM::BI__builtin_arm_mve_vaddq_f16, 692, 686},
  { ARM::BI__builtin_arm_mve_vaddq_f32, 702, 686},
  { ARM::BI__builtin_arm_mve_vaddq_m_f16, 720, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_f32, 732, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_s16, 744, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_s32, 756, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_s8, 768, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_u16, 779, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_u32, 791, 712},
  { ARM::BI__builtin_arm_mve_vaddq_m_u8, 803, 712},
  { ARM::BI__builtin_arm_mve_vaddq_s16, 814, 686},
  { ARM::BI__builtin_arm_mve_vaddq_s32, 824, 686},
  { ARM::BI__builtin_arm_mve_vaddq_s8, 834, 686},
  { ARM::BI__builtin_arm_mve_vaddq_u16, 843, 686},
  { ARM::BI__builtin_arm_mve_vaddq_u32, 853, 686},
  { ARM::BI__builtin_arm_mve_vaddq_u8, 863, 686},
  { ARM::BI__builtin_arm_mve_vaddq_x_f16, 880, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_f32, 892, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_s16, 904, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_s32, 916, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_s8, 928, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_u16, 939, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_u32, 951, 872},
  { ARM::BI__builtin_arm_mve_vaddq_x_u8, 963, 872},
  { ARM::BI__builtin_arm_mve_vandq_f16, 980, 974},
  { ARM::BI__builtin_arm_mve_vandq_f32, 990, 974},
  { ARM::BI__builtin_arm_mve_vandq_m_f16, 1008, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_f32, 1020, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_s16, 1032, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_s32, 1044, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_s8, 1056, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_u16, 1067, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_u32, 1079, 1000},
  { ARM::BI__builtin_arm_mve_vandq_m_u8, 1091, 1000},
  { ARM::BI__builtin_arm_mve_vandq_s16, 1102, 974},
  { ARM::BI__builtin_arm_mve_vandq_s32, 1112, 974},
  { ARM::BI__builtin_arm_mve_vandq_s8, 1122, 974},
  { ARM::BI__builtin_arm_mve_vandq_u16, 1131, 974},
  { ARM::BI__builtin_arm_mve_vandq_u32, 1141, 974},
  { ARM::BI__builtin_arm_mve_vandq_u8, 1151, 974},
  { ARM::BI__builtin_arm_mve_vandq_x_f16, 1168, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_f32, 1180, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_s16, 1192, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_s32, 1204, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_s8, 1216, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_u16, 1227, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_u32, 1239, 1160},
  { ARM::BI__builtin_arm_mve_vandq_x_u8, 1251, 1160},
  { ARM::BI__builtin_arm_mve_vbicq_f16, 1268, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_f32, 1278, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_m_f16, 1296, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_f32, 1308, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_n_s16, 1330, 1320},
  { ARM::BI__builtin_arm_mve_vbicq_m_n_s32, 1344, 1320},
  { ARM::BI__builtin_arm_mve_vbicq_m_n_u16, 1358, 1320},
  { ARM::BI__builtin_arm_mve_vbicq_m_n_u32, 1372, 1320},
  { ARM::BI__builtin_arm_mve_vbicq_m_s16, 1386, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_s32, 1398, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_s8, 1410, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_u16, 1421, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_u32, 1433, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_m_u8, 1445, 1288},
  { ARM::BI__builtin_arm_mve_vbicq_n_s16, 1456, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_n_s32, 1468, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_n_u16, 1480, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_n_u32, 1492, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_s16, 1504, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_s32, 1514, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_s8, 1524, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_u16, 1533, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_u32, 1543, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_u8, 1553, 1262},
  { ARM::BI__builtin_arm_mve_vbicq_x_f16, 1570, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_f32, 1582, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_s16, 1594, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_s32, 1606, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_s8, 1618, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_u16, 1629, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_u32, 1641, 1562},
  { ARM::BI__builtin_arm_mve_vbicq_x_u8, 1653, 1562},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_f16, 1678, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_f32, 1696, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_f16, 1730, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_f32, 1750, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_s16, 1770, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_s32, 1790, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_s8, 1810, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_u16, 1829, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_u32, 1849, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_m_u8, 1869, 1714},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_s16, 1888, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_s32, 1906, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_s8, 1924, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_u16, 1941, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_u32, 1959, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_u8, 1977, 1664},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_f16, 2010, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_f32, 2030, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_s16, 2050, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_s32, 2070, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_s8, 2090, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_u16, 2109, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_u32, 2129, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot270_x_u8, 2149, 1994},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_f16, 2181, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_f32, 2198, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_f16, 2230, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_f32, 2249, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_s16, 2268, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_s32, 2287, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_s8, 2306, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_u16, 2324, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_u32, 2343, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_m_u8, 2362, 2215},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_s16, 2380, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_s32, 2397, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_s8, 2414, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_u16, 2430, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_u32, 2447, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_u8, 2464, 2168},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_f16, 2495, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_f32, 2514, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_s16, 2533, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_s32, 2552, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_s8, 2571, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_u16, 2589, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_u32, 2608, 2480},
  { ARM::BI__builtin_arm_mve_vcaddq_rot90_x_u8, 2627, 2480},
  { ARM::BI__builtin_arm_mve_vcmlaq_f16, 2652, 2645},
  { ARM::BI__builtin_arm_mve_vcmlaq_f32, 2663, 2645},
  { ARM::BI__builtin_arm_mve_vcmlaq_m_f16, 2683, 2674},
  { ARM::BI__builtin_arm_mve_vcmlaq_m_f32, 2696, 2674},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot180_f16, 2723, 2709},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot180_f32, 2741, 2709},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot180_m_f16, 2775, 2759},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot180_m_f32, 2795, 2759},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot270_f16, 2829, 2815},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot270_f32, 2847, 2815},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot270_m_f16, 2881, 2865},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot270_m_f32, 2901, 2865},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot90_f16, 2934, 2921},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot90_f32, 2951, 2921},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot90_m_f16, 2983, 2968},
  { ARM::BI__builtin_arm_mve_vcmlaq_rot90_m_f32, 3002, 2968},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_n_u16, 3031, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_n_u32, 3047, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_n_u8, 3063, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_u16, 3078, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_u32, 3092, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_m_u8, 3106, 3021},
  { ARM::BI__builtin_arm_mve_vcmpcsq_n_u16, 3127, 3119},
  { ARM::BI__builtin_arm_mve_vcmpcsq_n_u32, 3141, 3119},
  { ARM::BI__builtin_arm_mve_vcmpcsq_n_u8, 3155, 3119},
  { ARM::BI__builtin_arm_mve_vcmpcsq_u16, 3168, 3119},
  { ARM::BI__builtin_arm_mve_vcmpcsq_u32, 3180, 3119},
  { ARM::BI__builtin_arm_mve_vcmpcsq_u8, 3192, 3119},
  { ARM::BI__builtin_arm_mve_vcmpeqq_f16, 3211, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_f32, 3223, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_f16, 3245, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_f32, 3259, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_f16, 3273, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_f32, 3289, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_s16, 3305, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_s32, 3321, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_s8, 3337, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_u16, 3352, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_u32, 3368, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_n_u8, 3384, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_s16, 3399, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_s32, 3413, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_s8, 3427, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_u16, 3440, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_u32, 3454, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_m_u8, 3468, 3235},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_f16, 3481, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_f32, 3495, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_s16, 3509, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_s32, 3523, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_s8, 3537, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_u16, 3550, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_u32, 3564, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_n_u8, 3578, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_s16, 3591, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_s32, 3603, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_s8, 3615, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_u16, 3626, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_u32, 3638, 3203},
  { ARM::BI__builtin_arm_mve_vcmpeqq_u8, 3650, 3203},
  { ARM::BI__builtin_arm_mve_vcmpgeq_f16, 3669, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_f32, 3681, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_f16, 3703, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_f32, 3717, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_n_f16, 3731, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_n_f32, 3747, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_n_s16, 3763, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_n_s32, 3779, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_n_s8, 3795, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_s16, 3810, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_s32, 3824, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_m_s8, 3838, 3693},
  { ARM::BI__builtin_arm_mve_vcmpgeq_n_f16, 3851, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_n_f32, 3865, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_n_s16, 3879, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_n_s32, 3893, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_n_s8, 3907, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_s16, 3920, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_s32, 3932, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgeq_s8, 3944, 3661},
  { ARM::BI__builtin_arm_mve_vcmpgtq_f16, 3963, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_f32, 3975, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_f16, 3997, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_f32, 4011, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_n_f16, 4025, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_n_f32, 4041, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_n_s16, 4057, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_n_s32, 4073, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_n_s8, 4089, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_s16, 4104, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_s32, 4118, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_m_s8, 4132, 3987},
  { ARM::BI__builtin_arm_mve_vcmpgtq_n_f16, 4145, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_n_f32, 4159, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_n_s16, 4173, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_n_s32, 4187, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_n_s8, 4201, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_s16, 4214, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_s32, 4226, 3955},
  { ARM::BI__builtin_arm_mve_vcmpgtq_s8, 4238, 3955},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_n_u16, 4259, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_n_u32, 4275, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_n_u8, 4291, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_u16, 4306, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_u32, 4320, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_m_u8, 4334, 4249},
  { ARM::BI__builtin_arm_mve_vcmphiq_n_u16, 4355, 4347},
  { ARM::BI__builtin_arm_mve_vcmphiq_n_u32, 4369, 4347},
  { ARM::BI__builtin_arm_mve_vcmphiq_n_u8, 4383, 4347},
  { ARM::BI__builtin_arm_mve_vcmphiq_u16, 4396, 4347},
  { ARM::BI__builtin_arm_mve_vcmphiq_u32, 4408, 4347},
  { ARM::BI__builtin_arm_mve_vcmphiq_u8, 4420, 4347},
  { ARM::BI__builtin_arm_mve_vcmpleq_f16, 4439, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_f32, 4451, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_f16, 4473, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_f32, 4487, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_n_f16, 4501, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_n_f32, 4517, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_n_s16, 4533, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_n_s32, 4549, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_n_s8, 4565, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_s16, 4580, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_s32, 4594, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_m_s8, 4608, 4463},
  { ARM::BI__builtin_arm_mve_vcmpleq_n_f16, 4621, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_n_f32, 4635, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_n_s16, 4649, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_n_s32, 4663, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_n_s8, 4677, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_s16, 4690, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_s32, 4702, 4431},
  { ARM::BI__builtin_arm_mve_vcmpleq_s8, 4714, 4431},
  { ARM::BI__builtin_arm_mve_vcmpltq_f16, 4733, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_f32, 4745, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_f16, 4767, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_f32, 4781, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_n_f16, 4795, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_n_f32, 4811, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_n_s16, 4827, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_n_s32, 4843, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_n_s8, 4859, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_s16, 4874, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_s32, 4888, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_m_s8, 4902, 4757},
  { ARM::BI__builtin_arm_mve_vcmpltq_n_f16, 4915, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_n_f32, 4929, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_n_s16, 4943, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_n_s32, 4957, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_n_s8, 4971, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_s16, 4984, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_s32, 4996, 4725},
  { ARM::BI__builtin_arm_mve_vcmpltq_s8, 5008, 4725},
  { ARM::BI__builtin_arm_mve_vcmpneq_f16, 5027, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_f32, 5039, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_f16, 5061, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_f32, 5075, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_f16, 5089, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_f32, 5105, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_s16, 5121, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_s32, 5137, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_s8, 5153, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_u16, 5168, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_u32, 5184, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_n_u8, 5200, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_s16, 5215, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_s32, 5229, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_s8, 5243, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_u16, 5256, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_u32, 5270, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_m_u8, 5284, 5051},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_f16, 5297, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_f32, 5311, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_s16, 5325, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_s32, 5339, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_s8, 5353, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_u16, 5366, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_u32, 5380, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_n_u8, 5394, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_s16, 5407, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_s32, 5419, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_s8, 5431, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_u16, 5442, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_u32, 5454, 5019},
  { ARM::BI__builtin_arm_mve_vcmpneq_u8, 5466, 5019},
  { ARM::BI__builtin_arm_mve_vcmulq_f16, 5484, 5477},
  { ARM::BI__builtin_arm_mve_vcmulq_f32, 5495, 5477},
  { ARM::BI__builtin_arm_mve_vcmulq_m_f16, 5515, 5506},
  { ARM::BI__builtin_arm_mve_vcmulq_m_f32, 5528, 5506},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_f16, 5555, 5541},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_f32, 5573, 5541},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_m_f16, 5607, 5591},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_m_f32, 5627, 5591},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_x_f16, 5663, 5647},
  { ARM::BI__builtin_arm_mve_vcmulq_rot180_x_f32, 5683, 5647},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_f16, 5717, 5703},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_f32, 5735, 5703},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_m_f16, 5769, 5753},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_m_f32, 5789, 5753},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_x_f16, 5825, 5809},
  { ARM::BI__builtin_arm_mve_vcmulq_rot270_x_f32, 5845, 5809},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_f16, 5878, 5865},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_f32, 5895, 5865},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_m_f16, 5927, 5912},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_m_f32, 5946, 5912},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_x_f16, 5980, 5965},
  { ARM::BI__builtin_arm_mve_vcmulq_rot90_x_f32, 5999, 5965},
  { ARM::BI__builtin_arm_mve_vcmulq_x_f16, 6027, 6018},
  { ARM::BI__builtin_arm_mve_vcmulq_x_f32, 6040, 6018},
  { ARM::BI__builtin_arm_mve_vcreateq_f16, 6053, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_f32, 6066, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_s16, 6079, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_s32, 6092, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_s64, 6105, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_s8, 6118, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_u16, 6130, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_u32, 6143, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_u64, 6156, -1},
  { ARM::BI__builtin_arm_mve_vcreateq_u8, 6169, -1},
  { ARM::BI__builtin_arm_mve_vctp16q, 6181, -1},
  { ARM::BI__builtin_arm_mve_vctp16q_m, 6189, -1},
  { ARM::BI__builtin_arm_mve_vctp32q, 6199, -1},
  { ARM::BI__builtin_arm_mve_vctp32q_m, 6207, -1},
  { ARM::BI__builtin_arm_mve_vctp64q, 6217, -1},
  { ARM::BI__builtin_arm_mve_vctp64q_m, 6225, -1},
  { ARM::BI__builtin_arm_mve_vctp8q, 6235, -1},
  { ARM::BI__builtin_arm_mve_vctp8q_m, 6242, -1},
  { ARM::BI__builtin_arm_mve_vcvtbq_f16_f32, 6251, -1},
  { ARM::BI__builtin_arm_mve_vcvtbq_m_f16_f32, 6266, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_f16_s16, 6293, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_f16_u16, 6311, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_f32_s32, 6329, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_f32_u32, 6347, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_s16_f16, 6365, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_s32_f32, 6383, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_u16_f16, 6401, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_m_n_u32_f32, 6419, 6283},
  { ARM::BI__builtin_arm_mve_vcvtq_n_f16_s16, 6445, 6437},
  { ARM::BI__builtin_arm_mve_vcvtq_n_f16_u16, 6461, 6437},
  { ARM::BI__builtin_arm_mve_vcvtq_n_f32_s32, 6477, 6437},
  { ARM::BI__builtin_arm_mve_vcvtq_n_f32_u32, 6493, 6437},
  { ARM::BI__builtin_arm_mve_vcvtq_n_s16_f16, 6509, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_n_s32_f32, 6525, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_n_u16_f16, 6541, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_n_u32_f32, 6557, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_f16_s16, 6583, 6573},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_f16_u16, 6601, 6573},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_f32_s32, 6619, 6573},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_f32_u32, 6637, 6573},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_s16_f16, 6655, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_s32_f32, 6673, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_u16_f16, 6691, -1},
  { ARM::BI__builtin_arm_mve_vcvtq_x_n_u32_f32, 6709, -1},
  { ARM::BI__builtin_arm_mve_vcvttq_f16_f32, 6727, -1},
  { ARM::BI__builtin_arm_mve_vcvttq_m_f16_f32, 6742, -1},
  { ARM::BI__builtin_arm_mve_vddupq_m_n_u16, 6768, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_m_n_u32, 6783, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_m_n_u8, 6798, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_m_wb_u16, 6812, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_m_wb_u32, 6828, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_m_wb_u8, 6844, 6759},
  { ARM::BI__builtin_arm_mve_vddupq_n_u16, 6870, 6859},
  { ARM::BI__builtin_arm_mve_vddupq_n_u32, 6894, 6883},
  { ARM::BI__builtin_arm_mve_vddupq_n_u8, 6917, 6907},
  { ARM::BI__builtin_arm_mve_vddupq_wb_u16, 6929, 6859},
  { ARM::BI__builtin_arm_mve_vddupq_wb_u32, 6943, 6883},
  { ARM::BI__builtin_arm_mve_vddupq_wb_u8, 6957, 6907},
  { ARM::BI__builtin_arm_mve_vddupq_x_n_u16, 6983, 6970},
  { ARM::BI__builtin_arm_mve_vddupq_x_n_u32, 7011, 6998},
  { ARM::BI__builtin_arm_mve_vddupq_x_n_u8, 7038, 7026},
  { ARM::BI__builtin_arm_mve_vddupq_x_wb_u16, 7052, 6970},
  { ARM::BI__builtin_arm_mve_vddupq_x_wb_u32, 7068, 6998},
  { ARM::BI__builtin_arm_mve_vddupq_x_wb_u8, 7084, 7026},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_f16, 7107, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_f32, 7121, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_s16, 7135, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_s32, 7149, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_s8, 7163, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_u16, 7176, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_u32, 7190, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_m_n_u8, 7204, 7099},
  { ARM::BI__builtin_arm_mve_vdupq_n_f16, 7217, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_f32, 7229, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_s16, 7241, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_s32, 7253, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_s8, 7265, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_u16, 7276, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_u32, 7288, -1},
  { ARM::BI__builtin_arm_mve_vdupq_n_u8, 7300, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_f16, 7311, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_f32, 7325, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_s16, 7339, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_s32, 7353, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_s8, 7367, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_u16, 7380, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_u32, 7394, -1},
  { ARM::BI__builtin_arm_mve_vdupq_x_n_u8, 7408, -1},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_n_u16, 7431, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_n_u32, 7447, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_n_u8, 7463, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_wb_u16, 7478, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_wb_u32, 7495, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_m_wb_u8, 7512, 7421},
  { ARM::BI__builtin_arm_mve_vdwdupq_n_u16, 7540, 7528},
  { ARM::BI__builtin_arm_mve_vdwdupq_n_u32, 7566, 7554},
  { ARM::BI__builtin_arm_mve_vdwdupq_n_u8, 7591, 7580},
  { ARM::BI__builtin_arm_mve_vdwdupq_wb_u16, 7604, 7528},
  { ARM::BI__builtin_arm_mve_vdwdupq_wb_u32, 7619, 7554},
  { ARM::BI__builtin_arm_mve_vdwdupq_wb_u8, 7634, 7580},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_n_u16, 7662, 7648},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_n_u32, 7692, 7678},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_n_u8, 7721, 7708},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_wb_u16, 7736, 7648},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_wb_u32, 7753, 7678},
  { ARM::BI__builtin_arm_mve_vdwdupq_x_wb_u8, 7770, 7708},
  { ARM::BI__builtin_arm_mve_veorq_f16, 7792, 7786},
  { ARM::BI__builtin_arm_mve_veorq_f32, 7802, 7786},
  { ARM::BI__builtin_arm_mve_veorq_m_f16, 7820, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_f32, 7832, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_s16, 7844, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_s32, 7856, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_s8, 7868, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_u16, 7879, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_u32, 7891, 7812},
  { ARM::BI__builtin_arm_mve_veorq_m_u8, 7903, 7812},
  { ARM::BI__builtin_arm_mve_veorq_s16, 7914, 7786},
  { ARM::BI__builtin_arm_mve_veorq_s32, 7924, 7786},
  { ARM::BI__builtin_arm_mve_veorq_s8, 7934, 7786},
  { ARM::BI__builtin_arm_mve_veorq_u16, 7943, 7786},
  { ARM::BI__builtin_arm_mve_veorq_u32, 7953, 7786},
  { ARM::BI__builtin_arm_mve_veorq_u8, 7963, 7786},
  { ARM::BI__builtin_arm_mve_veorq_x_f16, 7980, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_f32, 7992, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_s16, 8004, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_s32, 8016, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_s8, 8028, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_u16, 8039, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_u32, 8051, 7972},
  { ARM::BI__builtin_arm_mve_veorq_x_u8, 8063, 7972},
  { ARM::BI__builtin_arm_mve_vgetq_lane_f16, 8085, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_f32, 8100, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_s16, 8115, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_s32, 8130, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_s64, 8145, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_s8, 8160, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_u16, 8174, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_u32, 8189, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_u64, 8204, 8074},
  { ARM::BI__builtin_arm_mve_vgetq_lane_u8, 8219, 8074},
  { ARM::BI__builtin_arm_mve_vhaddq_m_s16, 8242, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_m_s32, 8255, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_m_s8, 8268, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_m_u16, 8280, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_m_u32, 8293, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_m_u8, 8306, 8233},
  { ARM::BI__builtin_arm_mve_vhaddq_s16, 8325, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_s32, 8336, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_s8, 8347, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_u16, 8357, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_u32, 8368, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_u8, 8379, 8318},
  { ARM::BI__builtin_arm_mve_vhaddq_x_s16, 8398, 8389},
  { ARM::BI__builtin_arm_mve_vhaddq_x_s32, 8411, 8389},
  { ARM::BI__builtin_arm_mve_vhaddq_x_s8, 8424, 8389},
  { ARM::BI__builtin_arm_mve_vhaddq_x_u16, 8436, 8389},
  { ARM::BI__builtin_arm_mve_vhaddq_x_u32, 8449, 8389},
  { ARM::BI__builtin_arm_mve_vhaddq_x_u8, 8462, 8389},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_m_s16, 8491, 8474},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_m_s32, 8512, 8474},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_m_s8, 8533, 8474},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_s16, 8568, 8553},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_s32, 8587, 8553},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_s8, 8606, 8553},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_x_s16, 8641, 8624},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_x_s32, 8662, 8624},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot270_x_s8, 8683, 8624},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_m_s16, 8719, 8703},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_m_s32, 8739, 8703},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_m_s8, 8759, 8703},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_s16, 8792, 8778},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_s32, 8810, 8778},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_s8, 8828, 8778},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_x_s16, 8861, 8845},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_x_s32, 8881, 8845},
  { ARM::BI__builtin_arm_mve_vhcaddq_rot90_x_s8, 8901, 8845},
  { ARM::BI__builtin_arm_mve_vhsubq_m_s16, 8929, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_m_s32, 8942, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_m_s8, 8955, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_m_u16, 8967, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_m_u32, 8980, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_m_u8, 8993, 8920},
  { ARM::BI__builtin_arm_mve_vhsubq_s16, 9012, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_s32, 9023, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_s8, 9034, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_u16, 9044, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_u32, 9055, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_u8, 9066, 9005},
  { ARM::BI__builtin_arm_mve_vhsubq_x_s16, 9085, 9076},
  { ARM::BI__builtin_arm_mve_vhsubq_x_s32, 9098, 9076},
  { ARM::BI__builtin_arm_mve_vhsubq_x_s8, 9111, 9076},
  { ARM::BI__builtin_arm_mve_vhsubq_x_u16, 9123, 9076},
  { ARM::BI__builtin_arm_mve_vhsubq_x_u32, 9136, 9076},
  { ARM::BI__builtin_arm_mve_vhsubq_x_u8, 9149, 9076},
  { ARM::BI__builtin_arm_mve_vidupq_m_n_u16, 9170, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_m_n_u32, 9185, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_m_n_u8, 9200, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_m_wb_u16, 9214, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_m_wb_u32, 9230, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_m_wb_u8, 9246, 9161},
  { ARM::BI__builtin_arm_mve_vidupq_n_u16, 9272, 9261},
  { ARM::BI__builtin_arm_mve_vidupq_n_u32, 9296, 9285},
  { ARM::BI__builtin_arm_mve_vidupq_n_u8, 9319, 9309},
  { ARM::BI__builtin_arm_mve_vidupq_wb_u16, 9331, 9261},
  { ARM::BI__builtin_arm_mve_vidupq_wb_u32, 9345, 9285},
  { ARM::BI__builtin_arm_mve_vidupq_wb_u8, 9359, 9309},
  { ARM::BI__builtin_arm_mve_vidupq_x_n_u16, 9385, 9372},
  { ARM::BI__builtin_arm_mve_vidupq_x_n_u32, 9413, 9400},
  { ARM::BI__builtin_arm_mve_vidupq_x_n_u8, 9440, 9428},
  { ARM::BI__builtin_arm_mve_vidupq_x_wb_u16, 9454, 9372},
  { ARM::BI__builtin_arm_mve_vidupq_x_wb_u32, 9470, 9400},
  { ARM::BI__builtin_arm_mve_vidupq_x_wb_u8, 9486, 9428},
  { ARM::BI__builtin_arm_mve_viwdupq_m_n_u16, 9511, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_m_n_u32, 9527, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_m_n_u8, 9543, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_m_wb_u16, 9558, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_m_wb_u32, 9575, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_m_wb_u8, 9592, 9501},
  { ARM::BI__builtin_arm_mve_viwdupq_n_u16, 9620, 9608},
  { ARM::BI__builtin_arm_mve_viwdupq_n_u32, 9646, 9634},
  { ARM::BI__builtin_arm_mve_viwdupq_n_u8, 9671, 9660},
  { ARM::BI__builtin_arm_mve_viwdupq_wb_u16, 9684, 9608},
  { ARM::BI__builtin_arm_mve_viwdupq_wb_u32, 9699, 9634},
  { ARM::BI__builtin_arm_mve_viwdupq_wb_u8, 9714, 9660},
  { ARM::BI__builtin_arm_mve_viwdupq_x_n_u16, 9742, 9728},
  { ARM::BI__builtin_arm_mve_viwdupq_x_n_u32, 9772, 9758},
  { ARM::BI__builtin_arm_mve_viwdupq_x_n_u8, 9801, 9788},
  { ARM::BI__builtin_arm_mve_viwdupq_x_wb_u16, 9816, 9728},
  { ARM::BI__builtin_arm_mve_viwdupq_x_wb_u32, 9833, 9758},
  { ARM::BI__builtin_arm_mve_viwdupq_x_wb_u8, 9850, 9788},
  { ARM::BI__builtin_arm_mve_vld1q_f16, 9872, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_f32, 9882, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_s16, 9892, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_s32, 9902, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_s8, 9912, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_u16, 9921, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_u32, 9931, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_u8, 9941, 9866},
  { ARM::BI__builtin_arm_mve_vld1q_z_f16, 9958, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_f32, 9970, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_s16, 9982, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_s32, 9994, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_s8, 10006, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_u16, 10017, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_u32, 10029, 9950},
  { ARM::BI__builtin_arm_mve_vld1q_z_u8, 10041, 9950},
  { ARM::BI__builtin_arm_mve_vld2q_f16, 10058, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_f32, 10068, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_s16, 10078, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_s32, 10088, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_s8, 10098, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_u16, 10107, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_u32, 10117, 10052},
  { ARM::BI__builtin_arm_mve_vld2q_u8, 10127, 10052},
  { ARM::BI__builtin_arm_mve_vld4q_f16, 10142, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_f32, 10152, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_s16, 10162, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_s32, 10172, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_s8, 10182, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_u16, 10191, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_u32, 10201, 10136},
  { ARM::BI__builtin_arm_mve_vld4q_u8, 10211, 10136},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_s16, 10241, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_s32, 10266, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_s8, 10291, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_u16, 10315, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_u32, 10340, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_u8, 10365, 10220},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_s16, 10412, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_s32, 10439, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_s8, 10466, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_u16, 10492, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_u32, 10519, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_gather_offset_z_u8, 10546, 10389},
  { ARM::BI__builtin_arm_mve_vldrbq_s16, 10572, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_s32, 10583, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_s8, 10594, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_u16, 10604, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_u32, 10615, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_u8, 10626, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_s16, 10636, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_s32, 10649, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_s8, 10662, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_u16, 10674, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_u32, 10687, -1},
  { ARM::BI__builtin_arm_mve_vldrbq_z_u8, 10700, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_s64, 10712, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_u64, 10735, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_wb_s64, 10758, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_wb_u64, 10784, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_wb_z_s64, 10810, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_wb_z_u64, 10838, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_z_s64, 10866, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_base_z_u64, 10891, -1},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_offset_s64, 10937, 10916},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_offset_u64, 10962, 10916},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_offset_z_s64, 11010, 10987},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_offset_z_u64, 11037, 10987},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_shifted_offset_s64, 11093, 11064},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_shifted_offset_u64, 11126, 11064},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_shifted_offset_z_s64, 11190, 11159},
  { ARM::BI__builtin_arm_mve_vldrdq_gather_shifted_offset_z_u64, 11225, 11159},
  { ARM::BI__builtin_arm_mve_vldrhq_f16, 11260, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_f16, 11292, 11271},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_s16, 11317, 11271},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_s32, 11342, 11271},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_u16, 11367, 11271},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_u32, 11392, 11271},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_z_f16, 11440, 11417},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_z_s16, 11467, 11417},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_z_s32, 11494, 11417},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_z_u16, 11521, 11417},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_offset_z_u32, 11548, 11417},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_f16, 11604, 11575},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_s16, 11637, 11575},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_s32, 11670, 11575},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_u16, 11703, 11575},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_u32, 11736, 11575},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_z_f16, 11800, 11769},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_z_s16, 11835, 11769},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_z_s32, 11870, 11769},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_z_u16, 11905, 11769},
  { ARM::BI__builtin_arm_mve_vldrhq_gather_shifted_offset_z_u32, 11940, 11769},
  { ARM::BI__builtin_arm_mve_vldrhq_s16, 11975, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_s32, 11986, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_u16, 11997, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_u32, 12008, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_z_f16, 12019, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_z_s16, 12032, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_z_s32, 12045, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_z_u16, 12058, -1},
  { ARM::BI__builtin_arm_mve_vldrhq_z_u32, 12071, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_f32, 12084, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_f32, 12095, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_s32, 12118, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_u32, 12141, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_f32, 12164, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_s32, 12190, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_u32, 12216, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_z_f32, 12242, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_z_s32, 12270, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_wb_z_u32, 12298, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_z_f32, 12326, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_z_s32, 12351, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_base_z_u32, 12376, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_f32, 12422, 12401},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_s32, 12447, 12401},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_u32, 12472, 12401},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_z_f32, 12520, 12497},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_z_s32, 12547, 12497},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_offset_z_u32, 12574, 12497},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_f32, 12630, 12601},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_s32, 12663, 12601},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_u32, 12696, 12601},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_z_f32, 12760, 12729},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_z_s32, 12795, 12729},
  { ARM::BI__builtin_arm_mve_vldrwq_gather_shifted_offset_z_u32, 12830, 12729},
  { ARM::BI__builtin_arm_mve_vldrwq_s32, 12865, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_u32, 12876, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_z_f32, 12887, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_z_s32, 12900, -1},
  { ARM::BI__builtin_arm_mve_vldrwq_z_u32, 12913, -1},
  { ARM::BI__builtin_arm_mve_vmaxaq_m_s16, 12935, 12926},
  { ARM::BI__builtin_arm_mve_vmaxaq_m_s32, 12948, 12926},
  { ARM::BI__builtin_arm_mve_vmaxaq_m_s8, 12961, 12926},
  { ARM::BI__builtin_arm_mve_vmaxaq_s16, 12980, 12973},
  { ARM::BI__builtin_arm_mve_vmaxaq_s32, 12991, 12973},
  { ARM::BI__builtin_arm_mve_vmaxaq_s8, 13002, 12973},
  { ARM::BI__builtin_arm_mve_vmaxnmaq_f16, 13021, 13012},
  { ARM::BI__builtin_arm_mve_vmaxnmaq_f32, 13034, 13012},
  { ARM::BI__builtin_arm_mve_vmaxnmaq_m_f16, 13058, 13047},
  { ARM::BI__builtin_arm_mve_vmaxnmaq_m_f32, 13073, 13047},
  { ARM::BI__builtin_arm_mve_vmaxnmq_f16, 13096, 13088},
  { ARM::BI__builtin_arm_mve_vmaxnmq_f32, 13108, 13088},
  { ARM::BI__builtin_arm_mve_vmaxnmq_m_f16, 13130, 13120},
  { ARM::BI__builtin_arm_mve_vmaxnmq_m_f32, 13144, 13120},
  { ARM::BI__builtin_arm_mve_vmaxnmq_x_f16, 13168, 13158},
  { ARM::BI__builtin_arm_mve_vmaxnmq_x_f32, 13182, 13158},
  { ARM::BI__builtin_arm_mve_vmaxq_m_s16, 13204, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_m_s32, 13216, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_m_s8, 13228, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_m_u16, 13239, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_m_u32, 13251, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_m_u8, 13263, 13196},
  { ARM::BI__builtin_arm_mve_vmaxq_s16, 13280, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_s32, 13290, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_s8, 13300, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_u16, 13309, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_u32, 13319, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_u8, 13329, 13274},
  { ARM::BI__builtin_arm_mve_vmaxq_x_s16, 13346, 13338},
  { ARM::BI__builtin_arm_mve_vmaxq_x_s32, 13358, 13338},
  { ARM::BI__builtin_arm_mve_vmaxq_x_s8, 13370, 13338},
  { ARM::BI__builtin_arm_mve_vmaxq_x_u16, 13381, 13338},
  { ARM::BI__builtin_arm_mve_vmaxq_x_u32, 13393, 13338},
  { ARM::BI__builtin_arm_mve_vmaxq_x_u8, 13405, 13338},
  { ARM::BI__builtin_arm_mve_vmaxvq_s16, 13423, 13416},
  { ARM::BI__builtin_arm_mve_vmaxvq_s32, 13434, 13416},
  { ARM::BI__builtin_arm_mve_vmaxvq_s8, 13445, 13416},
  { ARM::BI__builtin_arm_mve_vmaxvq_u16, 13455, 13416},
  { ARM::BI__builtin_arm_mve_vmaxvq_u32, 13466, 13416},
  { ARM::BI__builtin_arm_mve_vmaxvq_u8, 13477, 13416},
  { ARM::BI__builtin_arm_mve_vminaq_m_s16, 13496, 13487},
  { ARM::BI__builtin_arm_mve_vminaq_m_s32, 13509, 13487},
  { ARM::BI__builtin_arm_mve_vminaq_m_s8, 13522, 13487},
  { ARM::BI__builtin_arm_mve_vminaq_s16, 13541, 13534},
  { ARM::BI__builtin_arm_mve_vminaq_s32, 13552, 13534},
  { ARM::BI__builtin_arm_mve_vminaq_s8, 13563, 13534},
  { ARM::BI__builtin_arm_mve_vminnmaq_f16, 13582, 13573},
  { ARM::BI__builtin_arm_mve_vminnmaq_f32, 13595, 13573},
  { ARM::BI__builtin_arm_mve_vminnmaq_m_f16, 13619, 13608},
  { ARM::BI__builtin_arm_mve_vminnmaq_m_f32, 13634, 13608},
  { ARM::BI__builtin_arm_mve_vminnmq_f16, 13657, 13649},
  { ARM::BI__builtin_arm_mve_vminnmq_f32, 13669, 13649},
  { ARM::BI__builtin_arm_mve_vminnmq_m_f16, 13691, 13681},
  { ARM::BI__builtin_arm_mve_vminnmq_m_f32, 13705, 13681},
  { ARM::BI__builtin_arm_mve_vminnmq_x_f16, 13729, 13719},
  { ARM::BI__builtin_arm_mve_vminnmq_x_f32, 13743, 13719},
  { ARM::BI__builtin_arm_mve_vminq_m_s16, 13765, 13757},
  { ARM::BI__builtin_arm_mve_vminq_m_s32, 13777, 13757},
  { ARM::BI__builtin_arm_mve_vminq_m_s8, 13789, 13757},
  { ARM::BI__builtin_arm_mve_vminq_m_u16, 13800, 13757},
  { ARM::BI__builtin_arm_mve_vminq_m_u32, 13812, 13757},
  { ARM::BI__builtin_arm_mve_vminq_m_u8, 13824, 13757},
  { ARM::BI__builtin_arm_mve_vminq_s16, 13841, 13835},
  { ARM::BI__builtin_arm_mve_vminq_s32, 13851, 13835},
  { ARM::BI__builtin_arm_mve_vminq_s8, 13861, 13835},
  { ARM::BI__builtin_arm_mve_vminq_u16, 13870, 13835},
  { ARM::BI__builtin_arm_mve_vminq_u32, 13880, 13835},
  { ARM::BI__builtin_arm_mve_vminq_u8, 13890, 13835},
  { ARM::BI__builtin_arm_mve_vminq_x_s16, 13907, 13899},
  { ARM::BI__builtin_arm_mve_vminq_x_s32, 13919, 13899},
  { ARM::BI__builtin_arm_mve_vminq_x_s8, 13931, 13899},
  { ARM::BI__builtin_arm_mve_vminq_x_u16, 13942, 13899},
  { ARM::BI__builtin_arm_mve_vminq_x_u32, 13954, 13899},
  { ARM::BI__builtin_arm_mve_vminq_x_u8, 13966, 13899},
  { ARM::BI__builtin_arm_mve_vminvq_s16, 13984, 13977},
  { ARM::BI__builtin_arm_mve_vminvq_s32, 13995, 13977},
  { ARM::BI__builtin_arm_mve_vminvq_s8, 14006, 13977},
  { ARM::BI__builtin_arm_mve_vminvq_u16, 14016, 13977},
  { ARM::BI__builtin_arm_mve_vminvq_u32, 14027, 13977},
  { ARM::BI__builtin_arm_mve_vminvq_u8, 14038, 13977},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_s16, 14060, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_s32, 14076, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_s8, 14092, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_u16, 14107, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_u32, 14123, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_p_u8, 14139, 14048},
  { ARM::BI__builtin_arm_mve_vmladavaq_s16, 14164, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaq_s32, 14178, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaq_s8, 14192, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaq_u16, 14205, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaq_u32, 14219, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaq_u8, 14233, 14154},
  { ARM::BI__builtin_arm_mve_vmladavaxq_p_s16, 14259, 14246},
  { ARM::BI__builtin_arm_mve_vmladavaxq_p_s32, 14276, 14246},
  { ARM::BI__builtin_arm_mve_vmladavaxq_p_s8, 14293, 14246},
  { ARM::BI__builtin_arm_mve_vmladavaxq_s16, 14320, 14309},
  { ARM::BI__builtin_arm_mve_vmladavaxq_s32, 14335, 14309},
  { ARM::BI__builtin_arm_mve_vmladavaxq_s8, 14350, 14309},
  { ARM::BI__builtin_arm_mve_vmladavq_p_s16, 14375, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_p_s32, 14390, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_p_s8, 14405, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_p_u16, 14419, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_p_u32, 14434, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_p_u8, 14449, 14364},
  { ARM::BI__builtin_arm_mve_vmladavq_s16, 14472, 14463},
  { ARM::BI__builtin_arm_mve_vmladavq_s32, 14485, 14463},
  { ARM::BI__builtin_arm_mve_vmladavq_s8, 14498, 14463},
  { ARM::BI__builtin_arm_mve_vmladavq_u16, 14510, 14463},
  { ARM::BI__builtin_arm_mve_vmladavq_u32, 14523, 14463},
  { ARM::BI__builtin_arm_mve_vmladavq_u8, 14536, 14463},
  { ARM::BI__builtin_arm_mve_vmladavxq_p_s16, 14560, 14548},
  { ARM::BI__builtin_arm_mve_vmladavxq_p_s32, 14576, 14548},
  { ARM::BI__builtin_arm_mve_vmladavxq_p_s8, 14592, 14548},
  { ARM::BI__builtin_arm_mve_vmladavxq_s16, 14617, 14607},
  { ARM::BI__builtin_arm_mve_vmladavxq_s32, 14631, 14607},
  { ARM::BI__builtin_arm_mve_vmladavxq_s8, 14645, 14607},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_p_s16, 14671, 14658},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_p_s32, 14688, 14658},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_p_u16, 14705, 14658},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_p_u32, 14722, 14658},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_s16, 14750, 14739},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_s32, 14765, 14739},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_u16, 14780, 14739},
  { ARM::BI__builtin_arm_mve_vmlaldavaq_u32, 14795, 14739},
  { ARM::BI__builtin_arm_mve_vmlaldavaxq_p_s16, 14824, 14810},
  { ARM::BI__builtin_arm_mve_vmlaldavaxq_p_s32, 14842, 14810},
  { ARM::BI__builtin_arm_mve_vmlaldavaxq_s16, 14872, 14860},
  { ARM::BI__builtin_arm_mve_vmlaldavaxq_s32, 14888, 14860},
  { ARM::BI__builtin_arm_mve_vmlaldavq_p_s16, 14916, 14904},
  { ARM::BI__builtin_arm_mve_vmlaldavq_p_s32, 14932, 14904},
  { ARM::BI__builtin_arm_mve_vmlaldavq_p_u16, 14948, 14904},
  { ARM::BI__builtin_arm_mve_vmlaldavq_p_u32, 14964, 14904},
  { ARM::BI__builtin_arm_mve_vmlaldavq_s16, 14990, 14980},
  { ARM::BI__builtin_arm_mve_vmlaldavq_s32, 15004, 14980},
  { ARM::BI__builtin_arm_mve_vmlaldavq_u16, 15018, 14980},
  { ARM::BI__builtin_arm_mve_vmlaldavq_u32, 15032, 14980},
  { ARM::BI__builtin_arm_mve_vmlaldavxq_p_s16, 15059, 15046},
  { ARM::BI__builtin_arm_mve_vmlaldavxq_p_s32, 15076, 15046},
  { ARM::BI__builtin_arm_mve_vmlaldavxq_s16, 15104, 15093},
  { ARM::BI__builtin_arm_mve_vmlaldavxq_s32, 15119, 15093},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_p_s16, 15146, 15134},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_p_s32, 15162, 15134},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_p_s8, 15178, 15134},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_s16, 15203, 15193},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_s32, 15217, 15193},
  { ARM::BI__builtin_arm_mve_vmlsdavaq_s8, 15231, 15193},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_p_s16, 15257, 15244},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_p_s32, 15274, 15244},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_p_s8, 15291, 15244},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_s16, 15318, 15307},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_s32, 15333, 15307},
  { ARM::BI__builtin_arm_mve_vmlsdavaxq_s8, 15348, 15307},
  { ARM::BI__builtin_arm_mve_vmlsdavq_p_s16, 15373, 15362},
  { ARM::BI__builtin_arm_mve_vmlsdavq_p_s32, 15388, 15362},
  { ARM::BI__builtin_arm_mve_vmlsdavq_p_s8, 15403, 15362},
  { ARM::BI__builtin_arm_mve_vmlsdavq_s16, 15426, 15417},
  { ARM::BI__builtin_arm_mve_vmlsdavq_s32, 15439, 15417},
  { ARM::BI__builtin_arm_mve_vmlsdavq_s8, 15452, 15417},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_p_s16, 15476, 15464},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_p_s32, 15492, 15464},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_p_s8, 15508, 15464},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_s16, 15533, 15523},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_s32, 15547, 15523},
  { ARM::BI__builtin_arm_mve_vmlsdavxq_s8, 15561, 15523},
  { ARM::BI__builtin_arm_mve_vmlsldavaq_p_s16, 15587, 15574},
  { ARM::BI__builtin_arm_mve_vmlsldavaq_p_s32, 15604, 15574},
  { ARM::BI__builtin_arm_mve_vmlsldavaq_s16, 15632, 15621},
  { ARM::BI__builtin_arm_mve_vmlsldavaq_s32, 15647, 15621},
  { ARM::BI__builtin_arm_mve_vmlsldavaxq_p_s16, 15676, 15662},
  { ARM::BI__builtin_arm_mve_vmlsldavaxq_p_s32, 15694, 15662},
  { ARM::BI__builtin_arm_mve_vmlsldavaxq_s16, 15724, 15712},
  { ARM::BI__builtin_arm_mve_vmlsldavaxq_s32, 15740, 15712},
  { ARM::BI__builtin_arm_mve_vmlsldavq_p_s16, 15768, 15756},
  { ARM::BI__builtin_arm_mve_vmlsldavq_p_s32, 15784, 15756},
  { ARM::BI__builtin_arm_mve_vmlsldavq_s16, 15810, 15800},
  { ARM::BI__builtin_arm_mve_vmlsldavq_s32, 15824, 15800},
  { ARM::BI__builtin_arm_mve_vmlsldavxq_p_s16, 15851, 15838},
  { ARM::BI__builtin_arm_mve_vmlsldavxq_p_s32, 15868, 15838},
  { ARM::BI__builtin_arm_mve_vmlsldavxq_s16, 15896, 15885},
  { ARM::BI__builtin_arm_mve_vmlsldavxq_s32, 15911, 15885},
  { ARM::BI__builtin_arm_mve_vmulhq_m_s16, 15935, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_m_s32, 15948, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_m_s8, 15961, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_m_u16, 15973, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_m_u32, 15986, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_m_u8, 15999, 15926},
  { ARM::BI__builtin_arm_mve_vmulhq_s16, 16018, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_s32, 16029, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_s8, 16040, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_u16, 16050, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_u32, 16061, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_u8, 16072, 16011},
  { ARM::BI__builtin_arm_mve_vmulhq_x_s16, 16091, 16082},
  { ARM::BI__builtin_arm_mve_vmulhq_x_s32, 16104, 16082},
  { ARM::BI__builtin_arm_mve_vmulhq_x_s8, 16117, 16082},
  { ARM::BI__builtin_arm_mve_vmulhq_x_u16, 16129, 16082},
  { ARM::BI__builtin_arm_mve_vmulhq_x_u32, 16142, 16082},
  { ARM::BI__builtin_arm_mve_vmulhq_x_u8, 16155, 16082},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_s16, 16181, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_s32, 16199, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_s8, 16217, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_u16, 16234, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_u32, 16252, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_m_u8, 16270, 16167},
  { ARM::BI__builtin_arm_mve_vmullbq_int_s16, 16299, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_s32, 16315, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_s8, 16331, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_u16, 16346, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_u32, 16362, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_u8, 16378, 16287},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_s16, 16407, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_s32, 16425, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_s8, 16443, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_u16, 16460, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_u32, 16478, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_int_x_u8, 16496, 16393},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_m_p16, 16528, 16513},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_m_p8, 16547, 16513},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_p16, 16578, 16565},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_p8, 16595, 16565},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_x_p16, 16626, 16611},
  { ARM::BI__builtin_arm_mve_vmullbq_poly_x_p8, 16645, 16611},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_s16, 16677, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_s32, 16695, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_s8, 16713, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_u16, 16730, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_u32, 16748, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_m_u8, 16766, 16663},
  { ARM::BI__builtin_arm_mve_vmulltq_int_s16, 16795, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_s32, 16811, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_s8, 16827, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_u16, 16842, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_u32, 16858, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_u8, 16874, 16783},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_s16, 16903, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_s32, 16921, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_s8, 16939, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_u16, 16956, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_u32, 16974, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_int_x_u8, 16992, 16889},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_m_p16, 17024, 17009},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_m_p8, 17043, 17009},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_p16, 17074, 17061},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_p8, 17091, 17061},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_x_p16, 17122, 17107},
  { ARM::BI__builtin_arm_mve_vmulltq_poly_x_p8, 17141, 17107},
  { ARM::BI__builtin_arm_mve_vmulq_f16, 17165, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_f32, 17175, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_m_f16, 17193, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_f32, 17205, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_s16, 17217, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_s32, 17229, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_s8, 17241, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_u16, 17252, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_u32, 17264, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_m_u8, 17276, 17185},
  { ARM::BI__builtin_arm_mve_vmulq_s16, 17287, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_s32, 17297, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_s8, 17307, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_u16, 17316, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_u32, 17326, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_u8, 17336, 17159},
  { ARM::BI__builtin_arm_mve_vmulq_x_f16, 17353, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_f32, 17365, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_s16, 17377, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_s32, 17389, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_s8, 17401, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_u16, 17412, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_u32, 17424, 17345},
  { ARM::BI__builtin_arm_mve_vmulq_x_u8, 17436, 17345},
  { ARM::BI__builtin_arm_mve_vmvnq_m_n_s16, 17455, 17447},
  { ARM::BI__builtin_arm_mve_vmvnq_m_n_s32, 17469, 17447},
  { ARM::BI__builtin_arm_mve_vmvnq_m_n_u16, 17483, 17447},
  { ARM::BI__builtin_arm_mve_vmvnq_m_n_u32, 17497, 17447},
  { ARM::BI__builtin_arm_mve_vmvnq_n_s16, 17511, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_n_s32, 17523, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_n_u16, 17535, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_n_u32, 17547, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_x_n_s16, 17559, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_x_n_s32, 17573, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_x_n_u16, 17587, -1},
  { ARM::BI__builtin_arm_mve_vmvnq_x_n_u32, 17601, -1},
  { ARM::BI__builtin_arm_mve_vornq_f16, 17621, 17615},
  { ARM::BI__builtin_arm_mve_vornq_f32, 17631, 17615},
  { ARM::BI__builtin_arm_mve_vornq_m_f16, 17649, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_f32, 17661, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_s16, 17673, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_s32, 17685, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_s8, 17697, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_u16, 17708, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_u32, 17720, 17641},
  { ARM::BI__builtin_arm_mve_vornq_m_u8, 17732, 17641},
  { ARM::BI__builtin_arm_mve_vornq_s16, 17743, 17615},
  { ARM::BI__builtin_arm_mve_vornq_s32, 17753, 17615},
  { ARM::BI__builtin_arm_mve_vornq_s8, 17763, 17615},
  { ARM::BI__builtin_arm_mve_vornq_u16, 17772, 17615},
  { ARM::BI__builtin_arm_mve_vornq_u32, 17782, 17615},
  { ARM::BI__builtin_arm_mve_vornq_u8, 17792, 17615},
  { ARM::BI__builtin_arm_mve_vornq_x_f16, 17809, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_f32, 17821, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_s16, 17833, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_s32, 17845, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_s8, 17857, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_u16, 17868, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_u32, 17880, 17801},
  { ARM::BI__builtin_arm_mve_vornq_x_u8, 17892, 17801},
  { ARM::BI__builtin_arm_mve_vorrq_f16, 17909, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_f32, 17919, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_m_f16, 17937, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_f32, 17949, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_n_s16, 17971, 17961},
  { ARM::BI__builtin_arm_mve_vorrq_m_n_s32, 17985, 17961},
  { ARM::BI__builtin_arm_mve_vorrq_m_n_u16, 17999, 17961},
  { ARM::BI__builtin_arm_mve_vorrq_m_n_u32, 18013, 17961},
  { ARM::BI__builtin_arm_mve_vorrq_m_s16, 18027, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_s32, 18039, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_s8, 18051, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_u16, 18062, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_u32, 18074, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_m_u8, 18086, 17929},
  { ARM::BI__builtin_arm_mve_vorrq_n_s16, 18097, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_n_s32, 18109, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_n_u16, 18121, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_n_u32, 18133, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_s16, 18145, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_s32, 18155, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_s8, 18165, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_u16, 18174, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_u32, 18184, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_u8, 18194, 17903},
  { ARM::BI__builtin_arm_mve_vorrq_x_f16, 18211, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_f32, 18223, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_s16, 18235, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_s32, 18247, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_s8, 18259, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_u16, 18270, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_u32, 18282, 18203},
  { ARM::BI__builtin_arm_mve_vorrq_x_u8, 18294, 18203},
  { ARM::BI__builtin_arm_mve_vpnot, 18305, -1},
  { ARM::BI__builtin_arm_mve_vpselq_f16, 18318, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_f32, 18329, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_s16, 18340, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_s32, 18351, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_s64, 18362, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_s8, 18373, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_u16, 18383, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_u32, 18394, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_u64, 18405, 18311},
  { ARM::BI__builtin_arm_mve_vpselq_u8, 18416, 18311},
  { ARM::BI__builtin_arm_mve_vqaddq_m_s16, 18435, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_m_s32, 18448, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_m_s8, 18461, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_m_u16, 18473, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_m_u32, 18486, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_m_u8, 18499, 18426},
  { ARM::BI__builtin_arm_mve_vqaddq_s16, 18518, 18511},
  { ARM::BI__builtin_arm_mve_vqaddq_s32, 18529, 18511},
  { ARM::BI__builtin_arm_mve_vqaddq_s8, 18540, 18511},
  { ARM::BI__builtin_arm_mve_vqaddq_u16, 18550, 18511},
  { ARM::BI__builtin_arm_mve_vqaddq_u32, 18561, 18511},
  { ARM::BI__builtin_arm_mve_vqaddq_u8, 18572, 18511},
  { ARM::BI__builtin_arm_mve_vqdmulhq_m_s16, 18593, 18582},
  { ARM::BI__builtin_arm_mve_vqdmulhq_m_s32, 18608, 18582},
  { ARM::BI__builtin_arm_mve_vqdmulhq_m_s8, 18623, 18582},
  { ARM::BI__builtin_arm_mve_vqdmulhq_s16, 18646, 18637},
  { ARM::BI__builtin_arm_mve_vqdmulhq_s32, 18659, 18637},
  { ARM::BI__builtin_arm_mve_vqdmulhq_s8, 18672, 18637},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_m_s16, 18696, 18684},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_m_s32, 18712, 18684},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_m_s8, 18728, 18684},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_s16, 18753, 18743},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_s32, 18767, 18743},
  { ARM::BI__builtin_arm_mve_vqrdmulhq_s8, 18781, 18743},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_s16, 18806, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_s32, 18822, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_s8, 18838, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_u16, 18853, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_u32, 18869, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_n_u8, 18885, 18794},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_s16, 18910, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_s32, 18924, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_s8, 18938, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_u16, 18951, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_u32, 18965, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_m_u8, 18979, 18900},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_s16, 19000, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_s32, 19014, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_s8, 19028, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_u16, 19041, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_u32, 19055, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_n_u8, 19069, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_s16, 19082, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_s32, 19094, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_s8, 19106, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_u16, 19117, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_u32, 19129, 18992},
  { ARM::BI__builtin_arm_mve_vqrshlq_u8, 19141, 18992},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_m_n_s16, 19164, 19152},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_m_n_s32, 19182, 19152},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_m_n_u16, 19200, 19152},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_m_n_u32, 19218, 19152},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_n_s16, 19246, 19236},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_n_s32, 19262, 19236},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_n_u16, 19278, 19236},
  { ARM::BI__builtin_arm_mve_vqrshrnbq_n_u32, 19294, 19236},
  { ARM::BI__builtin_arm_mve_vqrshrntq_m_n_s16, 19322, 19310},
  { ARM::BI__builtin_arm_mve_vqrshrntq_m_n_s32, 19340, 19310},
  { ARM::BI__builtin_arm_mve_vqrshrntq_m_n_u16, 19358, 19310},
  { ARM::BI__builtin_arm_mve_vqrshrntq_m_n_u32, 19376, 19310},
  { ARM::BI__builtin_arm_mve_vqrshrntq_n_s16, 19404, 19394},
  { ARM::BI__builtin_arm_mve_vqrshrntq_n_s32, 19420, 19394},
  { ARM::BI__builtin_arm_mve_vqrshrntq_n_u16, 19436, 19394},
  { ARM::BI__builtin_arm_mve_vqrshrntq_n_u32, 19452, 19394},
  { ARM::BI__builtin_arm_mve_vqrshrunbq_m_n_s16, 19481, 19468},
  { ARM::BI__builtin_arm_mve_vqrshrunbq_m_n_s32, 19500, 19468},
  { ARM::BI__builtin_arm_mve_vqrshrunbq_n_s16, 19530, 19519},
  { ARM::BI__builtin_arm_mve_vqrshrunbq_n_s32, 19547, 19519},
  { ARM::BI__builtin_arm_mve_vqrshruntq_m_n_s16, 19577, 19564},
  { ARM::BI__builtin_arm_mve_vqrshruntq_m_n_s32, 19596, 19564},
  { ARM::BI__builtin_arm_mve_vqrshruntq_n_s16, 19626, 19615},
  { ARM::BI__builtin_arm_mve_vqrshruntq_n_s32, 19643, 19615},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_s16, 19671, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_s32, 19686, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_s8, 19701, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_u16, 19715, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_u32, 19730, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_n_u8, 19745, 19660},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_s16, 19770, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_s32, 19785, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_s8, 19800, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_u16, 19814, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_u32, 19829, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_r_u8, 19844, 19759},
  { ARM::BI__builtin_arm_mve_vqshlq_m_s16, 19867, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_m_s32, 19880, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_m_s8, 19893, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_m_u16, 19905, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_m_u32, 19918, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_m_u8, 19931, 19858},
  { ARM::BI__builtin_arm_mve_vqshlq_n_s16, 19952, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_n_s32, 19965, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_n_s8, 19978, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_n_u16, 19990, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_n_u32, 20003, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_n_u8, 20016, 19943},
  { ARM::BI__builtin_arm_mve_vqshlq_r_s16, 20037, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_r_s32, 20050, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_r_s8, 20063, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_r_u16, 20075, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_r_u32, 20088, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_r_u8, 20101, 20028},
  { ARM::BI__builtin_arm_mve_vqshlq_s16, 20120, 20113},
  { ARM::BI__builtin_arm_mve_vqshlq_s32, 20131, 20113},
  { ARM::BI__builtin_arm_mve_vqshlq_s8, 20142, 20113},
  { ARM::BI__builtin_arm_mve_vqshlq_u16, 20152, 20113},
  { ARM::BI__builtin_arm_mve_vqshlq_u32, 20163, 20113},
  { ARM::BI__builtin_arm_mve_vqshlq_u8, 20174, 20113},
  { ARM::BI__builtin_arm_mve_vqshluq_m_n_s16, 20194, 20184},
  { ARM::BI__builtin_arm_mve_vqshluq_m_n_s32, 20210, 20184},
  { ARM::BI__builtin_arm_mve_vqshluq_m_n_s8, 20226, 20184},
  { ARM::BI__builtin_arm_mve_vqshluq_n_s16, 20249, 20241},
  { ARM::BI__builtin_arm_mve_vqshluq_n_s32, 20263, 20241},
  { ARM::BI__builtin_arm_mve_vqshluq_n_s8, 20277, 20241},
  { ARM::BI__builtin_arm_mve_vqshrnbq_m_n_s16, 20301, 20290},
  { ARM::BI__builtin_arm_mve_vqshrnbq_m_n_s32, 20318, 20290},
  { ARM::BI__builtin_arm_mve_vqshrnbq_m_n_u16, 20335, 20290},
  { ARM::BI__builtin_arm_mve_vqshrnbq_m_n_u32, 20352, 20290},
  { ARM::BI__builtin_arm_mve_vqshrnbq_n_s16, 20378, 20369},
  { ARM::BI__builtin_arm_mve_vqshrnbq_n_s32, 20393, 20369},
  { ARM::BI__builtin_arm_mve_vqshrnbq_n_u16, 20408, 20369},
  { ARM::BI__builtin_arm_mve_vqshrnbq_n_u32, 20423, 20369},
  { ARM::BI__builtin_arm_mve_vqshrntq_m_n_s16, 20449, 20438},
  { ARM::BI__builtin_arm_mve_vqshrntq_m_n_s32, 20466, 20438},
  { ARM::BI__builtin_arm_mve_vqshrntq_m_n_u16, 20483, 20438},
  { ARM::BI__builtin_arm_mve_vqshrntq_m_n_u32, 20500, 20438},
  { ARM::BI__builtin_arm_mve_vqshrntq_n_s16, 20526, 20517},
  { ARM::BI__builtin_arm_mve_vqshrntq_n_s32, 20541, 20517},
  { ARM::BI__builtin_arm_mve_vqshrntq_n_u16, 20556, 20517},
  { ARM::BI__builtin_arm_mve_vqshrntq_n_u32, 20571, 20517},
  { ARM::BI__builtin_arm_mve_vqshrunbq_m_n_s16, 20598, 20586},
  { ARM::BI__builtin_arm_mve_vqshrunbq_m_n_s32, 20616, 20586},
  { ARM::BI__builtin_arm_mve_vqshrunbq_n_s16, 20644, 20634},
  { ARM::BI__builtin_arm_mve_vqshrunbq_n_s32, 20660, 20634},
  { ARM::BI__builtin_arm_mve_vqshruntq_m_n_s16, 20688, 20676},
  { ARM::BI__builtin_arm_mve_vqshruntq_m_n_s32, 20706, 20676},
  { ARM::BI__builtin_arm_mve_vqshruntq_n_s16, 20734, 20724},
  { ARM::BI__builtin_arm_mve_vqshruntq_n_s32, 20750, 20724},
  { ARM::BI__builtin_arm_mve_vqsubq_m_s16, 20775, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_m_s32, 20788, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_m_s8, 20801, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_m_u16, 20813, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_m_u32, 20826, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_m_u8, 20839, 20766},
  { ARM::BI__builtin_arm_mve_vqsubq_s16, 20858, 20851},
  { ARM::BI__builtin_arm_mve_vqsubq_s32, 20869, 20851},
  { ARM::BI__builtin_arm_mve_vqsubq_s8, 20880, 20851},
  { ARM::BI__builtin_arm_mve_vqsubq_u16, 20890, 20851},
  { ARM::BI__builtin_arm_mve_vqsubq_u32, 20901, 20851},
  { ARM::BI__builtin_arm_mve_vqsubq_u8, 20912, 20851},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_f32, 20940, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_s16, 20962, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_s32, 20984, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_s64, 21006, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_s8, 21028, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_u16, 21049, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_u32, 21071, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_u64, 21093, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f16_u8, 21115, 20922},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_f16, 21154, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_s16, 21176, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_s32, 21198, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_s64, 21220, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_s8, 21242, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_u16, 21263, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_u32, 21285, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_u64, 21307, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_f32_u8, 21329, 21136},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_f16, 21368, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_f32, 21390, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_s32, 21412, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_s64, 21434, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_s8, 21456, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_u16, 21477, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_u32, 21499, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_u64, 21521, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s16_u8, 21543, 21350},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_f16, 21582, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_f32, 21604, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_s16, 21626, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_s64, 21648, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_s8, 21670, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_u16, 21691, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_u32, 21713, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_u64, 21735, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s32_u8, 21757, 21564},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_f16, 21796, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_f32, 21818, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_s16, 21840, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_s32, 21862, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_s8, 21884, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_u16, 21905, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_u32, 21927, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_u64, 21949, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s64_u8, 21971, 21778},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_f16, 22009, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_f32, 22030, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_s16, 22051, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_s32, 22072, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_s64, 22093, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_u16, 22114, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_u32, 22135, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_u64, 22156, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_s8_u8, 22177, 21992},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_f16, 22215, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_f32, 22237, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_s16, 22259, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_s32, 22281, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_s64, 22303, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_s8, 22325, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_u32, 22346, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_u64, 22368, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u16_u8, 22390, 22197},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_f16, 22429, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_f32, 22451, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_s16, 22473, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_s32, 22495, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_s64, 22517, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_s8, 22539, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_u16, 22560, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_u64, 22582, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u32_u8, 22604, 22411},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_f16, 22643, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_f32, 22665, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_s16, 22687, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_s32, 22709, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_s64, 22731, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_s8, 22753, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_u16, 22774, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_u32, 22796, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u64_u8, 22818, 22625},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_f16, 22856, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_f32, 22877, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_s16, 22898, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_s32, 22919, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_s64, 22940, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_s8, 22961, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_u16, 22981, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_u32, 23002, 22839},
  { ARM::BI__builtin_arm_mve_vreinterpretq_u8_u64, 23023, 22839},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_s16, 23054, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_s32, 23068, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_s8, 23082, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_u16, 23095, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_u32, 23109, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_m_u8, 23123, 23044},
  { ARM::BI__builtin_arm_mve_vrhaddq_s16, 23144, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_s32, 23156, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_s8, 23168, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_u16, 23179, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_u32, 23191, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_u8, 23203, 23136},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_s16, 23224, 23214},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_s32, 23238, 23214},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_s8, 23252, 23214},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_u16, 23265, 23214},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_u32, 23279, 23214},
  { ARM::BI__builtin_arm_mve_vrhaddq_x_u8, 23293, 23214},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaq_p_s32, 23321, 23306},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaq_p_u32, 23340, 23306},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaq_s32, 23372, 23359},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaq_u32, 23389, 23359},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaxq_p_s32, 23422, 23406},
  { ARM::BI__builtin_arm_mve_vrmlaldavhaxq_s32, 23456, 23442},
  { ARM::BI__builtin_arm_mve_vrmlaldavhq_p_s32, 23488, 23474},
  { ARM::BI__builtin_arm_mve_vrmlaldavhq_p_u32, 23506, 23474},
  { ARM::BI__builtin_arm_mve_vrmlaldavhq_s32, 23536, 23524},
  { ARM::BI__builtin_arm_mve_vrmlaldavhq_u32, 23552, 23524},
  { ARM::BI__builtin_arm_mve_vrmlaldavhxq_p_s32, 23583, 23568},
  { ARM::BI__builtin_arm_mve_vrmlaldavhxq_s32, 23615, 23602},
  { ARM::BI__builtin_arm_mve_vrmlsldavhaq_p_s32, 23647, 23632},
  { ARM::BI__builtin_arm_mve_vrmlsldavhaq_s32, 23679, 23666},
  { ARM::BI__builtin_arm_mve_vrmlsldavhaxq_p_s32, 23712, 23696},
  { ARM::BI__builtin_arm_mve_vrmlsldavhaxq_s32, 23746, 23732},
  { ARM::BI__builtin_arm_mve_vrmlsldavhq_p_s32, 23778, 23764},
  { ARM::BI__builtin_arm_mve_vrmlsldavhq_s32, 23808, 23796},
  { ARM::BI__builtin_arm_mve_vrmlsldavhxq_p_s32, 23839, 23824},
  { ARM::BI__builtin_arm_mve_vrmlsldavhxq_s32, 23871, 23858},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_s16, 23898, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_s32, 23912, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_s8, 23926, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_u16, 23939, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_u32, 23953, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_m_u8, 23967, 23888},
  { ARM::BI__builtin_arm_mve_vrmulhq_s16, 23988, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_s32, 24000, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_s8, 24012, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_u16, 24023, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_u32, 24035, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_u8, 24047, 23980},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_s16, 24068, 24058},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_s32, 24082, 24058},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_s8, 24096, 24058},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_u16, 24109, 24058},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_u32, 24123, 24058},
  { ARM::BI__builtin_arm_mve_vrmulhq_x_u8, 24137, 24058},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_s16, 24161, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_s32, 24176, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_s8, 24191, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_u16, 24205, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_u32, 24220, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_n_u8, 24235, 24150},
  { ARM::BI__builtin_arm_mve_vrshlq_m_s16, 24258, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_m_s32, 24271, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_m_s8, 24284, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_m_u16, 24296, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_m_u32, 24309, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_m_u8, 24322, 24249},
  { ARM::BI__builtin_arm_mve_vrshlq_n_s16, 24341, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_n_s32, 24354, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_n_s8, 24367, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_n_u16, 24379, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_n_u32, 24392, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_n_u8, 24405, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_s16, 24417, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_s32, 24428, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_s8, 24439, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_u16, 24449, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_u32, 24460, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_u8, 24471, 24334},
  { ARM::BI__builtin_arm_mve_vrshlq_x_s16, 24490, 24481},
  { ARM::BI__builtin_arm_mve_vrshlq_x_s32, 24503, 24481},
  { ARM::BI__builtin_arm_mve_vrshlq_x_s8, 24516, 24481},
  { ARM::BI__builtin_arm_mve_vrshlq_x_u16, 24528, 24481},
  { ARM::BI__builtin_arm_mve_vrshlq_x_u32, 24541, 24481},
  { ARM::BI__builtin_arm_mve_vrshlq_x_u8, 24554, 24481},
  { ARM::BI__builtin_arm_mve_vrshrnbq_m_n_s16, 24577, 24566},
  { ARM::BI__builtin_arm_mve_vrshrnbq_m_n_s32, 24594, 24566},
  { ARM::BI__builtin_arm_mve_vrshrnbq_m_n_u16, 24611, 24566},
  { ARM::BI__builtin_arm_mve_vrshrnbq_m_n_u32, 24628, 24566},
  { ARM::BI__builtin_arm_mve_vrshrnbq_n_s16, 24654, 24645},
  { ARM::BI__builtin_arm_mve_vrshrnbq_n_s32, 24669, 24645},
  { ARM::BI__builtin_arm_mve_vrshrnbq_n_u16, 24684, 24645},
  { ARM::BI__builtin_arm_mve_vrshrnbq_n_u32, 24699, 24645},
  { ARM::BI__builtin_arm_mve_vrshrntq_m_n_s16, 24725, 24714},
  { ARM::BI__builtin_arm_mve_vrshrntq_m_n_s32, 24742, 24714},
  { ARM::BI__builtin_arm_mve_vrshrntq_m_n_u16, 24759, 24714},
  { ARM::BI__builtin_arm_mve_vrshrntq_m_n_u32, 24776, 24714},
  { ARM::BI__builtin_arm_mve_vrshrntq_n_s16, 24802, 24793},
  { ARM::BI__builtin_arm_mve_vrshrntq_n_s32, 24817, 24793},
  { ARM::BI__builtin_arm_mve_vrshrntq_n_u16, 24832, 24793},
  { ARM::BI__builtin_arm_mve_vrshrntq_n_u32, 24847, 24793},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_s16, 24871, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_s32, 24886, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_s8, 24901, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_u16, 24915, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_u32, 24930, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_m_n_u8, 24945, 24862},
  { ARM::BI__builtin_arm_mve_vrshrq_n_s16, 24966, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_n_s32, 24979, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_n_s8, 24992, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_n_u16, 25004, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_n_u32, 25017, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_n_u8, 25030, 24959},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_s16, 25051, 25042},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_s32, 25066, 25042},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_s8, 25081, 25042},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_u16, 25095, 25042},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_u32, 25110, 25042},
  { ARM::BI__builtin_arm_mve_vrshrq_x_n_u8, 25125, 25042},
  { ARM::BI__builtin_arm_mve_vsetq_lane_f16, 25150, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_f32, 25165, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_s16, 25180, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_s32, 25195, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_s64, 25210, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_s8, 25225, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_u16, 25239, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_u32, 25254, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_u64, 25269, 25139},
  { ARM::BI__builtin_arm_mve_vsetq_lane_u8, 25284, 25139},
  { ARM::BI__builtin_arm_mve_vshllbq_m_n_s16, 25308, 25298},
  { ARM::BI__builtin_arm_mve_vshllbq_m_n_s8, 25324, 25298},
  { ARM::BI__builtin_arm_mve_vshllbq_m_n_u16, 25339, 25298},
  { ARM::BI__builtin_arm_mve_vshllbq_m_n_u8, 25355, 25298},
  { ARM::BI__builtin_arm_mve_vshllbq_n_s16, 25378, 25370},
  { ARM::BI__builtin_arm_mve_vshllbq_n_s8, 25392, 25370},
  { ARM::BI__builtin_arm_mve_vshllbq_n_u16, 25405, 25370},
  { ARM::BI__builtin_arm_mve_vshllbq_n_u8, 25419, 25370},
  { ARM::BI__builtin_arm_mve_vshllbq_x_n_s16, 25442, 25432},
  { ARM::BI__builtin_arm_mve_vshllbq_x_n_s8, 25458, 25432},
  { ARM::BI__builtin_arm_mve_vshllbq_x_n_u16, 25473, 25432},
  { ARM::BI__builtin_arm_mve_vshllbq_x_n_u8, 25489, 25432},
  { ARM::BI__builtin_arm_mve_vshlltq_m_n_s16, 25514, 25504},
  { ARM::BI__builtin_arm_mve_vshlltq_m_n_s8, 25530, 25504},
  { ARM::BI__builtin_arm_mve_vshlltq_m_n_u16, 25545, 25504},
  { ARM::BI__builtin_arm_mve_vshlltq_m_n_u8, 25561, 25504},
  { ARM::BI__builtin_arm_mve_vshlltq_n_s16, 25584, 25576},
  { ARM::BI__builtin_arm_mve_vshlltq_n_s8, 25598, 25576},
  { ARM::BI__builtin_arm_mve_vshlltq_n_u16, 25611, 25576},
  { ARM::BI__builtin_arm_mve_vshlltq_n_u8, 25625, 25576},
  { ARM::BI__builtin_arm_mve_vshlltq_x_n_s16, 25648, 25638},
  { ARM::BI__builtin_arm_mve_vshlltq_x_n_s8, 25664, 25638},
  { ARM::BI__builtin_arm_mve_vshlltq_x_n_u16, 25679, 25638},
  { ARM::BI__builtin_arm_mve_vshlltq_x_n_u8, 25695, 25638},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_s16, 25720, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_s32, 25734, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_s8, 25748, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_u16, 25761, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_u32, 25775, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_n_u8, 25789, 25710},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_s16, 25812, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_s32, 25826, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_s8, 25840, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_u16, 25853, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_u32, 25867, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_r_u8, 25881, 25802},
  { ARM::BI__builtin_arm_mve_vshlq_m_s16, 25902, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_m_s32, 25914, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_m_s8, 25926, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_m_u16, 25937, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_m_u32, 25949, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_m_u8, 25961, 25894},
  { ARM::BI__builtin_arm_mve_vshlq_n_s16, 25980, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_n_s32, 25992, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_n_s8, 26004, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_n_u16, 26015, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_n_u32, 26027, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_n_u8, 26039, 25972},
  { ARM::BI__builtin_arm_mve_vshlq_r_s16, 26058, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_r_s32, 26070, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_r_s8, 26082, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_r_u16, 26093, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_r_u32, 26105, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_r_u8, 26117, 26050},
  { ARM::BI__builtin_arm_mve_vshlq_s16, 26134, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_s32, 26144, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_s8, 26154, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_u16, 26163, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_u32, 26173, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_u8, 26183, 26128},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_s16, 26202, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_s32, 26216, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_s8, 26230, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_u16, 26243, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_u32, 26257, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_n_u8, 26271, 26192},
  { ARM::BI__builtin_arm_mve_vshlq_x_s16, 26292, 26284},
  { ARM::BI__builtin_arm_mve_vshlq_x_s32, 26304, 26284},
  { ARM::BI__builtin_arm_mve_vshlq_x_s8, 26316, 26284},
  { ARM::BI__builtin_arm_mve_vshlq_x_u16, 26327, 26284},
  { ARM::BI__builtin_arm_mve_vshlq_x_u32, 26339, 26284},
  { ARM::BI__builtin_arm_mve_vshlq_x_u8, 26351, 26284},
  { ARM::BI__builtin_arm_mve_vshrnbq_m_n_s16, 26372, 26362},
  { ARM::BI__builtin_arm_mve_vshrnbq_m_n_s32, 26388, 26362},
  { ARM::BI__builtin_arm_mve_vshrnbq_m_n_u16, 26404, 26362},
  { ARM::BI__builtin_arm_mve_vshrnbq_m_n_u32, 26420, 26362},
  { ARM::BI__builtin_arm_mve_vshrnbq_n_s16, 26444, 26436},
  { ARM::BI__builtin_arm_mve_vshrnbq_n_s32, 26458, 26436},
  { ARM::BI__builtin_arm_mve_vshrnbq_n_u16, 26472, 26436},
  { ARM::BI__builtin_arm_mve_vshrnbq_n_u32, 26486, 26436},
  { ARM::BI__builtin_arm_mve_vshrntq_m_n_s16, 26510, 26500},
  { ARM::BI__builtin_arm_mve_vshrntq_m_n_s32, 26526, 26500},
  { ARM::BI__builtin_arm_mve_vshrntq_m_n_u16, 26542, 26500},
  { ARM::BI__builtin_arm_mve_vshrntq_m_n_u32, 26558, 26500},
  { ARM::BI__builtin_arm_mve_vshrntq_n_s16, 26582, 26574},
  { ARM::BI__builtin_arm_mve_vshrntq_n_s32, 26596, 26574},
  { ARM::BI__builtin_arm_mve_vshrntq_n_u16, 26610, 26574},
  { ARM::BI__builtin_arm_mve_vshrntq_n_u32, 26624, 26574},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_s16, 26646, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_s32, 26660, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_s8, 26674, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_u16, 26687, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_u32, 26701, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_m_n_u8, 26715, 26638},
  { ARM::BI__builtin_arm_mve_vshrq_n_s16, 26734, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_n_s32, 26746, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_n_s8, 26758, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_n_u16, 26769, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_n_u32, 26781, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_n_u8, 26793, 26728},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_s16, 26812, 26804},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_s32, 26826, 26804},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_s8, 26840, 26804},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_u16, 26853, 26804},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_u32, 26867, 26804},
  { ARM::BI__builtin_arm_mve_vshrq_x_n_u8, 26881, 26804},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_s16, 26902, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_s32, 26916, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_s8, 26930, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_u16, 26943, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_u32, 26957, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_m_n_u8, 26971, 26894},
  { ARM::BI__builtin_arm_mve_vsliq_n_s16, 26990, 26984},
  { ARM::BI__builtin_arm_mve_vsliq_n_s32, 27002, 26984},
  { ARM::BI__builtin_arm_mve_vsliq_n_s8, 27014, 26984},
  { ARM::BI__builtin_arm_mve_vsliq_n_u16, 27025, 26984},
  { ARM::BI__builtin_arm_mve_vsliq_n_u32, 27037, 26984},
  { ARM::BI__builtin_arm_mve_vsliq_n_u8, 27049, 26984},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_s16, 27068, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_s32, 27082, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_s8, 27096, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_u16, 27109, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_u32, 27123, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_m_n_u8, 27137, 27060},
  { ARM::BI__builtin_arm_mve_vsriq_n_s16, 27156, 27150},
  { ARM::BI__builtin_arm_mve_vsriq_n_s32, 27168, 27150},
  { ARM::BI__builtin_arm_mve_vsriq_n_s8, 27180, 27150},
  { ARM::BI__builtin_arm_mve_vsriq_n_u16, 27191, 27150},
  { ARM::BI__builtin_arm_mve_vsriq_n_u32, 27203, 27150},
  { ARM::BI__builtin_arm_mve_vsriq_n_u8, 27215, 27150},
  { ARM::BI__builtin_arm_mve_vst1q_f16, 27232, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_f32, 27242, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_p_f16, 27260, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_f32, 27272, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_s16, 27284, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_s32, 27296, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_s8, 27308, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_u16, 27319, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_u32, 27331, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_p_u8, 27343, 27252},
  { ARM::BI__builtin_arm_mve_vst1q_s16, 27354, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_s32, 27364, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_s8, 27374, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_u16, 27383, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_u32, 27393, 27226},
  { ARM::BI__builtin_arm_mve_vst1q_u8, 27403, 27226},
  { ARM::BI__builtin_arm_mve_vst2q_f16, 27418, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_f32, 27428, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_s16, 27438, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_s32, 27448, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_s8, 27458, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_u16, 27467, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_u32, 27477, 27412},
  { ARM::BI__builtin_arm_mve_vst2q_u8, 27487, 27412},
  { ARM::BI__builtin_arm_mve_vst4q_f16, 27502, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_f32, 27512, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_s16, 27522, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_s32, 27532, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_s8, 27542, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_u16, 27551, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_u32, 27561, 27496},
  { ARM::BI__builtin_arm_mve_vst4q_u8, 27571, 27496},
  { ARM::BI__builtin_arm_mve_vstrbq_p_s16, 27589, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_p_s32, 27602, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_p_s8, 27615, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_p_u16, 27627, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_p_u32, 27640, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_p_u8, 27653, 27580},
  { ARM::BI__builtin_arm_mve_vstrbq_s16, 27672, 27665},
  { ARM::BI__builtin_arm_mve_vstrbq_s32, 27683, 27665},
  { ARM::BI__builtin_arm_mve_vstrbq_s8, 27694, 27665},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_s16, 27728, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_s32, 27756, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_s8, 27784, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_u16, 27811, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_u32, 27839, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_p_u8, 27867, 27704},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_s16, 27916, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_s32, 27942, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_s8, 27968, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_u16, 27993, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_u32, 28019, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_scatter_offset_u8, 28045, 27894},
  { ARM::BI__builtin_arm_mve_vstrbq_u16, 28070, 27665},
  { ARM::BI__builtin_arm_mve_vstrbq_u32, 28081, 27665},
  { ARM::BI__builtin_arm_mve_vstrbq_u8, 28092, 27665},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_p_s64, 28124, 28102},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_p_u64, 28150, 28102},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_s64, 28196, 28176},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_u64, 28220, 28176},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_wb_p_s64, 28269, 28244},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_wb_p_u64, 28298, 28244},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_wb_s64, 28350, 28327},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_base_wb_u64, 28377, 28327},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_offset_p_s64, 28428, 28404},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_offset_p_u64, 28456, 28404},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_offset_s64, 28506, 28484},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_offset_u64, 28532, 28484},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_shifted_offset_p_s64, 28590, 28558},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_shifted_offset_p_u64, 28626, 28558},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_shifted_offset_s64, 28692, 28662},
  { ARM::BI__builtin_arm_mve_vstrdq_scatter_shifted_offset_u64, 28726, 28662},
  { ARM::BI__builtin_arm_mve_vstrhq_f16, 28767, 28760},
  { ARM::BI__builtin_arm_mve_vstrhq_p_f16, 28787, 28778},
  { ARM::BI__builtin_arm_mve_vstrhq_p_s16, 28800, 28778},
  { ARM::BI__builtin_arm_mve_vstrhq_p_s32, 28813, 28778},
  { ARM::BI__builtin_arm_mve_vstrhq_p_u16, 28826, 28778},
  { ARM::BI__builtin_arm_mve_vstrhq_p_u32, 28839, 28778},
  { ARM::BI__builtin_arm_mve_vstrhq_s16, 28852, 28760},
  { ARM::BI__builtin_arm_mve_vstrhq_s32, 28863, 28760},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_f16, 28896, 28874},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_p_f16, 28946, 28922},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_p_s16, 28974, 28922},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_p_s32, 29002, 28922},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_p_u16, 29030, 28922},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_p_u32, 29058, 28922},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_s16, 29086, 28874},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_s32, 29112, 28874},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_u16, 29138, 28874},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_offset_u32, 29164, 28874},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_f16, 29220, 29190},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_p_f16, 29286, 29254},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_p_s16, 29322, 29254},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_p_s32, 29358, 29254},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_p_u16, 29394, 29254},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_p_u32, 29430, 29254},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_s16, 29466, 29190},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_s32, 29500, 29190},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_u16, 29534, 29190},
  { ARM::BI__builtin_arm_mve_vstrhq_scatter_shifted_offset_u32, 29568, 29190},
  { ARM::BI__builtin_arm_mve_vstrhq_u16, 29602, 28760},
  { ARM::BI__builtin_arm_mve_vstrhq_u32, 29613, 28760},
  { ARM::BI__builtin_arm_mve_vstrwq_f32, 29631, 29624},
  { ARM::BI__builtin_arm_mve_vstrwq_p_f32, 29651, 29642},
  { ARM::BI__builtin_arm_mve_vstrwq_p_s32, 29664, 29642},
  { ARM::BI__builtin_arm_mve_vstrwq_p_u32, 29677, 29642},
  { ARM::BI__builtin_arm_mve_vstrwq_s32, 29690, 29624},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_f32, 29721, 29701},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_p_f32, 29767, 29745},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_p_s32, 29793, 29745},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_p_u32, 29819, 29745},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_s32, 29845, 29701},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_u32, 29869, 29701},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_f32, 29916, 29893},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_p_f32, 29968, 29943},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_p_s32, 29997, 29943},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_p_u32, 30026, 29943},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_s32, 30055, 29893},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_base_wb_u32, 30082, 29893},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_f32, 30131, 30109},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_p_f32, 30181, 30157},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_p_s32, 30209, 30157},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_p_u32, 30237, 30157},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_s32, 30265, 30109},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_offset_u32, 30291, 30109},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_f32, 30347, 30317},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_p_f32, 30413, 30381},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_p_s32, 30449, 30381},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_p_u32, 30485, 30381},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_s32, 30521, 30317},
  { ARM::BI__builtin_arm_mve_vstrwq_scatter_shifted_offset_u32, 30555, 30317},
  { ARM::BI__builtin_arm_mve_vstrwq_u32, 30589, 29624},
  { ARM::BI__builtin_arm_mve_vsubq_f16, 30606, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_f32, 30616, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_m_f16, 30634, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_f32, 30646, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_s16, 30658, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_s32, 30670, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_s8, 30682, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_u16, 30693, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_u32, 30705, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_m_u8, 30717, 30626},
  { ARM::BI__builtin_arm_mve_vsubq_s16, 30728, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_s32, 30738, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_s8, 30748, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_u16, 30757, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_u32, 30767, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_u8, 30777, 30600},
  { ARM::BI__builtin_arm_mve_vsubq_x_f16, 30794, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_f32, 30806, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_s16, 30818, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_s32, 30830, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_s8, 30842, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_u16, 30853, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_u32, 30865, 30786},
  { ARM::BI__builtin_arm_mve_vsubq_x_u8, 30877, 30786},
  { ARM::BI__builtin_arm_mve_vuninitializedq_f16, 30888, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_f32, 30908, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_f16, 30944, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_f32, 30976, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_s16, 31008, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_s32, 31040, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_s64, 31072, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_s8, 31104, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_u16, 31135, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_u32, 31167, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_u64, 31199, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_polymorphic_u8, 31231, 30928},
  { ARM::BI__builtin_arm_mve_vuninitializedq_s16, 31262, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_s32, 31282, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_s64, 31302, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_s8, 31322, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_u16, 31341, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_u32, 31361, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_u64, 31381, -1},
  { ARM::BI__builtin_arm_mve_vuninitializedq_u8, 31401, -1},
};

static const char IntrinNames[] = {
    "asrl\000lsll\000sqrshr\000sqrshrl\000sqrshrl_sat48\000sqshl\000sqshll\000"
    "srshr\000srshrl\000uqrshl\000uqrshll\000uqrshll_sat48\000uqshl\000uqshl"
    "l\000urshr\000urshrl\000vabavq_p\000vabavq_p_s16\000vabavq_p_s32\000vab"
    "avq_p_s8\000vabavq_p_u16\000vabavq_p_u32\000vabavq_p_u8\000vabavq\000va"
    "bavq_s16\000vabavq_s32\000vabavq_s8\000vabavq_u16\000vabavq_u32\000vaba"
    "vq_u8\000vabdq\000vabdq_f16\000vabdq_f32\000vabdq_m\000vabdq_m_f16\000v"
    "abdq_m_f32\000vabdq_m_s16\000vabdq_m_s32\000vabdq_m_s8\000vabdq_m_u16\000"
    "vabdq_m_u32\000vabdq_m_u8\000vabdq_s16\000vabdq_s32\000vabdq_s8\000vabd"
    "q_u16\000vabdq_u32\000vabdq_u8\000vabdq_x\000vabdq_x_f16\000vabdq_x_f32"
    "\000vabdq_x_s16\000vabdq_x_s32\000vabdq_x_s8\000vabdq_x_u16\000vabdq_x_"
    "u32\000vabdq_x_u8\000vadciq_m\000vadciq_m_s32\000vadciq_m_u32\000vadciq"
    "\000vadciq_s32\000vadciq_u32\000vadcq_m\000vadcq_m_s32\000vadcq_m_u32\000"
    "vadcq\000vadcq_s32\000vadcq_u32\000vaddq\000vaddq_f16\000vaddq_f32\000v"
    "addq_m\000vaddq_m_f16\000vaddq_m_f32\000vaddq_m_s16\000vaddq_m_s32\000v"
    "addq_m_s8\000vaddq_m_u16\000vaddq_m_u32\000vaddq_m_u8\000vaddq_s16\000v"
    "addq_s32\000vaddq_s8\000vaddq_u16\000vaddq_u32\000vaddq_u8\000vaddq_x\000"
    "vaddq_x_f16\000vaddq_x_f32\000vaddq_x_s16\000vaddq_x_s32\000vaddq_x_s8\000"
    "vaddq_x_u16\000vaddq_x_u32\000vaddq_x_u8\000vandq\000vandq_f16\000vandq"
    "_f32\000vandq_m\000vandq_m_f16\000vandq_m_f32\000vandq_m_s16\000vandq_m"
    "_s32\000vandq_m_s8\000vandq_m_u16\000vandq_m_u32\000vandq_m_u8\000vandq"
    "_s16\000vandq_s32\000vandq_s8\000vandq_u16\000vandq_u32\000vandq_u8\000"
    "vandq_x\000vandq_x_f16\000vandq_x_f32\000vandq_x_s16\000vandq_x_s32\000"
    "vandq_x_s8\000vandq_x_u16\000vandq_x_u32\000vandq_x_u8\000vbicq\000vbic"
    "q_f16\000vbicq_f32\000vbicq_m\000vbicq_m_f16\000vbicq_m_f32\000vbicq_m_"
    "n\000vbicq_m_n_s16\000vbicq_m_n_s32\000vbicq_m_n_u16\000vbicq_m_n_u32\000"
    "vbicq_m_s16\000vbicq_m_s32\000vbicq_m_s8\000vbicq_m_u16\000vbicq_m_u32\000"
    "vbicq_m_u8\000vbicq_n_s16\000vbicq_n_s32\000vbicq_n_u16\000vbicq_n_u32\000"
    "vbicq_s16\000vbicq_s32\000vbicq_s8\000vbicq_u16\000vbicq_u32\000vbicq_u"
    "8\000vbicq_x\000vbicq_x_f16\000vbicq_x_f32\000vbicq_x_s16\000vbicq_x_s3"
    "2\000vbicq_x_s8\000vbicq_x_u16\000vbicq_x_u32\000vbicq_x_u8\000vcaddq_r"
    "ot270\000vcaddq_rot270_f16\000vcaddq_rot270_f32\000vcaddq_rot270_m\000v"
    "caddq_rot270_m_f16\000vcaddq_rot270_m_f32\000vcaddq_rot270_m_s16\000vca"
    "ddq_rot270_m_s32\000vcaddq_rot270_m_s8\000vcaddq_rot270_m_u16\000vcaddq"
    "_rot270_m_u32\000vcaddq_rot270_m_u8\000vcaddq_rot270_s16\000vcaddq_rot2"
    "70_s32\000vcaddq_rot270_s8\000vcaddq_rot270_u16\000vcaddq_rot270_u32\000"
    "vcaddq_rot270_u8\000vcaddq_rot270_x\000vcaddq_rot270_x_f16\000vcaddq_ro"
    "t270_x_f32\000vcaddq_rot270_x_s16\000vcaddq_rot270_x_s32\000vcaddq_rot2"
    "70_x_s8\000vcaddq_rot270_x_u16\000vcaddq_rot270_x_u32\000vcaddq_rot270_"
    "x_u8\000vcaddq_rot90\000vcaddq_rot90_f16\000vcaddq_rot90_f32\000vcaddq_"
    "rot90_m\000vcaddq_rot90_m_f16\000vcaddq_rot90_m_f32\000vcaddq_rot90_m_s"
    "16\000vcaddq_rot90_m_s32\000vcaddq_rot90_m_s8\000vcaddq_rot90_m_u16\000"
    "vcaddq_rot90_m_u32\000vcaddq_rot90_m_u8\000vcaddq_rot90_s16\000vcaddq_r"
    "ot90_s32\000vcaddq_rot90_s8\000vcaddq_rot90_u16\000vcaddq_rot90_u32\000"
    "vcaddq_rot90_u8\000vcaddq_rot90_x\000vcaddq_rot90_x_f16\000vcaddq_rot90"
    "_x_f32\000vcaddq_rot90_x_s16\000vcaddq_rot90_x_s32\000vcaddq_rot90_x_s8"
    "\000vcaddq_rot90_x_u16\000vcaddq_rot90_x_u32\000vcaddq_rot90_x_u8\000vc"
    "mlaq\000vcmlaq_f16\000vcmlaq_f32\000vcmlaq_m\000vcmlaq_m_f16\000vcmlaq_"
    "m_f32\000vcmlaq_rot180\000vcmlaq_rot180_f16\000vcmlaq_rot180_f32\000vcm"
    "laq_rot180_m\000vcmlaq_rot180_m_f16\000vcmlaq_rot180_m_f32\000vcmlaq_ro"
    "t270\000vcmlaq_rot270_f16\000vcmlaq_rot270_f32\000vcmlaq_rot270_m\000vc"
    "mlaq_rot270_m_f16\000vcmlaq_rot270_m_f32\000vcmlaq_rot90\000vcmlaq_rot9"
    "0_f16\000vcmlaq_rot90_f32\000vcmlaq_rot90_m\000vcmlaq_rot90_m_f16\000vc"
    "mlaq_rot90_m_f32\000vcmpcsq_m\000vcmpcsq_m_n_u16\000vcmpcsq_m_n_u32\000"
    "vcmpcsq_m_n_u8\000vcmpcsq_m_u16\000vcmpcsq_m_u32\000vcmpcsq_m_u8\000vcm"
    "pcsq\000vcmpcsq_n_u16\000vcmpcsq_n_u32\000vcmpcsq_n_u8\000vcmpcsq_u16\000"
    "vcmpcsq_u32\000vcmpcsq_u8\000vcmpeqq\000vcmpeqq_f16\000vcmpeqq_f32\000v"
    "cmpeqq_m\000vcmpeqq_m_f16\000vcmpeqq_m_f32\000vcmpeqq_m_n_f16\000vcmpeq"
    "q_m_n_f32\000vcmpeqq_m_n_s16\000vcmpeqq_m_n_s32\000vcmpeqq_m_n_s8\000vc"
    "mpeqq_m_n_u16\000vcmpeqq_m_n_u32\000vcmpeqq_m_n_u8\000vcmpeqq_m_s16\000"
    "vcmpeqq_m_s32\000vcmpeqq_m_s8\000vcmpeqq_m_u16\000vcmpeqq_m_u32\000vcmp"
    "eqq_m_u8\000vcmpeqq_n_f16\000vcmpeqq_n_f32\000vcmpeqq_n_s16\000vcmpeqq_"
    "n_s32\000vcmpeqq_n_s8\000vcmpeqq_n_u16\000vcmpeqq_n_u32\000vcmpeqq_n_u8"
    "\000vcmpeqq_s16\000vcmpeqq_s32\000vcmpeqq_s8\000vcmpeqq_u16\000vcmpeqq_"
    "u32\000vcmpeqq_u8\000vcmpgeq\000vcmpgeq_f16\000vcmpgeq_f32\000vcmpgeq_m"
    "\000vcmpgeq_m_f16\000vcmpgeq_m_f32\000vcmpgeq_m_n_f16\000vcmpgeq_m_n_f3"
    "2\000vcmpgeq_m_n_s16\000vcmpgeq_m_n_s32\000vcmpgeq_m_n_s8\000vcmpgeq_m_"
    "s16\000vcmpgeq_m_s32\000vcmpgeq_m_s8\000vcmpgeq_n_f16\000vcmpgeq_n_f32\000"
    "vcmpgeq_n_s16\000vcmpgeq_n_s32\000vcmpgeq_n_s8\000vcmpgeq_s16\000vcmpge"
    "q_s32\000vcmpgeq_s8\000vcmpgtq\000vcmpgtq_f16\000vcmpgtq_f32\000vcmpgtq"
    "_m\000vcmpgtq_m_f16\000vcmpgtq_m_f32\000vcmpgtq_m_n_f16\000vcmpgtq_m_n_"
    "f32\000vcmpgtq_m_n_s16\000vcmpgtq_m_n_s32\000vcmpgtq_m_n_s8\000vcmpgtq_"
    "m_s16\000vcmpgtq_m_s32\000vcmpgtq_m_s8\000vcmpgtq_n_f16\000vcmpgtq_n_f3"
    "2\000vcmpgtq_n_s16\000vcmpgtq_n_s32\000vcmpgtq_n_s8\000vcmpgtq_s16\000v"
    "cmpgtq_s32\000vcmpgtq_s8\000vcmphiq_m\000vcmphiq_m_n_u16\000vcmphiq_m_n"
    "_u32\000vcmphiq_m_n_u8\000vcmphiq_m_u16\000vcmphiq_m_u32\000vcmphiq_m_u"
    "8\000vcmphiq\000vcmphiq_n_u16\000vcmphiq_n_u32\000vcmphiq_n_u8\000vcmph"
    "iq_u16\000vcmphiq_u32\000vcmphiq_u8\000vcmpleq\000vcmpleq_f16\000vcmple"
    "q_f32\000vcmpleq_m\000vcmpleq_m_f16\000vcmpleq_m_f32\000vcmpleq_m_n_f16"
    "\000vcmpleq_m_n_f32\000vcmpleq_m_n_s16\000vcmpleq_m_n_s32\000vcmpleq_m_"
    "n_s8\000vcmpleq_m_s16\000vcmpleq_m_s32\000vcmpleq_m_s8\000vcmpleq_n_f16"
    "\000vcmpleq_n_f32\000vcmpleq_n_s16\000vcmpleq_n_s32\000vcmpleq_n_s8\000"
    "vcmpleq_s16\000vcmpleq_s32\000vcmpleq_s8\000vcmpltq\000vcmpltq_f16\000v"
    "cmpltq_f32\000vcmpltq_m\000vcmpltq_m_f16\000vcmpltq_m_f32\000vcmpltq_m_"
    "n_f16\000vcmpltq_m_n_f32\000vcmpltq_m_n_s16\000vcmpltq_m_n_s32\000vcmpl"
    "tq_m_n_s8\000vcmpltq_m_s16\000vcmpltq_m_s32\000vcmpltq_m_s8\000vcmpltq_"
    "n_f16\000vcmpltq_n_f32\000vcmpltq_n_s16\000vcmpltq_n_s32\000vcmpltq_n_s"
    "8\000vcmpltq_s16\000vcmpltq_s32\000vcmpltq_s8\000vcmpneq\000vcmpneq_f16"
    "\000vcmpneq_f32\000vcmpneq_m\000vcmpneq_m_f16\000vcmpneq_m_f32\000vcmpn"
    "eq_m_n_f16\000vcmpneq_m_n_f32\000vcmpneq_m_n_s16\000vcmpneq_m_n_s32\000"
    "vcmpneq_m_n_s8\000vcmpneq_m_n_u16\000vcmpneq_m_n_u32\000vcmpneq_m_n_u8\000"
    "vcmpneq_m_s16\000vcmpneq_m_s32\000vcmpneq_m_s8\000vcmpneq_m_u16\000vcmp"
    "neq_m_u32\000vcmpneq_m_u8\000vcmpneq_n_f16\000vcmpneq_n_f32\000vcmpneq_"
    "n_s16\000vcmpneq_n_s32\000vcmpneq_n_s8\000vcmpneq_n_u16\000vcmpneq_n_u3"
    "2\000vcmpneq_n_u8\000vcmpneq_s16\000vcmpneq_s32\000vcmpneq_s8\000vcmpne"
    "q_u16\000vcmpneq_u32\000vcmpneq_u8\000vcmulq\000vcmulq_f16\000vcmulq_f3"
    "2\000vcmulq_m\000vcmulq_m_f16\000vcmulq_m_f32\000vcmulq_rot180\000vcmul"
    "q_rot180_f16\000vcmulq_rot180_f32\000vcmulq_rot180_m\000vcmulq_rot180_m"
    "_f16\000vcmulq_rot180_m_f32\000vcmulq_rot180_x\000vcmulq_rot180_x_f16\000"
    "vcmulq_rot180_x_f32\000vcmulq_rot270\000vcmulq_rot270_f16\000vcmulq_rot"
    "270_f32\000vcmulq_rot270_m\000vcmulq_rot270_m_f16\000vcmulq_rot270_m_f3"
    "2\000vcmulq_rot270_x\000vcmulq_rot270_x_f16\000vcmulq_rot270_x_f32\000v"
    "cmulq_rot90\000vcmulq_rot90_f16\000vcmulq_rot90_f32\000vcmulq_rot90_m\000"
    "vcmulq_rot90_m_f16\000vcmulq_rot90_m_f32\000vcmulq_rot90_x\000vcmulq_ro"
    "t90_x_f16\000vcmulq_rot90_x_f32\000vcmulq_x\000vcmulq_x_f16\000vcmulq_x"
    "_f32\000vcreateq_f16\000vcreateq_f32\000vcreateq_s16\000vcreateq_s32\000"
    "vcreateq_s64\000vcreateq_s8\000vcreateq_u16\000vcreateq_u32\000vcreateq"
    "_u64\000vcreateq_u8\000vctp16q\000vctp16q_m\000vctp32q\000vctp32q_m\000"
    "vctp64q\000vctp64q_m\000vctp8q\000vctp8q_m\000vcvtbq_f16_f32\000vcvtbq_"
    "m_f16_f32\000vcvtq_m_n\000vcvtq_m_n_f16_s16\000vcvtq_m_n_f16_u16\000vcv"
    "tq_m_n_f32_s32\000vcvtq_m_n_f32_u32\000vcvtq_m_n_s16_f16\000vcvtq_m_n_s"
    "32_f32\000vcvtq_m_n_u16_f16\000vcvtq_m_n_u32_f32\000vcvtq_n\000vcvtq_n_"
    "f16_s16\000vcvtq_n_f16_u16\000vcvtq_n_f32_s32\000vcvtq_n_f32_u32\000vcv"
    "tq_n_s16_f16\000vcvtq_n_s32_f32\000vcvtq_n_u16_f16\000vcvtq_n_u32_f32\000"
    "vcvtq_x_n\000vcvtq_x_n_f16_s16\000vcvtq_x_n_f16_u16\000vcvtq_x_n_f32_s3"
    "2\000vcvtq_x_n_f32_u32\000vcvtq_x_n_s16_f16\000vcvtq_x_n_s32_f32\000vcv"
    "tq_x_n_u16_f16\000vcvtq_x_n_u32_f32\000vcvttq_f16_f32\000vcvttq_m_f16_f"
    "32\000vddupq_m\000vddupq_m_n_u16\000vddupq_m_n_u32\000vddupq_m_n_u8\000"
    "vddupq_m_wb_u16\000vddupq_m_wb_u32\000vddupq_m_wb_u8\000vddupq_u16\000v"
    "ddupq_n_u16\000vddupq_u32\000vddupq_n_u32\000vddupq_u8\000vddupq_n_u8\000"
    "vddupq_wb_u16\000vddupq_wb_u32\000vddupq_wb_u8\000vddupq_x_u16\000vddup"
    "q_x_n_u16\000vddupq_x_u32\000vddupq_x_n_u32\000vddupq_x_u8\000vddupq_x_"
    "n_u8\000vddupq_x_wb_u16\000vddupq_x_wb_u32\000vddupq_x_wb_u8\000vdupq_m"
    "\000vdupq_m_n_f16\000vdupq_m_n_f32\000vdupq_m_n_s16\000vdupq_m_n_s32\000"
    "vdupq_m_n_s8\000vdupq_m_n_u16\000vdupq_m_n_u32\000vdupq_m_n_u8\000vdupq"
    "_n_f16\000vdupq_n_f32\000vdupq_n_s16\000vdupq_n_s32\000vdupq_n_s8\000vd"
    "upq_n_u16\000vdupq_n_u32\000vdupq_n_u8\000vdupq_x_n_f16\000vdupq_x_n_f3"
    "2\000vdupq_x_n_s16\000vdupq_x_n_s32\000vdupq_x_n_s8\000vdupq_x_n_u16\000"
    "vdupq_x_n_u32\000vdupq_x_n_u8\000vdwdupq_m\000vdwdupq_m_n_u16\000vdwdup"
    "q_m_n_u32\000vdwdupq_m_n_u8\000vdwdupq_m_wb_u16\000vdwdupq_m_wb_u32\000"
    "vdwdupq_m_wb_u8\000vdwdupq_u16\000vdwdupq_n_u16\000vdwdupq_u32\000vdwdu"
    "pq_n_u32\000vdwdupq_u8\000vdwdupq_n_u8\000vdwdupq_wb_u16\000vdwdupq_wb_"
    "u32\000vdwdupq_wb_u8\000vdwdupq_x_u16\000vdwdupq_x_n_u16\000vdwdupq_x_u"
    "32\000vdwdupq_x_n_u32\000vdwdupq_x_u8\000vdwdupq_x_n_u8\000vdwdupq_x_wb"
    "_u16\000vdwdupq_x_wb_u32\000vdwdupq_x_wb_u8\000veorq\000veorq_f16\000ve"
    "orq_f32\000veorq_m\000veorq_m_f16\000veorq_m_f32\000veorq_m_s16\000veor"
    "q_m_s32\000veorq_m_s8\000veorq_m_u16\000veorq_m_u32\000veorq_m_u8\000ve"
    "orq_s16\000veorq_s32\000veorq_s8\000veorq_u16\000veorq_u32\000veorq_u8\000"
    "veorq_x\000veorq_x_f16\000veorq_x_f32\000veorq_x_s16\000veorq_x_s32\000"
    "veorq_x_s8\000veorq_x_u16\000veorq_x_u32\000veorq_x_u8\000vgetq_lane\000"
    "vgetq_lane_f16\000vgetq_lane_f32\000vgetq_lane_s16\000vgetq_lane_s32\000"
    "vgetq_lane_s64\000vgetq_lane_s8\000vgetq_lane_u16\000vgetq_lane_u32\000"
    "vgetq_lane_u64\000vgetq_lane_u8\000vhaddq_m\000vhaddq_m_s16\000vhaddq_m"
    "_s32\000vhaddq_m_s8\000vhaddq_m_u16\000vhaddq_m_u32\000vhaddq_m_u8\000v"
    "haddq\000vhaddq_s16\000vhaddq_s32\000vhaddq_s8\000vhaddq_u16\000vhaddq_"
    "u32\000vhaddq_u8\000vhaddq_x\000vhaddq_x_s16\000vhaddq_x_s32\000vhaddq_"
    "x_s8\000vhaddq_x_u16\000vhaddq_x_u32\000vhaddq_x_u8\000vhcaddq_rot270_m"
    "\000vhcaddq_rot270_m_s16\000vhcaddq_rot270_m_s32\000vhcaddq_rot270_m_s8"
    "\000vhcaddq_rot270\000vhcaddq_rot270_s16\000vhcaddq_rot270_s32\000vhcad"
    "dq_rot270_s8\000vhcaddq_rot270_x\000vhcaddq_rot270_x_s16\000vhcaddq_rot"
    "270_x_s32\000vhcaddq_rot270_x_s8\000vhcaddq_rot90_m\000vhcaddq_rot90_m_"
    "s16\000vhcaddq_rot90_m_s32\000vhcaddq_rot90_m_s8\000vhcaddq_rot90\000vh"
    "caddq_rot90_s16\000vhcaddq_rot90_s32\000vhcaddq_rot90_s8\000vhcaddq_rot"
    "90_x\000vhcaddq_rot90_x_s16\000vhcaddq_rot90_x_s32\000vhcaddq_rot90_x_s"
    "8\000vhsubq_m\000vhsubq_m_s16\000vhsubq_m_s32\000vhsubq_m_s8\000vhsubq_"
    "m_u16\000vhsubq_m_u32\000vhsubq_m_u8\000vhsubq\000vhsubq_s16\000vhsubq_"
    "s32\000vhsubq_s8\000vhsubq_u16\000vhsubq_u32\000vhsubq_u8\000vhsubq_x\000"
    "vhsubq_x_s16\000vhsubq_x_s32\000vhsubq_x_s8\000vhsubq_x_u16\000vhsubq_x"
    "_u32\000vhsubq_x_u8\000vidupq_m\000vidupq_m_n_u16\000vidupq_m_n_u32\000"
    "vidupq_m_n_u8\000vidupq_m_wb_u16\000vidupq_m_wb_u32\000vidupq_m_wb_u8\000"
    "vidupq_u16\000vidupq_n_u16\000vidupq_u32\000vidupq_n_u32\000vidupq_u8\000"
    "vidupq_n_u8\000vidupq_wb_u16\000vidupq_wb_u32\000vidupq_wb_u8\000vidupq"
    "_x_u16\000vidupq_x_n_u16\000vidupq_x_u32\000vidupq_x_n_u32\000vidupq_x_"
    "u8\000vidupq_x_n_u8\000vidupq_x_wb_u16\000vidupq_x_wb_u32\000vidupq_x_w"
    "b_u8\000viwdupq_m\000viwdupq_m_n_u16\000viwdupq_m_n_u32\000viwdupq_m_n_"
    "u8\000viwdupq_m_wb_u16\000viwdupq_m_wb_u32\000viwdupq_m_wb_u8\000viwdup"
    "q_u16\000viwdupq_n_u16\000viwdupq_u32\000viwdupq_n_u32\000viwdupq_u8\000"
    "viwdupq_n_u8\000viwdupq_wb_u16\000viwdupq_wb_u32\000viwdupq_wb_u8\000vi"
    "wdupq_x_u16\000viwdupq_x_n_u16\000viwdupq_x_u32\000viwdupq_x_n_u32\000v"
    "iwdupq_x_u8\000viwdupq_x_n_u8\000viwdupq_x_wb_u16\000viwdupq_x_wb_u32\000"
    "viwdupq_x_wb_u8\000vld1q\000vld1q_f16\000vld1q_f32\000vld1q_s16\000vld1"
    "q_s32\000vld1q_s8\000vld1q_u16\000vld1q_u32\000vld1q_u8\000vld1q_z\000v"
    "ld1q_z_f16\000vld1q_z_f32\000vld1q_z_s16\000vld1q_z_s32\000vld1q_z_s8\000"
    "vld1q_z_u16\000vld1q_z_u32\000vld1q_z_u8\000vld2q\000vld2q_f16\000vld2q"
    "_f32\000vld2q_s16\000vld2q_s32\000vld2q_s8\000vld2q_u16\000vld2q_u32\000"
    "vld2q_u8\000vld4q\000vld4q_f16\000vld4q_f32\000vld4q_s16\000vld4q_s32\000"
    "vld4q_s8\000vld4q_u16\000vld4q_u32\000vld4q_u8\000vldrbq_gather_offset\000"
    "vldrbq_gather_offset_s16\000vldrbq_gather_offset_s32\000vldrbq_gather_o"
    "ffset_s8\000vldrbq_gather_offset_u16\000vldrbq_gather_offset_u32\000vld"
    "rbq_gather_offset_u8\000vldrbq_gather_offset_z\000vldrbq_gather_offset_"
    "z_s16\000vldrbq_gather_offset_z_s32\000vldrbq_gather_offset_z_s8\000vld"
    "rbq_gather_offset_z_u16\000vldrbq_gather_offset_z_u32\000vldrbq_gather_"
    "offset_z_u8\000vldrbq_s16\000vldrbq_s32\000vldrbq_s8\000vldrbq_u16\000v"
    "ldrbq_u32\000vldrbq_u8\000vldrbq_z_s16\000vldrbq_z_s32\000vldrbq_z_s8\000"
    "vldrbq_z_u16\000vldrbq_z_u32\000vldrbq_z_u8\000vldrdq_gather_base_s64\000"
    "vldrdq_gather_base_u64\000vldrdq_gather_base_wb_s64\000vldrdq_gather_ba"
    "se_wb_u64\000vldrdq_gather_base_wb_z_s64\000vldrdq_gather_base_wb_z_u64"
    "\000vldrdq_gather_base_z_s64\000vldrdq_gather_base_z_u64\000vldrdq_gath"
    "er_offset\000vldrdq_gather_offset_s64\000vldrdq_gather_offset_u64\000vl"
    "drdq_gather_offset_z\000vldrdq_gather_offset_z_s64\000vldrdq_gather_off"
    "set_z_u64\000vldrdq_gather_shifted_offset\000vldrdq_gather_shifted_offs"
    "et_s64\000vldrdq_gather_shifted_offset_u64\000vldrdq_gather_shifted_off"
    "set_z\000vldrdq_gather_shifted_offset_z_s64\000vldrdq_gather_shifted_of"
    "fset_z_u64\000vldrhq_f16\000vldrhq_gather_offset\000vldrhq_gather_offse"
    "t_f16\000vldrhq_gather_offset_s16\000vldrhq_gather_offset_s32\000vldrhq"
    "_gather_offset_u16\000vldrhq_gather_offset_u32\000vldrhq_gather_offset_"
    "z\000vldrhq_gather_offset_z_f16\000vldrhq_gather_offset_z_s16\000vldrhq"
    "_gather_offset_z_s32\000vldrhq_gather_offset_z_u16\000vldrhq_gather_off"
    "set_z_u32\000vldrhq_gather_shifted_offset\000vldrhq_gather_shifted_offs"
    "et_f16\000vldrhq_gather_shifted_offset_s16\000vldrhq_gather_shifted_off"
    "set_s32\000vldrhq_gather_shifted_offset_u16\000vldrhq_gather_shifted_of"
    "fset_u32\000vldrhq_gather_shifted_offset_z\000vldrhq_gather_shifted_off"
    "set_z_f16\000vldrhq_gather_shifted_offset_z_s16\000vldrhq_gather_shifte"
    "d_offset_z_s32\000vldrhq_gather_shifted_offset_z_u16\000vldrhq_gather_s"
    "hifted_offset_z_u32\000vldrhq_s16\000vldrhq_s32\000vldrhq_u16\000vldrhq"
    "_u32\000vldrhq_z_f16\000vldrhq_z_s16\000vldrhq_z_s32\000vldrhq_z_u16\000"
    "vldrhq_z_u32\000vldrwq_f32\000vldrwq_gather_base_f32\000vldrwq_gather_b"
    "ase_s32\000vldrwq_gather_base_u32\000vldrwq_gather_base_wb_f32\000vldrw"
    "q_gather_base_wb_s32\000vldrwq_gather_base_wb_u32\000vldrwq_gather_base"
    "_wb_z_f32\000vldrwq_gather_base_wb_z_s32\000vldrwq_gather_base_wb_z_u32"
    "\000vldrwq_gather_base_z_f32\000vldrwq_gather_base_z_s32\000vldrwq_gath"
    "er_base_z_u32\000vldrwq_gather_offset\000vldrwq_gather_offset_f32\000vl"
    "drwq_gather_offset_s32\000vldrwq_gather_offset_u32\000vldrwq_gather_off"
    "set_z\000vldrwq_gather_offset_z_f32\000vldrwq_gather_offset_z_s32\000vl"
    "drwq_gather_offset_z_u32\000vldrwq_gather_shifted_offset\000vldrwq_gath"
    "er_shifted_offset_f32\000vldrwq_gather_shifted_offset_s32\000vldrwq_gat"
    "her_shifted_offset_u32\000vldrwq_gather_shifted_offset_z\000vldrwq_gath"
    "er_shifted_offset_z_f32\000vldrwq_gather_shifted_offset_z_s32\000vldrwq"
    "_gather_shifted_offset_z_u32\000vldrwq_s32\000vldrwq_u32\000vldrwq_z_f3"
    "2\000vldrwq_z_s32\000vldrwq_z_u32\000vmaxaq_m\000vmaxaq_m_s16\000vmaxaq"
    "_m_s32\000vmaxaq_m_s8\000vmaxaq\000vmaxaq_s16\000vmaxaq_s32\000vmaxaq_s"
    "8\000vmaxnmaq\000vmaxnmaq_f16\000vmaxnmaq_f32\000vmaxnmaq_m\000vmaxnmaq"
    "_m_f16\000vmaxnmaq_m_f32\000vmaxnmq\000vmaxnmq_f16\000vmaxnmq_f32\000vm"
    "axnmq_m\000vmaxnmq_m_f16\000vmaxnmq_m_f32\000vmaxnmq_x\000vmaxnmq_x_f16"
    "\000vmaxnmq_x_f32\000vmaxq_m\000vmaxq_m_s16\000vmaxq_m_s32\000vmaxq_m_s"
    "8\000vmaxq_m_u16\000vmaxq_m_u32\000vmaxq_m_u8\000vmaxq\000vmaxq_s16\000"
    "vmaxq_s32\000vmaxq_s8\000vmaxq_u16\000vmaxq_u32\000vmaxq_u8\000vmaxq_x\000"
    "vmaxq_x_s16\000vmaxq_x_s32\000vmaxq_x_s8\000vmaxq_x_u16\000vmaxq_x_u32\000"
    "vmaxq_x_u8\000vmaxvq\000vmaxvq_s16\000vmaxvq_s32\000vmaxvq_s8\000vmaxvq"
    "_u16\000vmaxvq_u32\000vmaxvq_u8\000vminaq_m\000vminaq_m_s16\000vminaq_m"
    "_s32\000vminaq_m_s8\000vminaq\000vminaq_s16\000vminaq_s32\000vminaq_s8\000"
    "vminnmaq\000vminnmaq_f16\000vminnmaq_f32\000vminnmaq_m\000vminnmaq_m_f1"
    "6\000vminnmaq_m_f32\000vminnmq\000vminnmq_f16\000vminnmq_f32\000vminnmq"
    "_m\000vminnmq_m_f16\000vminnmq_m_f32\000vminnmq_x\000vminnmq_x_f16\000v"
    "minnmq_x_f32\000vminq_m\000vminq_m_s16\000vminq_m_s32\000vminq_m_s8\000"
    "vminq_m_u16\000vminq_m_u32\000vminq_m_u8\000vminq\000vminq_s16\000vminq"
    "_s32\000vminq_s8\000vminq_u16\000vminq_u32\000vminq_u8\000vminq_x\000vm"
    "inq_x_s16\000vminq_x_s32\000vminq_x_s8\000vminq_x_u16\000vminq_x_u32\000"
    "vminq_x_u8\000vminvq\000vminvq_s16\000vminvq_s32\000vminvq_s8\000vminvq"
    "_u16\000vminvq_u32\000vminvq_u8\000vmladavaq_p\000vmladavaq_p_s16\000vm"
    "ladavaq_p_s32\000vmladavaq_p_s8\000vmladavaq_p_u16\000vmladavaq_p_u32\000"
    "vmladavaq_p_u8\000vmladavaq\000vmladavaq_s16\000vmladavaq_s32\000vmlada"
    "vaq_s8\000vmladavaq_u16\000vmladavaq_u32\000vmladavaq_u8\000vmladavaxq_"
    "p\000vmladavaxq_p_s16\000vmladavaxq_p_s32\000vmladavaxq_p_s8\000vmladav"
    "axq\000vmladavaxq_s16\000vmladavaxq_s32\000vmladavaxq_s8\000vmladavq_p\000"
    "vmladavq_p_s16\000vmladavq_p_s32\000vmladavq_p_s8\000vmladavq_p_u16\000"
    "vmladavq_p_u32\000vmladavq_p_u8\000vmladavq\000vmladavq_s16\000vmladavq"
    "_s32\000vmladavq_s8\000vmladavq_u16\000vmladavq_u32\000vmladavq_u8\000v"
    "mladavxq_p\000vmladavxq_p_s16\000vmladavxq_p_s32\000vmladavxq_p_s8\000v"
    "mladavxq\000vmladavxq_s16\000vmladavxq_s32\000vmladavxq_s8\000vmlaldava"
    "q_p\000vmlaldavaq_p_s16\000vmlaldavaq_p_s32\000vmlaldavaq_p_u16\000vmla"
    "ldavaq_p_u32\000vmlaldavaq\000vmlaldavaq_s16\000vmlaldavaq_s32\000vmlal"
    "davaq_u16\000vmlaldavaq_u32\000vmlaldavaxq_p\000vmlaldavaxq_p_s16\000vm"
    "laldavaxq_p_s32\000vmlaldavaxq\000vmlaldavaxq_s16\000vmlaldavaxq_s32\000"
    "vmlaldavq_p\000vmlaldavq_p_s16\000vmlaldavq_p_s32\000vmlaldavq_p_u16\000"
    "vmlaldavq_p_u32\000vmlaldavq\000vmlaldavq_s16\000vmlaldavq_s32\000vmlal"
    "davq_u16\000vmlaldavq_u32\000vmlaldavxq_p\000vmlaldavxq_p_s16\000vmlald"
    "avxq_p_s32\000vmlaldavxq\000vmlaldavxq_s16\000vmlaldavxq_s32\000vmlsdav"
    "aq_p\000vmlsdavaq_p_s16\000vmlsdavaq_p_s32\000vmlsdavaq_p_s8\000vmlsdav"
    "aq\000vmlsdavaq_s16\000vmlsdavaq_s32\000vmlsdavaq_s8\000vmlsdavaxq_p\000"
    "vmlsdavaxq_p_s16\000vmlsdavaxq_p_s32\000vmlsdavaxq_p_s8\000vmlsdavaxq\000"
    "vmlsdavaxq_s16\000vmlsdavaxq_s32\000vmlsdavaxq_s8\000vmlsdavq_p\000vmls"
    "davq_p_s16\000vmlsdavq_p_s32\000vmlsdavq_p_s8\000vmlsdavq\000vmlsdavq_s"
    "16\000vmlsdavq_s32\000vmlsdavq_s8\000vmlsdavxq_p\000vmlsdavxq_p_s16\000"
    "vmlsdavxq_p_s32\000vmlsdavxq_p_s8\000vmlsdavxq\000vmlsdavxq_s16\000vmls"
    "davxq_s32\000vmlsdavxq_s8\000vmlsldavaq_p\000vmlsldavaq_p_s16\000vmlsld"
    "avaq_p_s32\000vmlsldavaq\000vmlsldavaq_s16\000vmlsldavaq_s32\000vmlslda"
    "vaxq_p\000vmlsldavaxq_p_s16\000vmlsldavaxq_p_s32\000vmlsldavaxq\000vmls"
    "ldavaxq_s16\000vmlsldavaxq_s32\000vmlsldavq_p\000vmlsldavq_p_s16\000vml"
    "sldavq_p_s32\000vmlsldavq\000vmlsldavq_s16\000vmlsldavq_s32\000vmlsldav"
    "xq_p\000vmlsldavxq_p_s16\000vmlsldavxq_p_s32\000vmlsldavxq\000vmlsldavx"
    "q_s16\000vmlsldavxq_s32\000vmulhq_m\000vmulhq_m_s16\000vmulhq_m_s32\000"
    "vmulhq_m_s8\000vmulhq_m_u16\000vmulhq_m_u32\000vmulhq_m_u8\000vmulhq\000"
    "vmulhq_s16\000vmulhq_s32\000vmulhq_s8\000vmulhq_u16\000vmulhq_u32\000vm"
    "ulhq_u8\000vmulhq_x\000vmulhq_x_s16\000vmulhq_x_s32\000vmulhq_x_s8\000v"
    "mulhq_x_u16\000vmulhq_x_u32\000vmulhq_x_u8\000vmullbq_int_m\000vmullbq_"
    "int_m_s16\000vmullbq_int_m_s32\000vmullbq_int_m_s8\000vmullbq_int_m_u16"
    "\000vmullbq_int_m_u32\000vmullbq_int_m_u8\000vmullbq_int\000vmullbq_int"
    "_s16\000vmullbq_int_s32\000vmullbq_int_s8\000vmullbq_int_u16\000vmullbq"
    "_int_u32\000vmullbq_int_u8\000vmullbq_int_x\000vmullbq_int_x_s16\000vmu"
    "llbq_int_x_s32\000vmullbq_int_x_s8\000vmullbq_int_x_u16\000vmullbq_int_"
    "x_u32\000vmullbq_int_x_u8\000vmullbq_poly_m\000vmullbq_poly_m_p16\000vm"
    "ullbq_poly_m_p8\000vmullbq_poly\000vmullbq_poly_p16\000vmullbq_poly_p8\000"
    "vmullbq_poly_x\000vmullbq_poly_x_p16\000vmullbq_poly_x_p8\000vmulltq_in"
    "t_m\000vmulltq_int_m_s16\000vmulltq_int_m_s32\000vmulltq_int_m_s8\000vm"
    "ulltq_int_m_u16\000vmulltq_int_m_u32\000vmulltq_int_m_u8\000vmulltq_int"
    "\000vmulltq_int_s16\000vmulltq_int_s32\000vmulltq_int_s8\000vmulltq_int"
    "_u16\000vmulltq_int_u32\000vmulltq_int_u8\000vmulltq_int_x\000vmulltq_i"
    "nt_x_s16\000vmulltq_int_x_s32\000vmulltq_int_x_s8\000vmulltq_int_x_u16\000"
    "vmulltq_int_x_u32\000vmulltq_int_x_u8\000vmulltq_poly_m\000vmulltq_poly"
    "_m_p16\000vmulltq_poly_m_p8\000vmulltq_poly\000vmulltq_poly_p16\000vmul"
    "ltq_poly_p8\000vmulltq_poly_x\000vmulltq_poly_x_p16\000vmulltq_poly_x_p"
    "8\000vmulq\000vmulq_f16\000vmulq_f32\000vmulq_m\000vmulq_m_f16\000vmulq"
    "_m_f32\000vmulq_m_s16\000vmulq_m_s32\000vmulq_m_s8\000vmulq_m_u16\000vm"
    "ulq_m_u32\000vmulq_m_u8\000vmulq_s16\000vmulq_s32\000vmulq_s8\000vmulq_"
    "u16\000vmulq_u32\000vmulq_u8\000vmulq_x\000vmulq_x_f16\000vmulq_x_f32\000"
    "vmulq_x_s16\000vmulq_x_s32\000vmulq_x_s8\000vmulq_x_u16\000vmulq_x_u32\000"
    "vmulq_x_u8\000vmvnq_m\000vmvnq_m_n_s16\000vmvnq_m_n_s32\000vmvnq_m_n_u1"
    "6\000vmvnq_m_n_u32\000vmvnq_n_s16\000vmvnq_n_s32\000vmvnq_n_u16\000vmvn"
    "q_n_u32\000vmvnq_x_n_s16\000vmvnq_x_n_s32\000vmvnq_x_n_u16\000vmvnq_x_n"
    "_u32\000vornq\000vornq_f16\000vornq_f32\000vornq_m\000vornq_m_f16\000vo"
    "rnq_m_f32\000vornq_m_s16\000vornq_m_s32\000vornq_m_s8\000vornq_m_u16\000"
    "vornq_m_u32\000vornq_m_u8\000vornq_s16\000vornq_s32\000vornq_s8\000vorn"
    "q_u16\000vornq_u32\000vornq_u8\000vornq_x\000vornq_x_f16\000vornq_x_f32"
    "\000vornq_x_s16\000vornq_x_s32\000vornq_x_s8\000vornq_x_u16\000vornq_x_"
    "u32\000vornq_x_u8\000vorrq\000vorrq_f16\000vorrq_f32\000vorrq_m\000vorr"
    "q_m_f16\000vorrq_m_f32\000vorrq_m_n\000vorrq_m_n_s16\000vorrq_m_n_s32\000"
    "vorrq_m_n_u16\000vorrq_m_n_u32\000vorrq_m_s16\000vorrq_m_s32\000vorrq_m"
    "_s8\000vorrq_m_u16\000vorrq_m_u32\000vorrq_m_u8\000vorrq_n_s16\000vorrq"
    "_n_s32\000vorrq_n_u16\000vorrq_n_u32\000vorrq_s16\000vorrq_s32\000vorrq"
    "_s8\000vorrq_u16\000vorrq_u32\000vorrq_u8\000vorrq_x\000vorrq_x_f16\000"
    "vorrq_x_f32\000vorrq_x_s16\000vorrq_x_s32\000vorrq_x_s8\000vorrq_x_u16\000"
    "vorrq_x_u32\000vorrq_x_u8\000vpnot\000vpselq\000vpselq_f16\000vpselq_f3"
    "2\000vpselq_s16\000vpselq_s32\000vpselq_s64\000vpselq_s8\000vpselq_u16\000"
    "vpselq_u32\000vpselq_u64\000vpselq_u8\000vqaddq_m\000vqaddq_m_s16\000vq"
    "addq_m_s32\000vqaddq_m_s8\000vqaddq_m_u16\000vqaddq_m_u32\000vqaddq_m_u"
    "8\000vqaddq\000vqaddq_s16\000vqaddq_s32\000vqaddq_s8\000vqaddq_u16\000v"
    "qaddq_u32\000vqaddq_u8\000vqdmulhq_m\000vqdmulhq_m_s16\000vqdmulhq_m_s3"
    "2\000vqdmulhq_m_s8\000vqdmulhq\000vqdmulhq_s16\000vqdmulhq_s32\000vqdmu"
    "lhq_s8\000vqrdmulhq_m\000vqrdmulhq_m_s16\000vqrdmulhq_m_s32\000vqrdmulh"
    "q_m_s8\000vqrdmulhq\000vqrdmulhq_s16\000vqrdmulhq_s32\000vqrdmulhq_s8\000"
    "vqrshlq_m_n\000vqrshlq_m_n_s16\000vqrshlq_m_n_s32\000vqrshlq_m_n_s8\000"
    "vqrshlq_m_n_u16\000vqrshlq_m_n_u32\000vqrshlq_m_n_u8\000vqrshlq_m\000vq"
    "rshlq_m_s16\000vqrshlq_m_s32\000vqrshlq_m_s8\000vqrshlq_m_u16\000vqrshl"
    "q_m_u32\000vqrshlq_m_u8\000vqrshlq\000vqrshlq_n_s16\000vqrshlq_n_s32\000"
    "vqrshlq_n_s8\000vqrshlq_n_u16\000vqrshlq_n_u32\000vqrshlq_n_u8\000vqrsh"
    "lq_s16\000vqrshlq_s32\000vqrshlq_s8\000vqrshlq_u16\000vqrshlq_u32\000vq"
    "rshlq_u8\000vqrshrnbq_m\000vqrshrnbq_m_n_s16\000vqrshrnbq_m_n_s32\000vq"
    "rshrnbq_m_n_u16\000vqrshrnbq_m_n_u32\000vqrshrnbq\000vqrshrnbq_n_s16\000"
    "vqrshrnbq_n_s32\000vqrshrnbq_n_u16\000vqrshrnbq_n_u32\000vqrshrntq_m\000"
    "vqrshrntq_m_n_s16\000vqrshrntq_m_n_s32\000vqrshrntq_m_n_u16\000vqrshrnt"
    "q_m_n_u32\000vqrshrntq\000vqrshrntq_n_s16\000vqrshrntq_n_s32\000vqrshrn"
    "tq_n_u16\000vqrshrntq_n_u32\000vqrshrunbq_m\000vqrshrunbq_m_n_s16\000vq"
    "rshrunbq_m_n_s32\000vqrshrunbq\000vqrshrunbq_n_s16\000vqrshrunbq_n_s32\000"
    "vqrshruntq_m\000vqrshruntq_m_n_s16\000vqrshruntq_m_n_s32\000vqrshruntq\000"
    "vqrshruntq_n_s16\000vqrshruntq_n_s32\000vqshlq_m_n\000vqshlq_m_n_s16\000"
    "vqshlq_m_n_s32\000vqshlq_m_n_s8\000vqshlq_m_n_u16\000vqshlq_m_n_u32\000"
    "vqshlq_m_n_u8\000vqshlq_m_r\000vqshlq_m_r_s16\000vqshlq_m_r_s32\000vqsh"
    "lq_m_r_s8\000vqshlq_m_r_u16\000vqshlq_m_r_u32\000vqshlq_m_r_u8\000vqshl"
    "q_m\000vqshlq_m_s16\000vqshlq_m_s32\000vqshlq_m_s8\000vqshlq_m_u16\000v"
    "qshlq_m_u32\000vqshlq_m_u8\000vqshlq_n\000vqshlq_n_s16\000vqshlq_n_s32\000"
    "vqshlq_n_s8\000vqshlq_n_u16\000vqshlq_n_u32\000vqshlq_n_u8\000vqshlq_r\000"
    "vqshlq_r_s16\000vqshlq_r_s32\000vqshlq_r_s8\000vqshlq_r_u16\000vqshlq_r"
    "_u32\000vqshlq_r_u8\000vqshlq\000vqshlq_s16\000vqshlq_s32\000vqshlq_s8\000"
    "vqshlq_u16\000vqshlq_u32\000vqshlq_u8\000vqshluq_m\000vqshluq_m_n_s16\000"
    "vqshluq_m_n_s32\000vqshluq_m_n_s8\000vqshluq\000vqshluq_n_s16\000vqshlu"
    "q_n_s32\000vqshluq_n_s8\000vqshrnbq_m\000vqshrnbq_m_n_s16\000vqshrnbq_m"
    "_n_s32\000vqshrnbq_m_n_u16\000vqshrnbq_m_n_u32\000vqshrnbq\000vqshrnbq_"
    "n_s16\000vqshrnbq_n_s32\000vqshrnbq_n_u16\000vqshrnbq_n_u32\000vqshrntq"
    "_m\000vqshrntq_m_n_s16\000vqshrntq_m_n_s32\000vqshrntq_m_n_u16\000vqshr"
    "ntq_m_n_u32\000vqshrntq\000vqshrntq_n_s16\000vqshrntq_n_s32\000vqshrntq"
    "_n_u16\000vqshrntq_n_u32\000vqshrunbq_m\000vqshrunbq_m_n_s16\000vqshrun"
    "bq_m_n_s32\000vqshrunbq\000vqshrunbq_n_s16\000vqshrunbq_n_s32\000vqshru"
    "ntq_m\000vqshruntq_m_n_s16\000vqshruntq_m_n_s32\000vqshruntq\000vqshrun"
    "tq_n_s16\000vqshruntq_n_s32\000vqsubq_m\000vqsubq_m_s16\000vqsubq_m_s32"
    "\000vqsubq_m_s8\000vqsubq_m_u16\000vqsubq_m_u32\000vqsubq_m_u8\000vqsub"
    "q\000vqsubq_s16\000vqsubq_s32\000vqsubq_s8\000vqsubq_u16\000vqsubq_u32\000"
    "vqsubq_u8\000vreinterpretq_f16\000vreinterpretq_f16_f32\000vreinterpret"
    "q_f16_s16\000vreinterpretq_f16_s32\000vreinterpretq_f16_s64\000vreinter"
    "pretq_f16_s8\000vreinterpretq_f16_u16\000vreinterpretq_f16_u32\000vrein"
    "terpretq_f16_u64\000vreinterpretq_f16_u8\000vreinterpretq_f32\000vreint"
    "erpretq_f32_f16\000vreinterpretq_f32_s16\000vreinterpretq_f32_s32\000vr"
    "einterpretq_f32_s64\000vreinterpretq_f32_s8\000vreinterpretq_f32_u16\000"
    "vreinterpretq_f32_u32\000vreinterpretq_f32_u64\000vreinterpretq_f32_u8\000"
    "vreinterpretq_s16\000vreinterpretq_s16_f16\000vreinterpretq_s16_f32\000"
    "vreinterpretq_s16_s32\000vreinterpretq_s16_s64\000vreinterpretq_s16_s8\000"
    "vreinterpretq_s16_u16\000vreinterpretq_s16_u32\000vreinterpretq_s16_u64"
    "\000vreinterpretq_s16_u8\000vreinterpretq_s32\000vreinterpretq_s32_f16\000"
    "vreinterpretq_s32_f32\000vreinterpretq_s32_s16\000vreinterpretq_s32_s64"
    "\000vreinterpretq_s32_s8\000vreinterpretq_s32_u16\000vreinterpretq_s32_"
    "u32\000vreinterpretq_s32_u64\000vreinterpretq_s32_u8\000vreinterpretq_s"
    "64\000vreinterpretq_s64_f16\000vreinterpretq_s64_f32\000vreinterpretq_s"
    "64_s16\000vreinterpretq_s64_s32\000vreinterpretq_s64_s8\000vreinterpret"
    "q_s64_u16\000vreinterpretq_s64_u32\000vreinterpretq_s64_u64\000vreinter"
    "pretq_s64_u8\000vreinterpretq_s8\000vreinterpretq_s8_f16\000vreinterpre"
    "tq_s8_f32\000vreinterpretq_s8_s16\000vreinterpretq_s8_s32\000vreinterpr"
    "etq_s8_s64\000vreinterpretq_s8_u16\000vreinterpretq_s8_u32\000vreinterp"
    "retq_s8_u64\000vreinterpretq_s8_u8\000vreinterpretq_u16\000vreinterpret"
    "q_u16_f16\000vreinterpretq_u16_f32\000vreinterpretq_u16_s16\000vreinter"
    "pretq_u16_s32\000vreinterpretq_u16_s64\000vreinterpretq_u16_s8\000vrein"
    "terpretq_u16_u32\000vreinterpretq_u16_u64\000vreinterpretq_u16_u8\000vr"
    "einterpretq_u32\000vreinterpretq_u32_f16\000vreinterpretq_u32_f32\000vr"
    "einterpretq_u32_s16\000vreinterpretq_u32_s32\000vreinterpretq_u32_s64\000"
    "vreinterpretq_u32_s8\000vreinterpretq_u32_u16\000vreinterpretq_u32_u64\000"
    "vreinterpretq_u32_u8\000vreinterpretq_u64\000vreinterpretq_u64_f16\000v"
    "reinterpretq_u64_f32\000vreinterpretq_u64_s16\000vreinterpretq_u64_s32\000"
    "vreinterpretq_u64_s64\000vreinterpretq_u64_s8\000vreinterpretq_u64_u16\000"
    "vreinterpretq_u64_u32\000vreinterpretq_u64_u8\000vreinterpretq_u8\000vr"
    "einterpretq_u8_f16\000vreinterpretq_u8_f32\000vreinterpretq_u8_s16\000v"
    "reinterpretq_u8_s32\000vreinterpretq_u8_s64\000vreinterpretq_u8_s8\000v"
    "reinterpretq_u8_u16\000vreinterpretq_u8_u32\000vreinterpretq_u8_u64\000"
    "vrhaddq_m\000vrhaddq_m_s16\000vrhaddq_m_s32\000vrhaddq_m_s8\000vrhaddq_"
    "m_u16\000vrhaddq_m_u32\000vrhaddq_m_u8\000vrhaddq\000vrhaddq_s16\000vrh"
    "addq_s32\000vrhaddq_s8\000vrhaddq_u16\000vrhaddq_u32\000vrhaddq_u8\000v"
    "rhaddq_x\000vrhaddq_x_s16\000vrhaddq_x_s32\000vrhaddq_x_s8\000vrhaddq_x"
    "_u16\000vrhaddq_x_u32\000vrhaddq_x_u8\000vrmlaldavhaq_p\000vrmlaldavhaq"
    "_p_s32\000vrmlaldavhaq_p_u32\000vrmlaldavhaq\000vrmlaldavhaq_s32\000vrm"
    "laldavhaq_u32\000vrmlaldavhaxq_p\000vrmlaldavhaxq_p_s32\000vrmlaldavhax"
    "q\000vrmlaldavhaxq_s32\000vrmlaldavhq_p\000vrmlaldavhq_p_s32\000vrmlald"
    "avhq_p_u32\000vrmlaldavhq\000vrmlaldavhq_s32\000vrmlaldavhq_u32\000vrml"
    "aldavhxq_p\000vrmlaldavhxq_p_s32\000vrmlaldavhxq\000vrmlaldavhxq_s32\000"
    "vrmlsldavhaq_p\000vrmlsldavhaq_p_s32\000vrmlsldavhaq\000vrmlsldavhaq_s3"
    "2\000vrmlsldavhaxq_p\000vrmlsldavhaxq_p_s32\000vrmlsldavhaxq\000vrmlsld"
    "avhaxq_s32\000vrmlsldavhq_p\000vrmlsldavhq_p_s32\000vrmlsldavhq\000vrml"
    "sldavhq_s32\000vrmlsldavhxq_p\000vrmlsldavhxq_p_s32\000vrmlsldavhxq\000"
    "vrmlsldavhxq_s32\000vrmulhq_m\000vrmulhq_m_s16\000vrmulhq_m_s32\000vrmu"
    "lhq_m_s8\000vrmulhq_m_u16\000vrmulhq_m_u32\000vrmulhq_m_u8\000vrmulhq\000"
    "vrmulhq_s16\000vrmulhq_s32\000vrmulhq_s8\000vrmulhq_u16\000vrmulhq_u32\000"
    "vrmulhq_u8\000vrmulhq_x\000vrmulhq_x_s16\000vrmulhq_x_s32\000vrmulhq_x_"
    "s8\000vrmulhq_x_u16\000vrmulhq_x_u32\000vrmulhq_x_u8\000vrshlq_m_n\000v"
    "rshlq_m_n_s16\000vrshlq_m_n_s32\000vrshlq_m_n_s8\000vrshlq_m_n_u16\000v"
    "rshlq_m_n_u32\000vrshlq_m_n_u8\000vrshlq_m\000vrshlq_m_s16\000vrshlq_m_"
    "s32\000vrshlq_m_s8\000vrshlq_m_u16\000vrshlq_m_u32\000vrshlq_m_u8\000vr"
    "shlq\000vrshlq_n_s16\000vrshlq_n_s32\000vrshlq_n_s8\000vrshlq_n_u16\000"
    "vrshlq_n_u32\000vrshlq_n_u8\000vrshlq_s16\000vrshlq_s32\000vrshlq_s8\000"
    "vrshlq_u16\000vrshlq_u32\000vrshlq_u8\000vrshlq_x\000vrshlq_x_s16\000vr"
    "shlq_x_s32\000vrshlq_x_s8\000vrshlq_x_u16\000vrshlq_x_u32\000vrshlq_x_u"
    "8\000vrshrnbq_m\000vrshrnbq_m_n_s16\000vrshrnbq_m_n_s32\000vrshrnbq_m_n"
    "_u16\000vrshrnbq_m_n_u32\000vrshrnbq\000vrshrnbq_n_s16\000vrshrnbq_n_s3"
    "2\000vrshrnbq_n_u16\000vrshrnbq_n_u32\000vrshrntq_m\000vrshrntq_m_n_s16"
    "\000vrshrntq_m_n_s32\000vrshrntq_m_n_u16\000vrshrntq_m_n_u32\000vrshrnt"
    "q\000vrshrntq_n_s16\000vrshrntq_n_s32\000vrshrntq_n_u16\000vrshrntq_n_u"
    "32\000vrshrq_m\000vrshrq_m_n_s16\000vrshrq_m_n_s32\000vrshrq_m_n_s8\000"
    "vrshrq_m_n_u16\000vrshrq_m_n_u32\000vrshrq_m_n_u8\000vrshrq\000vrshrq_n"
    "_s16\000vrshrq_n_s32\000vrshrq_n_s8\000vrshrq_n_u16\000vrshrq_n_u32\000"
    "vrshrq_n_u8\000vrshrq_x\000vrshrq_x_n_s16\000vrshrq_x_n_s32\000vrshrq_x"
    "_n_s8\000vrshrq_x_n_u16\000vrshrq_x_n_u32\000vrshrq_x_n_u8\000vsetq_lan"
    "e\000vsetq_lane_f16\000vsetq_lane_f32\000vsetq_lane_s16\000vsetq_lane_s"
    "32\000vsetq_lane_s64\000vsetq_lane_s8\000vsetq_lane_u16\000vsetq_lane_u"
    "32\000vsetq_lane_u64\000vsetq_lane_u8\000vshllbq_m\000vshllbq_m_n_s16\000"
    "vshllbq_m_n_s8\000vshllbq_m_n_u16\000vshllbq_m_n_u8\000vshllbq\000vshll"
    "bq_n_s16\000vshllbq_n_s8\000vshllbq_n_u16\000vshllbq_n_u8\000vshllbq_x\000"
    "vshllbq_x_n_s16\000vshllbq_x_n_s8\000vshllbq_x_n_u16\000vshllbq_x_n_u8\000"
    "vshlltq_m\000vshlltq_m_n_s16\000vshlltq_m_n_s8\000vshlltq_m_n_u16\000vs"
    "hlltq_m_n_u8\000vshlltq\000vshlltq_n_s16\000vshlltq_n_s8\000vshlltq_n_u"
    "16\000vshlltq_n_u8\000vshlltq_x\000vshlltq_x_n_s16\000vshlltq_x_n_s8\000"
    "vshlltq_x_n_u16\000vshlltq_x_n_u8\000vshlq_m_n\000vshlq_m_n_s16\000vshl"
    "q_m_n_s32\000vshlq_m_n_s8\000vshlq_m_n_u16\000vshlq_m_n_u32\000vshlq_m_"
    "n_u8\000vshlq_m_r\000vshlq_m_r_s16\000vshlq_m_r_s32\000vshlq_m_r_s8\000"
    "vshlq_m_r_u16\000vshlq_m_r_u32\000vshlq_m_r_u8\000vshlq_m\000vshlq_m_s1"
    "6\000vshlq_m_s32\000vshlq_m_s8\000vshlq_m_u16\000vshlq_m_u32\000vshlq_m"
    "_u8\000vshlq_n\000vshlq_n_s16\000vshlq_n_s32\000vshlq_n_s8\000vshlq_n_u"
    "16\000vshlq_n_u32\000vshlq_n_u8\000vshlq_r\000vshlq_r_s16\000vshlq_r_s3"
    "2\000vshlq_r_s8\000vshlq_r_u16\000vshlq_r_u32\000vshlq_r_u8\000vshlq\000"
    "vshlq_s16\000vshlq_s32\000vshlq_s8\000vshlq_u16\000vshlq_u32\000vshlq_u"
    "8\000vshlq_x_n\000vshlq_x_n_s16\000vshlq_x_n_s32\000vshlq_x_n_s8\000vsh"
    "lq_x_n_u16\000vshlq_x_n_u32\000vshlq_x_n_u8\000vshlq_x\000vshlq_x_s16\000"
    "vshlq_x_s32\000vshlq_x_s8\000vshlq_x_u16\000vshlq_x_u32\000vshlq_x_u8\000"
    "vshrnbq_m\000vshrnbq_m_n_s16\000vshrnbq_m_n_s32\000vshrnbq_m_n_u16\000v"
    "shrnbq_m_n_u32\000vshrnbq\000vshrnbq_n_s16\000vshrnbq_n_s32\000vshrnbq_"
    "n_u16\000vshrnbq_n_u32\000vshrntq_m\000vshrntq_m_n_s16\000vshrntq_m_n_s"
    "32\000vshrntq_m_n_u16\000vshrntq_m_n_u32\000vshrntq\000vshrntq_n_s16\000"
    "vshrntq_n_s32\000vshrntq_n_u16\000vshrntq_n_u32\000vshrq_m\000vshrq_m_n"
    "_s16\000vshrq_m_n_s32\000vshrq_m_n_s8\000vshrq_m_n_u16\000vshrq_m_n_u32"
    "\000vshrq_m_n_u8\000vshrq\000vshrq_n_s16\000vshrq_n_s32\000vshrq_n_s8\000"
    "vshrq_n_u16\000vshrq_n_u32\000vshrq_n_u8\000vshrq_x\000vshrq_x_n_s16\000"
    "vshrq_x_n_s32\000vshrq_x_n_s8\000vshrq_x_n_u16\000vshrq_x_n_u32\000vshr"
    "q_x_n_u8\000vsliq_m\000vsliq_m_n_s16\000vsliq_m_n_s32\000vsliq_m_n_s8\000"
    "vsliq_m_n_u16\000vsliq_m_n_u32\000vsliq_m_n_u8\000vsliq\000vsliq_n_s16\000"
    "vsliq_n_s32\000vsliq_n_s8\000vsliq_n_u16\000vsliq_n_u32\000vsliq_n_u8\000"
    "vsriq_m\000vsriq_m_n_s16\000vsriq_m_n_s32\000vsriq_m_n_s8\000vsriq_m_n_"
    "u16\000vsriq_m_n_u32\000vsriq_m_n_u8\000vsriq\000vsriq_n_s16\000vsriq_n"
    "_s32\000vsriq_n_s8\000vsriq_n_u16\000vsriq_n_u32\000vsriq_n_u8\000vst1q"
    "\000vst1q_f16\000vst1q_f32\000vst1q_p\000vst1q_p_f16\000vst1q_p_f32\000"
    "vst1q_p_s16\000vst1q_p_s32\000vst1q_p_s8\000vst1q_p_u16\000vst1q_p_u32\000"
    "vst1q_p_u8\000vst1q_s16\000vst1q_s32\000vst1q_s8\000vst1q_u16\000vst1q_"
    "u32\000vst1q_u8\000vst2q\000vst2q_f16\000vst2q_f32\000vst2q_s16\000vst2"
    "q_s32\000vst2q_s8\000vst2q_u16\000vst2q_u32\000vst2q_u8\000vst4q\000vst"
    "4q_f16\000vst4q_f32\000vst4q_s16\000vst4q_s32\000vst4q_s8\000vst4q_u16\000"
    "vst4q_u32\000vst4q_u8\000vstrbq_p\000vstrbq_p_s16\000vstrbq_p_s32\000vs"
    "trbq_p_s8\000vstrbq_p_u16\000vstrbq_p_u32\000vstrbq_p_u8\000vstrbq\000v"
    "strbq_s16\000vstrbq_s32\000vstrbq_s8\000vstrbq_scatter_offset_p\000vstr"
    "bq_scatter_offset_p_s16\000vstrbq_scatter_offset_p_s32\000vstrbq_scatte"
    "r_offset_p_s8\000vstrbq_scatter_offset_p_u16\000vstrbq_scatter_offset_p"
    "_u32\000vstrbq_scatter_offset_p_u8\000vstrbq_scatter_offset\000vstrbq_s"
    "catter_offset_s16\000vstrbq_scatter_offset_s32\000vstrbq_scatter_offset"
    "_s8\000vstrbq_scatter_offset_u16\000vstrbq_scatter_offset_u32\000vstrbq"
    "_scatter_offset_u8\000vstrbq_u16\000vstrbq_u32\000vstrbq_u8\000vstrdq_s"
    "catter_base_p\000vstrdq_scatter_base_p_s64\000vstrdq_scatter_base_p_u64"
    "\000vstrdq_scatter_base\000vstrdq_scatter_base_s64\000vstrdq_scatter_ba"
    "se_u64\000vstrdq_scatter_base_wb_p\000vstrdq_scatter_base_wb_p_s64\000v"
    "strdq_scatter_base_wb_p_u64\000vstrdq_scatter_base_wb\000vstrdq_scatter"
    "_base_wb_s64\000vstrdq_scatter_base_wb_u64\000vstrdq_scatter_offset_p\000"
    "vstrdq_scatter_offset_p_s64\000vstrdq_scatter_offset_p_u64\000vstrdq_sc"
    "atter_offset\000vstrdq_scatter_offset_s64\000vstrdq_scatter_offset_u64\000"
    "vstrdq_scatter_shifted_offset_p\000vstrdq_scatter_shifted_offset_p_s64\000"
    "vstrdq_scatter_shifted_offset_p_u64\000vstrdq_scatter_shifted_offset\000"
    "vstrdq_scatter_shifted_offset_s64\000vstrdq_scatter_shifted_offset_u64\000"
    "vstrhq\000vstrhq_f16\000vstrhq_p\000vstrhq_p_f16\000vstrhq_p_s16\000vst"
    "rhq_p_s32\000vstrhq_p_u16\000vstrhq_p_u32\000vstrhq_s16\000vstrhq_s32\000"
    "vstrhq_scatter_offset\000vstrhq_scatter_offset_f16\000vstrhq_scatter_of"
    "fset_p\000vstrhq_scatter_offset_p_f16\000vstrhq_scatter_offset_p_s16\000"
    "vstrhq_scatter_offset_p_s32\000vstrhq_scatter_offset_p_u16\000vstrhq_sc"
    "atter_offset_p_u32\000vstrhq_scatter_offset_s16\000vstrhq_scatter_offse"
    "t_s32\000vstrhq_scatter_offset_u16\000vstrhq_scatter_offset_u32\000vstr"
    "hq_scatter_shifted_offset\000vstrhq_scatter_shifted_offset_f16\000vstrh"
    "q_scatter_shifted_offset_p\000vstrhq_scatter_shifted_offset_p_f16\000vs"
    "trhq_scatter_shifted_offset_p_s16\000vstrhq_scatter_shifted_offset_p_s3"
    "2\000vstrhq_scatter_shifted_offset_p_u16\000vstrhq_scatter_shifted_offs"
    "et_p_u32\000vstrhq_scatter_shifted_offset_s16\000vstrhq_scatter_shifted"
    "_offset_s32\000vstrhq_scatter_shifted_offset_u16\000vstrhq_scatter_shif"
    "ted_offset_u32\000vstrhq_u16\000vstrhq_u32\000vstrwq\000vstrwq_f32\000v"
    "strwq_p\000vstrwq_p_f32\000vstrwq_p_s32\000vstrwq_p_u32\000vstrwq_s32\000"
    "vstrwq_scatter_base\000vstrwq_scatter_base_f32\000vstrwq_scatter_base_p"
    "\000vstrwq_scatter_base_p_f32\000vstrwq_scatter_base_p_s32\000vstrwq_sc"
    "atter_base_p_u32\000vstrwq_scatter_base_s32\000vstrwq_scatter_base_u32\000"
    "vstrwq_scatter_base_wb\000vstrwq_scatter_base_wb_f32\000vstrwq_scatter_"
    "base_wb_p\000vstrwq_scatter_base_wb_p_f32\000vstrwq_scatter_base_wb_p_s"
    "32\000vstrwq_scatter_base_wb_p_u32\000vstrwq_scatter_base_wb_s32\000vst"
    "rwq_scatter_base_wb_u32\000vstrwq_scatter_offset\000vstrwq_scatter_offs"
    "et_f32\000vstrwq_scatter_offset_p\000vstrwq_scatter_offset_p_f32\000vst"
    "rwq_scatter_offset_p_s32\000vstrwq_scatter_offset_p_u32\000vstrwq_scatt"
    "er_offset_s32\000vstrwq_scatter_offset_u32\000vstrwq_scatter_shifted_of"
    "fset\000vstrwq_scatter_shifted_offset_f32\000vstrwq_scatter_shifted_off"
    "set_p\000vstrwq_scatter_shifted_offset_p_f32\000vstrwq_scatter_shifted_"
    "offset_p_s32\000vstrwq_scatter_shifted_offset_p_u32\000vstrwq_scatter_s"
    "hifted_offset_s32\000vstrwq_scatter_shifted_offset_u32\000vstrwq_u32\000"
    "vsubq\000vsubq_f16\000vsubq_f32\000vsubq_m\000vsubq_m_f16\000vsubq_m_f3"
    "2\000vsubq_m_s16\000vsubq_m_s32\000vsubq_m_s8\000vsubq_m_u16\000vsubq_m"
    "_u32\000vsubq_m_u8\000vsubq_s16\000vsubq_s32\000vsubq_s8\000vsubq_u16\000"
    "vsubq_u32\000vsubq_u8\000vsubq_x\000vsubq_x_f16\000vsubq_x_f32\000vsubq"
    "_x_s16\000vsubq_x_s32\000vsubq_x_s8\000vsubq_x_u16\000vsubq_x_u32\000vs"
    "ubq_x_u8\000vuninitializedq_f16\000vuninitializedq_f32\000vuninitialize"
    "dq\000vuninitializedq_polymorphic_f16\000vuninitializedq_polymorphic_f3"
    "2\000vuninitializedq_polymorphic_s16\000vuninitializedq_polymorphic_s32"
    "\000vuninitializedq_polymorphic_s64\000vuninitializedq_polymorphic_s8\000"
    "vuninitializedq_polymorphic_u16\000vuninitializedq_polymorphic_u32\000v"
    "uninitializedq_polymorphic_u64\000vuninitializedq_polymorphic_u8\000vun"
    "initializedq_s16\000vuninitializedq_s32\000vuninitializedq_s64\000vunin"
    "itializedq_s8\000vuninitializedq_u16\000vuninitializedq_u32\000vuniniti"
    "alizedq_u64\000vuninitializedq_u8\000"};

auto It = std::lower_bound(std::begin(Map), std::end(Map), BuiltinID,
  [](const IntrinToName &L, unsigned Id) {
    return L.Id < Id;
  });
if (It == std::end(Map) || It->Id != BuiltinID)
  return false;
StringRef FullName(&IntrinNames[It->FullName]);
if (AliasName == FullName)
  return true;
if (It->ShortName == -1)
  return false;
StringRef ShortName(&IntrinNames[It->ShortName]);
return AliasName == ShortName;
