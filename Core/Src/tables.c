#include "tables.h"

const int16_t expMap[] =
{
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 3, 3, 3, 3, 3,
		3, 3, 4, 4, 4, 4, 4, 5,
		5, 5, 5, 6, 6, 6, 6, 7,
		7, 7, 8, 8, 9, 9, 9, 10,
		10, 11, 11, 12, 13, 13, 14, 15,
		15, 16, 17, 18, 19, 20, 21, 22,
		23, 24, 25, 26, 27, 29, 30, 32,
		33, 35, 37, 39, 41, 43, 45, 47,
		49, 52, 54, 57, 60, 63, 66, 69,
		73, 77, 80, 84, 89, 93, 98, 103,
		108, 113, 119, 125, 131, 137, 144, 151,
		159, 167, 175, 184, 193, 203, 213, 224,
		235, 246, 259, 272, 285, 300, 314, 330,
		347, 364, 382, 401, 421, 442, 464, 488

};

const int16_t blackmanHarris1024[] =
{
		1, 1, 2, 2, 2, 2, 2, 2,
		3, 3, 3, 4, 4, 4, 5, 5,
		6, 7, 7, 8, 9, 9, 10, 11,
		12, 13, 14, 15, 16, 17, 18, 20,
		21, 22, 24, 25, 27, 28, 30, 32,
		33, 35, 37, 39, 41, 43, 46, 48,
		50, 53, 55, 58, 60, 63, 66, 69,
		72, 75, 78, 82, 85, 89, 92, 96,
		100, 104, 108, 112, 116, 121, 125, 130,
		135, 140, 145, 150, 156, 161, 167, 173,
		179, 185, 191, 197, 204, 211, 218, 225,
		232, 240, 247, 255, 263, 272, 280, 289,
		298, 307, 316, 325, 335, 345, 355, 366,
		376, 387, 398, 410, 421, 433, 445, 458,
		470, 483, 496, 510, 524, 538, 552, 566,
		581, 597, 612, 628, 644, 660, 677, 694,
		712, 729, 748, 766, 785, 804, 823, 843,
		864, 884, 905, 926, 948, 970, 993, 1016,
		1039, 1063, 1087, 1111, 1136, 1162, 1187, 1214,
		1240, 1267, 1295, 1323, 1351, 1380, 1409, 1439,
		1469, 1500, 1531, 1563, 1595, 1628, 1661, 1695,
		1729, 1764, 1799, 1835, 1871, 1908, 1945, 1983,
		2021, 2060, 2100, 2140, 2180, 2222, 2263, 2306,
		2348, 2392, 2436, 2480, 2525, 2571, 2618, 2665,
		2712, 2760, 2809, 2858, 2908, 2959, 3010, 3062,
		3114, 3167, 3221, 3276, 3331, 3386, 3442, 3499,
		3557, 3615, 3674, 3734, 3794, 3855, 3916, 3979,
		4041, 4105, 4169, 4234, 4300, 4366, 4433, 4501,
		4569, 4638, 4708, 4778, 4849, 4921, 4994, 5067,
		5141, 5215, 5291, 5367, 5443, 5521, 5599, 5678,
		5757, 5838, 5918, 6000, 6082, 6166, 6249, 6334,
		6419, 6505, 6591, 6679, 6767, 6855, 6945, 7035,
		7126, 7217, 7309, 7402, 7496, 7590, 7685, 7780,
		7877, 7974, 8071, 8170, 8269, 8369, 8469, 8570,
		8672, 8774, 8877, 8981, 9085, 9190, 9296, 9402,
		9509, 9617, 9725, 9834, 9943, 10053, 10164, 10275,
		10387, 10499, 10612, 10726, 10840, 10955, 11070, 11186,
		11303, 11420, 11538, 11656, 11774, 11894, 12013, 12134,
		12255, 12376, 12498, 12620, 12743, 12866, 12990, 13114,
		13238, 13364, 13489, 13615, 13741, 13868, 13995, 14123,
		14251, 14379, 14508, 14637, 14767, 14896, 15027, 15157,
		15288, 15419, 15550, 15682, 15814, 15946, 16079, 16212,
		16345, 16478, 16611, 16745, 16879, 17013, 17147, 17282,
		17416, 17551, 17686, 17821, 17956, 18091, 18226, 18362,
		18497, 18633, 18768, 18904, 19040, 19175, 19311, 19447,
		19582, 19718, 19853, 19989, 20124, 20259, 20395, 20530,
		20665, 20800, 20934, 21069, 21203, 21338, 21472, 21605,
		21739, 21872, 22005, 22138, 22271, 22403, 22535, 22667,
		22798, 22929, 23060, 23190, 23320, 23450, 23579, 23708,
		23836, 23964, 24092, 24219, 24345, 24471, 24597, 24722,
		24846, 24970, 25093, 25216, 25339, 25460, 25581, 25702,
		25821, 25941, 26059, 26177, 26294, 26411, 26526, 26641,
		26756, 26869, 26982, 27094, 27205, 27316, 27426, 27534,
		27642, 27750, 27856, 27961, 28066, 28170, 28273, 28375,
		28476, 28576, 28675, 28773, 28870, 28966, 29062, 29156,
		29249, 29341, 29432, 29523, 29612, 29700, 29787, 29873,
		29957, 30041, 30124, 30205, 30285, 30365, 30443, 30520,
		30595, 30670, 30743, 30816, 30887, 30956, 31025, 31092,
		31158, 31223, 31287, 31349, 31410, 31470, 31529, 31586,
		31642, 31697, 31750, 31802, 31853, 31903, 31951, 31998,
		32043, 32087, 32130, 32172, 32212, 32250, 32288, 32324,
		32358, 32391, 32423, 32454, 32483, 32510, 32537, 32562,
		32585, 32607, 32628, 32647, 32665, 32681, 32696, 32710,
		32722, 32733, 32742, 32750, 32756, 32761, 32765, 32767,
		32767, 32767, 32765, 32761, 32756, 32750, 32742, 32733,
		32722, 32710, 32696, 32681, 32665, 32647, 32628, 32607,
		32585, 32562, 32537, 32510, 32483, 32454, 32423, 32391,
		32358, 32324, 32288, 32250, 32212, 32172, 32130, 32087,
		32043, 31998, 31951, 31903, 31853, 31802, 31750, 31697,
		31642, 31586, 31529, 31470, 31410, 31349, 31287, 31223,
		31158, 31092, 31025, 30956, 30887, 30816, 30743, 30670,
		30595, 30520, 30443, 30365, 30285, 30205, 30124, 30041,
		29957, 29873, 29787, 29700, 29612, 29523, 29432, 29341,
		29249, 29156, 29062, 28966, 28870, 28773, 28675, 28576,
		28476, 28375, 28273, 28170, 28066, 27961, 27856, 27750,
		27642, 27534, 27426, 27316, 27205, 27094, 26982, 26869,
		26756, 26641, 26526, 26411, 26294, 26177, 26059, 25941,
		25821, 25702, 25581, 25460, 25339, 25216, 25093, 24970,
		24846, 24722, 24597, 24471, 24345, 24219, 24092, 23964,
		23836, 23708, 23579, 23450, 23320, 23190, 23060, 22929,
		22798, 22667, 22535, 22403, 22271, 22138, 22005, 21872,
		21739, 21605, 21472, 21338, 21203, 21069, 20934, 20800,
		20665, 20530, 20395, 20259, 20124, 19989, 19853, 19718,
		19582, 19447, 19311, 19175, 19040, 18904, 18768, 18633,
		18497, 18362, 18226, 18091, 17956, 17821, 17686, 17551,
		17416, 17282, 17147, 17013, 16879, 16745, 16611, 16478,
		16345, 16212, 16079, 15946, 15814, 15682, 15550, 15419,
		15288, 15157, 15027, 14896, 14767, 14637, 14508, 14379,
		14251, 14123, 13995, 13868, 13741, 13615, 13489, 13364,
		13238, 13114, 12990, 12866, 12743, 12620, 12498, 12376,
		12255, 12134, 12013, 11894, 11774, 11656, 11538, 11420,
		11303, 11186, 11070, 10955, 10840, 10726, 10612, 10499,
		10387, 10275, 10164, 10053, 9943, 9834, 9725, 9617,
		9509, 9402, 9296, 9190, 9085, 8981, 8877, 8774,
		8672, 8570, 8469, 8369, 8269, 8170, 8071, 7974,
		7877, 7780, 7685, 7590, 7496, 7402, 7309, 7217,
		7126, 7035, 6945, 6855, 6767, 6679, 6591, 6505,
		6419, 6334, 6249, 6166, 6082, 6000, 5918, 5838,
		5757, 5678, 5599, 5521, 5443, 5367, 5291, 5215,
		5141, 5067, 4994, 4921, 4849, 4778, 4708, 4638,
		4569, 4501, 4433, 4366, 4300, 4234, 4169, 4105,
		4041, 3979, 3916, 3855, 3794, 3734, 3674, 3615,
		3557, 3499, 3442, 3386, 3331, 3276, 3221, 3167,
		3114, 3062, 3010, 2959, 2908, 2858, 2809, 2760,
		2712, 2665, 2618, 2571, 2525, 2480, 2436, 2392,
		2348, 2306, 2263, 2222, 2180, 2140, 2100, 2060,
		2021, 1983, 1945, 1908, 1871, 1835, 1799, 1764,
		1729, 1695, 1661, 1628, 1595, 1563, 1531, 1500,
		1469, 1439, 1409, 1380, 1351, 1323, 1295, 1267,
		1240, 1214, 1187, 1162, 1136, 1111, 1087, 1063,
		1039, 1016, 993, 970, 948, 926, 905, 884,
		864, 843, 823, 804, 785, 766, 748, 729,
		712, 694, 677, 660, 644, 628, 612, 597,
		581, 566, 552, 538, 524, 510, 496, 483,
		470, 458, 445, 433, 421, 410, 398, 387,
		376, 366, 355, 345, 335, 325, 316, 307,
		298, 289, 280, 272, 263, 255, 247, 240,
		232, 225, 218, 211, 204, 197, 191, 185,
		179, 173, 167, 161, 156, 150, 145, 140,
		135, 130, 125, 121, 116, 112, 108, 104,
		100, 96, 92, 89, 85, 82, 78, 75,
		72, 69, 66, 63, 60, 58, 55, 53,
		50, 48, 46, 43, 41, 39, 37, 35,
		33, 32, 30, 28, 27, 25, 24, 22,
		21, 20, 18, 17, 16, 15, 14, 13,
		12, 11, 10, 9, 9, 8, 7, 7,
		6, 5, 5, 4, 4, 4, 3, 3,
		3, 2, 2, 2, 2, 2, 2, 1
};
