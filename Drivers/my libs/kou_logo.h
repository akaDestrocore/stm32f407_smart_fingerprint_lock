#ifndef KOU_LOGO_H_
#define KOU_LOGO_H_

#include "stdint.h"

//for the image bitmaps I used https://retro-esp32.github.io/Convert-Image-To-Byte-Array/ web-site

const uint16_t koulogo[50][50] = {
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,61343,50815,46591,52895,50847,38270,31965,34015,38207,42399,52927,63455,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,55039,40287,19326,4638,383,383,351,351,383,383,351,351,383,383,10910,25598,40319,55039,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,46623,13118,4606,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,4606,17246,52927,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,40350,8798,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,10879,40382,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,55071,17214,2463,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,17214,55039,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,40319,383,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,38206,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,30012,2526,351,383,351,351,351,351,351,351,383,383,383,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,383,351,2526,29980,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,21375,446,351,2431,15070,2494,351,351,351,351,2463,19295,27711,17182,2494,351,351,351,351,351,351,2495,19263,27743,19262,2462,351,351,351,351,2494,12990,383,351,2494,19295,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,25694,351,351,351,17151,65535,44509,4542,351,351,383,42431,65535,65535,65535,44541,383,351,351,351,351,383,44542,65535,65535,65535,42431,383,351,351,4574,44510,65535,15102,351,351,351,27900,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,38239,2495,351,351,351,50783,65535,65535,23518,350,351,23487,65535,65535,65535,65535,65535,19263,351,351,351,351,19263,65535,65535,65535,65535,65535,21406,351,2430,27677,65535,65535,48735,351,351,351,2558,38238,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,55007,2463,351,351,383,17182,65535,65535,65535,46653,4605,351,36095,65535,65535,65535,65535,65535,27711,351,351,351,351,27711,65535,65535,65535,65535,65535,36094,351,6685,46652,65535,65535,65535,17182,383,351,351,2463,55007,65535,65535,65535,65535},
	{65535,65535,65535,65535,17214,351,351,351,4574,25659,65535,65535,65535,29851,4573,351,19295,65535,65535,65535,65535,65535,15071,351,351,351,351,15071,65535,65535,65535,65535,65535,19295,351,4574,29851,65535,65535,65535,25659,4574,351,351,351,17214,63455,65535,65535,65535},
	{65535,65535,65535,38238,383,351,351,351,6686,42428,65535,65535,65535,42462,4606,351,383,40287,65535,65535,65535,33983,351,351,351,351,351,351,34014,65535,65535,65535,40287,351,351,4574,42430,65535,65535,65535,42428,6686,351,351,351,2431,38238,65535,65535,65535},
	{65535,65535,65535,13022,351,351,351,351,2430,21374,65535,65535,50846,17182,351,351,351,17151,65535,65535,65535,10878,351,351,351,351,351,351,10910,65535,65535,65535,17151,351,351,351,17182,50846,65535,65535,21374,2462,351,351,351,351,12990,61343,65535,65535},
	{65535,65535,38207,351,351,351,351,351,351,4575,55007,65535,42430,2462,351,351,351,25599,65535,65535,65535,23518,351,351,351,351,351,351,23519,65535,65535,65535,25567,351,351,351,2462,42430,65535,55007,4575,351,351,351,351,351,351,36095,65535,65535},
	{65535,65535,15102,351,351,351,351,351,351,6655,57119,65535,65535,40319,4575,4606,27677,65535,65535,65535,65535,65535,21374,383,351,351,351,23487,65535,65535,65535,65535,65535,25629,4573,4543,40319,65535,65535,57119,6655,351,351,351,351,351,351,15102,65535,65535},
	{65535,46654,2494,351,351,351,351,351,351,6655,57151,65535,65535,65535,65535,63423,65535,65535,65535,65535,65535,65535,65535,57119,55039,55039,57119,65535,65535,65535,65535,65535,65535,65535,63423,65535,65535,65535,65535,57151,6655,351,351,351,351,351,351,4542,48702,65535},
	{65535,29823,351,351,351,351,351,351,351,6655,59231,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,59231,6655,351,351,351,351,351,351,351,31903,65535},
	{65535,17182,351,351,351,351,351,351,351,4543,36095,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,36095,4543,351,351,351,351,351,351,351,17183,65535},
	{48734,6686,351,351,351,351,351,351,351,351,6623,12991,15071,15071,15039,19294,44510,65535,65535,65535,65535,65535,36095,4575,2463,2463,4575,34015,65535,65535,65535,65535,65535,44478,19326,15039,15071,15071,12991,6655,351,351,351,351,351,351,351,351,8798,48734},
	{36127,383,351,351,351,351,351,351,351,351,351,351,351,351,351,351,2462,36157,65535,65535,65535,29791,351,351,351,351,351,351,29791,65535,65535,65535,34046,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,38207},
	{27743,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,17182,65535,65535,65535,12990,351,351,351,351,351,351,12990,65535,65535,65535,15039,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,29791},
	{27679,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,17214,65535,65535,65535,10910,351,351,351,351,351,351,10910,65535,65535,65535,17214,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,27647},
	{25599,351,351,351,351,351,351,351,351,351,351,351,351,351,351,383,8766,46622,65535,65535,65535,34015,2463,383,351,351,383,2494,36095,65535,65535,65535,44542,6654,383,351,351,351,351,351,351,351,351,351,351,351,351,351,351,27679},
	{21470,351,351,351,351,351,351,351,351,351,351,351,351,351,351,15071,46654,65535,65535,65535,65535,65535,27743,6750,383,383,6782,29949,65535,65535,65535,65535,65535,40318,15070,351,351,351,351,351,351,351,351,351,351,351,351,351,351,27679},
	{21470,351,351,351,351,351,351,351,351,351,351,351,351,351,4543,65535,65535,65535,65535,65535,65535,65535,65535,59263,8862,8863,59263,65535,65535,65535,65535,65535,65535,65535,65535,4543,351,351,351,351,351,351,351,351,351,351,351,351,351,25599},
	{25599,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,65535,65535,65535,65535,65535,65535,65535,65535,65535,8767,8767,65535,65535,65535,65535,65535,65535,65535,65535,65535,2463,351,351,351,351,351,351,351,351,351,351,351,351,351,27679},
	{27679,351,351,351,351,351,351,351,351,351,351,351,351,351,351,65535,65535,65535,65535,65535,65535,65535,65535,46655,4575,4638,48861,65535,65535,65535,65535,65535,65535,65535,63455,351,351,351,351,351,351,351,351,351,351,351,351,351,351,27679},
	{29791,351,351,351,351,351,351,351,351,351,351,351,351,351,351,48734,65535,65535,65535,65535,65535,65535,65535,30013,2494,2526,34173,65535,65535,65535,65535,65535,65535,65535,44479,351,351,351,351,351,351,351,351,351,351,351,351,351,351,29791},
	{36159,2463,351,351,351,351,351,351,351,351,351,351,351,351,351,29821,65535,65535,65535,65535,65535,65535,65535,15229,383,383,17278,65535,65535,65535,65535,65535,65535,65535,23486,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,38207},
	{48767,8798,351,351,351,351,351,351,351,351,351,351,351,351,351,10878,65535,65535,65535,65535,65535,65535,65535,8798,351,351,8799,65535,65535,65535,65535,65535,65535,65535,8734,351,351,351,351,351,351,351,351,351,351,351,351,351,8798,50847},
	{65535,19295,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,61343,65535,65535,65535,65535,65535,61311,4607,351,351,6687,61375,65535,65535,65535,65535,65535,59263,2463,351,351,351,351,351,351,351,351,351,351,351,351,351,19326,65535},
	{65535,31903,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,46590,65535,65535,65535,65535,65535,44478,4543,351,351,4575,48702,65535,65535,65535,65535,65535,46590,2431,351,351,351,351,351,351,351,351,351,351,351,351,351,36095,65535},
	{65535,50847,6654,351,351,351,351,351,351,351,351,351,351,351,351,383,31902,65535,65535,65535,65535,65535,23455,383,351,351,2463,27710,65535,65535,65535,65535,65535,31902,383,351,351,351,351,351,351,351,351,351,351,351,351,6686,52959,65535},
	{65535,65535,19295,351,351,351,351,351,351,351,351,351,351,351,351,351,17150,65535,65535,65535,65535,65535,6687,351,351,351,351,10909,65535,65535,65535,65535,65535,15134,351,351,351,351,351,351,351,351,351,351,351,351,383,23487,65535,65535},
	{65535,65535,44479,4543,351,351,351,351,351,351,351,351,351,351,351,351,4543,61343,65535,65535,65535,65535,6655,351,351,351,351,6655,65535,65535,65535,65535,61343,4542,351,351,351,351,351,351,351,351,351,351,351,351,4575,48735,65535,65535},
	{65535,65535,65535,17215,351,351,351,351,351,351,351,351,351,351,351,351,2463,48703,65535,65535,65535,52927,4575,351,351,351,351,4575,52959,65535,65535,65535,48702,2463,351,351,351,351,351,351,351,351,351,351,351,351,19295,65535,65535,65535},
	{65535,65535,65535,44542,4575,351,351,351,351,351,351,351,351,351,351,351,2431,33983,65535,65535,65535,31902,2463,351,351,351,351,2463,31934,65535,65535,65535,31934,2431,351,351,351,351,351,351,351,351,351,351,351,4575,44543,65535,65535,65535},
	{65535,65535,65535,65535,25630,351,351,351,351,351,351,351,351,351,351,351,351,15071,65535,65535,65535,17214,383,351,351,351,351,383,17214,65535,65535,65535,15071,351,351,351,351,351,351,351,351,351,351,351,382,27710,65535,65535,65535,65535},
	{65535,65535,65535,65535,63423,8799,383,351,351,351,351,351,351,351,351,351,351,4574,55039,65535,59230,8766,351,351,351,351,351,351,8766,59198,65535,55006,4574,351,351,351,351,351,351,351,351,351,351,2431,10942,63455,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,52927,10910,351,351,351,351,351,351,351,351,351,351,351,44510,65535,42398,4575,351,351,351,351,351,351,4575,40349,65535,42430,351,351,351,351,351,351,351,351,351,351,383,12990,57119,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,42493,4575,351,351,351,351,351,351,351,351,351,351,31902,52894,27677,2463,351,351,351,351,351,351,2463,27677,48765,29822,351,351,351,351,351,351,351,351,351,383,6718,42557,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,38239,8798,383,351,351,351,351,351,351,351,351,8798,15102,6686,351,351,351,351,351,351,351,351,6654,15102,8798,351,351,351,351,351,351,351,351,383,8830,42556,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,44669,13022,383,351,351,351,351,351,351,351,383,2431,351,351,351,351,351,351,351,351,351,351,2431,383,351,351,351,351,351,351,351,2463,15071,46718,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,57151,8893,383,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,383,8893,61343,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,31935,8799,2463,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,2463,8862,34141,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,57151,25630,6718,2431,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,351,2431,8798,25725,57150,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,61343,27774,8830,382,351,351,351,351,351,351,351,351,351,351,351,351,351,351,382,8830,27774,61343,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,50878,32029,17309,6687,4574,2463,2431,2431,2431,2431,2463,4575,6687,17309,32029,50878,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,63423,50847,48703,46591,40319,40319,46591,48703,50847,63423,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
};

const uint16_t fingerprintIMG[40][40] = {
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65465,65359,65289,65253,65253,65216,65216,65216,65216,65253,65288,65358,65429,65533,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65465,65289,65216,65216,65218,65254,65289,65289,65289,65289,65289,65254,65219,65216,65216,65217,65289,65429,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65218,65216,65323,65464,65535,65535,65535,65535,65535,65535,65535,65535,65535,65498,65395,65322,65217,65216,65463,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65465,65498,65535,65533,65463,65394,65358,65358,65358,65358,65359,65428,65466,65534,65535,65535,65501,65462,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65428,65289,65217,65216,65216,65216,65216,65216,65216,65216,65216,65216,65218,65323,65429,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65427,65252,65216,65217,65321,65393,65463,65499,65535,65535,65500,65498,65429,65360,65287,65217,65216,65252,65393,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65501,65288,65216,65251,65393,65533,65535,65535,65535,65533,65465,65430,65430,65463,65499,65534,65535,65500,65392,65251,65216,65252,65464,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65500,65252,65216,65357,65533,65535,65535,65499,65360,65253,65216,65216,65216,65216,65216,65216,65217,65288,65395,65534,65534,65393,65217,65216,65395,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65286,65217,65428,65535,65535,65500,65323,65216,65216,65251,65323,65393,65394,65428,65393,65359,65289,65217,65216,65251,65395,65535,65498,65253,65216,65429,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65500,65218,65394,65535,65535,65428,65218,65216,65288,65463,65534,65535,65535,65535,65535,65535,65535,65535,65533,65393,65251,65216,65322,65534,65500,65252,65393,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65394,65216,65251,65430,65535,65534,65429,65324,65254,65217,65217,65253,65322,65395,65500,65535,65501,65322,65216,65288,65534,65535,65498,65218,65464,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65394,65216,65253,65500,65535,65428,65252,65216,65216,65253,65289,65289,65286,65217,65216,65218,65359,65534,65535,65323,65216,65323,65535,65500,65217,65286,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65464,65217,65253,65500,65534,65323,65216,65251,65393,65500,65535,65535,65535,65535,65534,65428,65286,65216,65253,65499,65535,65322,65216,65428,65535,65358,65216,65430,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65534,65252,65218,65499,65534,65289,65216,65323,65533,65535,65499,65394,65358,65358,65392,65463,65535,65535,65394,65216,65252,65499,65533,65252,65217,65500,65500,65217,65288,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65393,65216,65394,65535,65357,65216,65324,65535,65501,65324,65216,65216,65216,65216,65216,65216,65254,65464,65535,65428,65216,65253,65534,65430,65216,65322,65535,65358,65253,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65534,65251,65252,65534,65464,65216,65289,65535,65501,65254,65216,65289,65463,65501,65535,65465,65359,65218,65216,65428,65535,65394,65216,65359,65535,65323,65216,65429,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65463,65216,65392,65535,65288,65216,65498,65534,65286,65217,65428,65535,65535,65501,65499,65535,65535,65498,65252,65216,65464,65535,65287,65218,65500,65500,65219,65218,65499,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65357,65216,65499,65499,65216,65323,65535,65394,65216,65392,65535,65498,65288,65216,65216,65251,65394,65535,65498,65218,65253,65535,65429,65216,65358,65535,65430,65216,65251,65429,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65287,65219,65535,65427,65216,65429,65535,65252,65251,65535,65499,65218,65251,65393,65427,65287,65216,65393,65535,65359,65216,65463,65534,65254,65216,65357,65534,65430,65251,65251,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65219,65288,65535,65358,65216,65500,65499,65216,65358,65535,65323,65217,65499,65535,65535,65534,65287,65217,65500,65499,65216,65289,65535,65501,65289,65216,65288,65500,65534,65500,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65216,65289,65535,65322,65216,65535,65430,65216,65393,65535,65252,65289,65535,65359,65252,65535,65430,65216,65392,65535,65324,65216,65358,65535,65535,65358,65216,65253,65498,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65359,65430,65535,65324,65216,65535,65463,65216,65393,65535,65252,65287,65535,65360,65216,65429,65535,65254,65217,65498,65534,65287,65216,65358,65535,65535,65395,65217,65218,65463,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65358,65216,65465,65500,65216,65357,65535,65322,65217,65500,65499,65217,65287,65535,65464,65217,65253,65534,65501,65254,65216,65324,65535,65535,65464,65218,65252,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65393,65463,65535,65429,65216,65427,65535,65218,65288,65535,65463,65216,65358,65535,65358,65216,65395,65535,65392,65216,65288,65534,65534,65289,65216,65289,65533,65535,65500,65498,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65218,65288,65535,65499,65216,65357,65535,65288,65218,65534,65535,65287,65217,65498,65534,65252,65217,65465,65535,65357,65216,65288,65534,65534,65357,65216,65253,65498,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65322,65217,65500,65535,65218,65288,65535,65358,65216,65501,65535,65465,65216,65288,65534,65465,65217,65219,65498,65535,65357,65216,65288,65534,65535,65394,65217,65219,65463,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65430,65216,65393,65535,65218,65287,65535,65393,65216,65464,65535,65535,65359,65216,65359,65535,65430,65217,65253,65498,65535,65357,65216,65253,65500,65535,65462,65218,65217,65500,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65394,65500,65500,65216,65325,65535,65428,65216,65429,65535,65535,65535,65288,65216,65395,65535,65395,65217,65219,65466,65535,65392,65216,65252,65466,65535,65499,65393,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65359,65216,65430,65535,65428,65216,65430,65358,65430,65535,65500,65253,65216,65463,65535,65430,65217,65217,65429,65535,65429,65217,65217,65429,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65534,65393,65216,65288,65535,65535,65395,65216,65464,65218,65288,65535,65535,65499,65218,65217,65463,65535,65465,65251,65216,65359,65534,65465,65252,65216,65395,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65499,65252,65216,65253,65500,65535,65535,65323,65216,65500,65253,65252,65535,65535,65535,65465,65218,65217,65395,65535,65500,65287,65216,65288,65501,65500,65322,65395,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65499,65288,65393,65534,65535,65535,65499,65217,65287,65535,65253,65253,65535,65535,65535,65535,65499,65253,65216,65393,65535,65534,65358,65216,65252,65466,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65430,65252,65216,65464,65534,65217,65322,65535,65535,65535,65535,65535,65500,65286,65216,65324,65534,65535,65427,65217,65289,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65463,65253,65216,65218,65428,65535,65463,65216,65393,65535,65499,65358,65359,65465,65535,65534,65322,65216,65286,65500,65535,65498,65498,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65360,65251,65359,65499,65535,65535,65322,65217,65499,65500,65251,65216,65216,65216,65323,65534,65535,65392,65216,65252,65466,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65430,65216,65323,65535,65359,65216,65429,65500,65322,65216,65321,65535,65535,65429,65219,65429,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65393,65217,65219,65501,65535,65252,65253,65535,65535,65535,65322,65216,65394,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65463,65216,65252,65465,65535,65500,65216,65325,65535,65535,65535,65533,65253,65218,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65463,65534,65535,65535,65430,65216,65394,65535,65535,65535,65535,65500,65498,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},
	{65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65465,65218,65465,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535},

};



#endif /* KOU_LOGO_H_ */
