#include <stdint.h>
#include <stdlib.h>

// const int lookup_table[20] = {21000, 22000, 23000, 24000, 25000, 26000, 27000, 28000, 29000, 10000, 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000};
// const int table_size = sizeof(lookup_table)/sizeof(lookup_table[0]);


// // Signal up to 35 Hz
// const int lookup_table[1000] = {57039, 54170, 51626, 54714, 52068, 49794, 54129, 53105, 51796, 55242, 52176, 51130, 57120, 54327, 49491, 52614, 52715, 51494, 53028, 51954, 53068, 53847, 50143, 52215, 52973, 46587, 50426, 56825, 51964, 52419, 55713, 48796, 47122, 52545, 52277, 51768, 51032, 47899, 49978, 50864, 48141, 51798, 53980, 49734, 48775, 47909, 47003, 52867, 55412, 51346, 51307, 52647, 50952, 48980, 47926, 52635, 57883, 51294, 44696, 49739, 53370, 52556, 53360, 50740, 49546, 55082, 54426, 47997, 49496, 54199, 54003, 52461, 51772, 52738, 53671, 51254, 51696, 56188, 55021, 52270, 54654, 53441, 49308, 50420, 50293, 47384, 49607, 52640, 52207, 52927, 53414, 51791, 51258, 51883, 52839, 52536, 49973, 50540, 53699, 52269, 49803, 51581, 55183, 58692, 56730, 49530, 50087, 54135, 48654, 47187, 55279, 52930, 45874, 52503, 57994, 52065, 51552, 56766, 54370, 49210, 48288, 48810, 51796, 56373, 56309, 53628, 53110, 51801, 48516, 47046, 48965, 53437, 56590, 54287, 50066, 48884, 49497, 50906, 52948, 53638, 53608, 53200, 51015, 50494, 53749, 55466, 53662, 53068, 55083, 55007, 50479, 47195, 49593, 52690, 52117, 50264, 50246, 52813, 53955, 50846, 49886, 52203, 50490, 48507, 50962, 51262, 50551, 53162, 53211, 51692, 53282, 52193, 49603, 53016, 56084, 52087, 49324, 53902, 56566, 50736, 47731, 52404, 52081, 48857, 52179, 51438, 46458, 50513, 54110, 50944, 51714, 50520, 46121, 51822, 56343, 49916, 48677, 51639, 49529, 52743, 58933, 55000, 48681, 49473, 53220, 55764, 55135, 51700, 48730, 47977, 50733, 55370, 56458, 53157, 50476, 52508, 55392, 52346, 49121, 52254, 52961, 49388, 50741, 53691, 52879, 52003, 50911, 50021, 52016, 53454, 53434, 53431, 52229, 53054, 55094, 53212, 53397, 56711, 53486, 49591, 53260, 53857, 48173, 47530, 51347, 51152, 49433, 52002, 55865, 55995, 53355, 51081, 51212, 53388, 53945, 52609, 51738, 50360, 51184, 54867, 52613, 47887, 50416, 51107, 48470, 54009, 56453, 48659, 48982, 54429, 50707, 50273, 55971, 53625, 51159, 56421, 57579, 53741, 51129, 48483, 49831, 53435, 50237, 48708, 54095, 53154, 49632, 52994, 51037, 44624, 47561, 51972, 53160, 56992, 54420, 46888, 48452, 51341, 48862, 49405, 50918, 51112, 51954, 49942, 51474, 57967, 55509, 50023, 53416, 55227, 51553, 49861, 50726, 53111, 52378, 49386, 53479, 57486, 55712, 57102, 54016, 45699, 50069, 56046, 50773, 50167, 52096, 47755, 47675, 48789, 48267, 54887, 55921, 46143, 45739, 52807, 52484, 50415, 53468, 57388, 56898, 53330, 53042, 54773, 55526, 57469, 56494, 52792, 53577, 54533, 53077, 52851, 50542, 50382, 54099, 50260, 47288, 55142, 56616, 49742, 48512, 49495, 51291, 53831, 49656, 49127, 58435, 59641, 51678, 49131, 50316, 50228, 49747, 48395, 49211, 54088, 56551, 52900, 50021, 51655, 50982, 48897, 51532, 51406, 48100, 52117, 54993, 50177, 49057, 50596, 51521, 56296, 56135, 50348, 50971, 52789, 51386, 52250, 53191, 54778, 54179, 46882, 48026, 56810, 53471, 49490, 54336, 51747, 47683, 52091, 54249, 54754, 55759, 53154, 53659, 54550, 50835, 53101, 56880, 51974, 47940, 48220, 49204, 52948, 54797, 53723, 54245, 53115, 52413, 54235, 51506, 47981, 48817, 48321, 47890, 48936, 47252, 47150, 50833, 53806, 53998, 49457, 46499, 51600, 53405, 48355, 47543, 48915, 49058, 51209, 50186, 47970, 52122, 54539, 52423, 54210, 53648, 46507, 47629, 57870, 59083, 52522, 52178, 51725, 46296, 50216, 60206, 58499, 51332, 50602, 52706, 55648, 56794, 51361, 46744, 49979, 54332, 52845, 50797, 54911, 54722, 44500, 44118, 54182, 52287, 46735, 50155, 50683, 51834, 57073, 52979, 49066, 52751, 48713, 45034, 50668, 49164, 46029, 54141, 58170, 52181, 47643, 46959, 48625, 49365, 46833, 46849, 49843, 51266, 52053, 52119, 51247, 50586, 50385, 53359, 55115, 50533, 50419, 55034, 51246, 48966, 56534, 57146, 51252, 53587, 55321, 47972, 44883, 50829, 52327, 47359, 48956, 53860, 49885, 47243, 52533, 50835, 45172, 48304, 50892, 49996, 53216, 53225, 50069, 53319, 55598, 51852, 49464, 49812, 53608, 57657, 52689, 46922, 50180, 51980, 50390, 51549, 48602, 44202, 47353, 49679, 49450, 54216, 54801, 48407, 49942, 57622, 57580, 52122, 50571, 51998, 49610, 45535, 49034, 55202, 52522, 48244, 51458, 54114, 52300, 52731, 54613, 52904, 51225, 52865, 51349, 47070, 50993, 57301, 52819, 47697, 49936, 48936, 46472, 48888, 49627, 49553, 51434, 51062, 53020, 55993, 51120, 48758, 55164, 55817, 50789, 51814, 55698, 56299, 54505, 51695, 49485, 49764, 52257, 52774, 50003, 49181, 49672, 50021, 54187, 53777, 46042, 49113, 59069, 57853, 54655, 55323, 52764, 53308, 54444, 49365, 49037, 51264, 48022, 49468, 52299, 48466, 49737, 55608, 55348, 52470, 50599, 52562, 58234, 56800, 51180, 51307, 51304, 51116, 53464, 50420, 48320, 54573, 56628, 53321, 54020, 53757, 50834, 52442, 55948, 54873, 52222, 53468, 54295, 50149, 48312, 50610, 50408, 51070, 53444, 50642, 47096, 48436, 51995, 55111, 53529, 51394, 55823, 55637, 48746, 49584, 51672, 47989, 49551, 53318, 53870, 56088, 53885, 48826, 52462, 56831, 54716, 52621, 51441, 52791, 56615, 55725, 52696, 51311, 49396, 49906, 51107, 52356, 58416, 57711, 48528, 51404, 58414, 53104, 51297, 55523, 51539, 48551, 51611, 49964, 46505, 47166, 49553, 51602, 50391, 48857, 52025, 53516, 51419, 53465, 57114, 55209, 50550, 49629, 53124, 55627, 55453, 56242, 55908, 51634, 47809, 47590, 50607, 53747, 52429, 50397, 51625, 50823, 49056, 51031, 52061, 51247, 51332, 49465, 48540, 51286, 51650, 50354, 50237, 48621, 49200, 51191, 48384, 48585, 52853, 50380, 49364, 54303, 51676, 48429, 52710, 49669, 43829, 48616, 51956, 51500, 56209, 56759, 52049, 52395, 53702, 52760, 51669, 50639, 54130, 54719, 45880, 46191, 54883, 50853, 46184, 51660, 52369, 53354, 59196, 56197, 53428, 57314, 51826, 45655, 51379, 52739, 48153, 50733, 53690, 51783, 52441, 54825, 53258, 49001, 47736, 50860, 53136, 53188, 53641, 53096, 51827, 51450, 49905, 48588, 50578, 54050, 56368, 56132, 54814, 54770, 53613, 52219, 53547, 52586, 50401, 53243, 54112, 51108, 53398, 55499, 52014, 52209, 54506, 52907, 53852, 55352, 50145, 46442, 50208, 54118, 54542, 52728, 49713, 50003, 54492, 56560, 54250, 52676, 53841, 54268, 50553, 46174, 48699, 54246, 52428, 48986, 53154, 56465, 53785, 53029, 53457, 51692, 52861, 53894, 48496, 44933, 49613, 54720, 55491, 55633, 55405, 52026, 48507, 49731, 52586, 51559, 49531, 51281, 53947, 53221, 50711, 50213, 52337, 54150, 53043, 51651, 53233, 53069, 48872, 49130, 53091, 50827, 49403, 56288, 57390, 48860, 45724, 49290, 51629, 52430, 51180, 49522, 51822, 54660, 54311, 52518, 50471, 50811, 54066, 54986, 53227, 53079, 53796, 52255, 49011, 48318, 51335, 53609, 54325, 54799, 53402, 53584, 55843, 52465, 48854, 53504, 55833, 51733, 50669, 50222, 48620, 51729, 52810, 48709, 50220, 55286, 54753, 50429, 45063, 44984, 54388, 56589, 46986, 48295, 56558, 50668, 45802, 53586, 54486, 50899, 55974, 55209, 46014, 44980, 48812, 50233, 51799, 51084, 51333, 55642, 54145, 48654, 49164, 50699, 50690, 53363, 54424, 51639, 48390, 46547, 49013, 52250, 49548, 46892, 50857, 56227, 58106, 57310, 55891, 53859, 50914, 51443, 55997, 57590, 56269, 56065, 54626, 52262, 52835, 55005, 54673, 51075, 48513, 48867, 49345, 51743, 54757, 52971, 53222};
// const int lookup_table_2[1000] = {8495, 11364, 13908, 10820, 13466, 15740, 11405, 12429, 13738, 10292, 13358, 14404, 8414, 11207, 16043, 12920, 12819, 14040, 12506, 13580, 12466, 11687, 15391, 13319, 12561, 18947, 15108, 8709, 13570, 13115, 9821, 16738, 18412, 12989, 13257, 13766, 14502, 17635, 15556, 14670, 17393, 13736, 11554, 15800, 16759, 17625, 18531, 12667, 10122, 14188, 14227, 12887, 14582, 16554, 17608, 12899, 7651, 14240, 20838, 15795, 12164, 12978, 12174, 14794, 15988, 10452, 11108, 17537, 16038, 11335, 11531, 13073, 13762, 12796, 11863, 14280, 13838, 9346, 10513, 13264, 10880, 12093, 16226, 15114, 15241, 18150, 15927, 12894, 13327, 12607, 12120, 13743, 14276, 13651, 12695, 12998, 15561, 14994, 11835, 13265, 15731, 13953, 10351, 6842, 8804, 16004, 15447, 11399, 16880, 18347, 10255, 12604, 19660, 13031, 7540, 13469, 13982, 8768, 11164, 16324, 17246, 16724, 13738, 9161, 9225, 11906, 12424, 13733, 17018, 18488, 16569, 12097, 8944, 11247, 15468, 16650, 16037, 14628, 12586, 11896, 11926, 12334, 14519, 15040, 11785, 10068, 11872, 12466, 10451, 10527, 15055, 18339, 15941, 12844, 13417, 15270, 15288, 12721, 11579, 14688, 15648, 13331, 15044, 17027, 14572, 14272, 14983, 12372, 12323, 13842, 12252, 13341, 15931, 12518, 9450, 13447, 16210, 11632, 8968, 14798, 17803, 13130, 13453, 16677, 13355, 14096, 19076, 15021, 11424, 14590, 13820, 15014, 19413, 13712, 9191, 15618, 16857, 13895, 16005, 12791, 6601, 10534, 16853, 16061, 12314, 9770, 10399, 13834, 16804, 17557, 14801, 10164, 9076, 12377, 15058, 13026, 10142, 13188, 16413, 13280, 12573, 16146, 14793, 11843, 12655, 13531, 14623, 15513, 13518, 12080, 12100, 12103, 13305, 12480, 10440, 12322, 12137, 8823, 12048, 15943, 12274, 11677, 17361, 18004, 14187, 14382, 16101, 13532, 9669, 9539, 12179, 14453, 14322, 12146, 11589, 12925, 13796, 15174, 14350, 10667, 12921, 17647, 15118, 14427, 17064, 11525, 9081, 16875, 16552, 11105, 14827, 15261, 9563, 11909, 14375, 9113, 7955, 11793, 14405, 17051, 15703, 12099, 15297, 16826, 11439, 12380, 15902, 12540, 14497, 20910, 17973, 13562, 12374, 8542, 11114, 18646, 17082, 14193, 16672, 16129, 14616, 14422, 13580, 15592, 14060, 7567, 10025, 15511, 12118, 10307, 13981, 15673, 14808, 12423, 13156, 16148, 12055, 8048, 9822, 8432, 11518, 19835, 15465, 9488, 14761, 15367, 13438, 17779, 17859, 16745, 17267, 10647, 9613, 19391, 19795, 12727, 13050, 15119, 12066, 8146, 8636, 12204, 12492, 10761, 10008, 8065, 9040, 12742, 11957, 11001, 12457, 12683, 14992, 15152, 11435, 15274, 18246, 10392, 8918, 15792, 17022, 16039, 14243, 11703, 15878, 16407, 7099, 5893, 13856, 16403, 15218, 15306, 15787, 17139, 16323, 11446, 8983, 12634, 15513, 13879, 14552, 16637, 14002, 14128, 17434, 13417, 10541, 15357, 16477, 14938, 14013, 9238, 9399, 15186, 14563, 12745, 14148, 13284, 12343, 10756, 11355, 18652, 17508, 8724, 12063, 16044, 11198, 13787, 17851, 13443, 11285, 10780, 9775, 12380, 11875, 10984, 14699, 12433, 8654, 13560, 17594, 17314, 16330, 12586, 10737, 11811, 11289, 12419, 13121, 11299, 14028, 17553, 16717, 17213, 17644, 16598, 18282, 18384, 14701, 11728, 11536, 16077, 19035, 13934, 12129, 17179, 17991, 16619, 16476, 14325, 15348, 17564, 13412, 10995, 13111, 11324, 11886, 19027, 17905, 7664, 6451, 13012, 13356, 13809, 19238, 15318, 5328, 7035, 14202, 14932, 12828, 9886, 8740, 14173, 18790, 15555, 11202, 12689, 14737, 10623, 10812, 21034, 21416, 11352, 13247, 18799, 15379, 14851, 13700, 8461, 12555, 16468, 12783, 16821, 20500, 14866, 16370, 19505, 11393, 7364, 13353, 17891, 18575, 16909, 16169, 18701, 18685, 15691, 14268, 13481, 13415, 14287, 14948, 15149, 12175, 10419, 15001, 15115, 10500, 14288, 16568, 9000, 8388, 14282, 11947, 10213, 17562, 20651, 14705, 13207, 18175, 16578, 11674, 15649, 18291, 13001, 14699, 20362, 17230, 14642, 15538, 12318, 12309, 15465, 12215, 9936, 13682, 16070, 15722, 11926, 7877, 12845, 18612, 15354, 13554, 15144, 13985, 16932, 21332, 18181, 15855, 16084, 11318, 10733, 17127, 15592, 7912, 7954, 13412, 14963, 13536, 15924, 19999, 16500, 10332, 13012, 17290, 14076, 11420, 13234, 12803, 10921, 12630, 14309, 12669, 14185, 18464, 14541, 8233, 12715, 17837, 15598, 16598, 19062, 16646, 15907, 15981, 14100, 14472, 12514, 9541, 14414, 16776, 10370, 9717, 14745, 13720, 9836, 9235, 11029, 13839, 16049, 15770, 13277, 12760, 15531, 16353, 15862, 15513, 11347, 11757, 19492, 16421, 6465, 7681, 10879, 10211, 12770, 12226, 11090, 16169, 16497, 14270, 17512, 16066, 13235, 17068, 15797, 9926, 10186, 13064, 14935, 12972, 7300, 8734, 14354, 14227, 14230, 14418, 12070, 15114, 17214, 10961, 8906, 12213, 11514, 11777, 14700, 13092, 9586, 10661, 13312, 12066, 11239, 15385, 17222, 14924, 15126, 14464, 12090, 14892, 18438, 17098, 13539, 10423, 12005, 14140, 9711, 9897, 16788, 15950, 13862, 17545, 15983, 12216, 11664, 9446, 11649, 16708, 13072, 8703, 10818, 12913, 14093, 12743, 8919, 9809, 12838, 14223, 16138, 15628, 14427, 13178, 7118, 7823, 17006, 14130, 7120, 12430, 14237, 10011, 13995, 16983, 13923, 15570, 19029, 18368, 15981, 13932, 15143, 16677, 13509, 12018, 14115, 12069, 8420, 10325, 14984, 15905, 12410, 9907, 10081, 9292, 9626, 13900, 17725, 17944, 14927, 11787, 13105, 15137, 13909, 14711, 16478, 14503, 13473, 14287, 14202, 16069, 16994, 14248, 13884, 15180, 15297, 16913, 16334, 14343, 17150, 16949, 12681, 15154, 16170, 11231, 13858, 17105, 12824, 15865, 21705, 16918, 13578, 14034, 9325, 8775, 13485, 13139, 11832, 12774, 13865, 14895, 11404, 10815, 19654, 19343, 10651, 14681, 19350, 13874, 13165, 12180, 6338, 9337, 12106, 8220, 13708, 19879, 14155, 12795, 17381, 14801, 11844, 13751, 13093, 10709, 12276, 16533, 17798, 14674, 12398, 12346, 11893, 12438, 13707, 14084, 15629, 16946, 14956, 11484, 9166, 9402, 10720, 10764, 11921, 13315, 11987, 12948, 15133, 12291, 11422, 14426, 12136, 10035, 13520, 13325, 11028, 12627, 11682, 10182, 15389, 19092, 15326, 11416, 10992, 12806, 15821, 15531, 11042, 8974, 11284, 12858, 11693, 11266, 14981, 19360, 16835, 11288, 13106, 16548, 12380, 9069, 11749, 12505, 12077, 13842, 12673, 11640, 17038, 20601, 15921, 10814, 10043, 9901, 10129, 13508, 17027, 15803, 12948, 13975, 16003, 14253, 11587, 12313, 14823, 15321, 13197, 11384, 12491, 13883, 12301, 12465, 16662, 16404, 12443, 14707, 16131, 9246, 8144, 16674, 19810, 16244, 13905, 13104, 14354, 16012, 13712, 10874, 11223, 13016, 15063, 14723, 11468, 10548, 12307, 12455, 11738, 13279, 16523, 17216, 14199, 11925, 11209, 10735, 12132, 11950, 9691, 13069, 16680, 12030, 9701, 13801, 14865, 15312, 16914, 13805, 12724, 16825, 15314, 10248, 10781, 15105, 20471, 20550, 11146, 8945, 18548, 17239, 8976, 14866, 19732, 11948, 11048, 14635, 9560, 10325, 19520, 20554, 16722, 15301, 13735, 14450, 14201, 9892, 11389, 16880, 16370, 14835, 14844, 12171, 11110, 13895, 17144, 18987, 16521, 13284, 15986, 18642, 14677, 9307, 7428, 8224, 9643, 11675, 14620, 14091, 9537, 7944, 9265, 9469, 10908, 13272, 12699, 10529, 10861, 14459, 17021, 16667, 16189, 13791, 10777, 12563, 12312};
 
// Signal up to 20 Hz
const int lookup_table[1000] = {60475, 60487, 55201, 49279, 47610, 50821, 55038, 56026, 53341, 50219, 49614, 50944, 51074, 48224, 44258, 42844, 45632, 50597, 54066, 54040, 51534, 49167, 48885, 50765, 53297, 54449, 52993, 49615, 46924, 47699, 52328, 57866, 60193, 57619, 52445, 48773, 48718, 50840, 52161, 51397, 50001, 50317, 53097, 56897, 59486, 59539, 57175, 53465, 49775, 47329, 46794, 47906, 49560, 50604, 50769, 50872, 51973, 54160, 56082, 55836, 52702, 48272, 45681, 47107, 51568, 55348, 55189, 51465, 48047, 48620, 53034, 57454, 58221, 55345, 52075, 51327, 53011, 54839, 55095, 54072, 53071, 52849, 53382, 54491, 55830, 56311, 54548, 50624, 47046, 46904, 50638, 55155, 56800, 55073, 52845, 52837, 54615, 55619, 54657, 53317, 53612, 55053, 55059, 52209, 48187, 46018, 46860, 49216, 51063, 51853, 52143, 52137, 51641, 51263, 52364, 55027, 56780, 54711, 49224, 44725, 45513, 51065, 56235, 56483, 52342, 48251, 47475, 49144, 50353, 50060, 49851, 51181, 53156, 53753, 52589, 51412, 51741, 52892, 52994, 51555, 50180, 50615, 52729, 54915, 56095, 56603, 57058, 57100, 55866, 53505, 51522, 51318, 52665, 53927, 53699, 52044, 50249, 49713, 51013, 53531, 55655, 55731, 53538, 50980, 50708, 53445, 56843, 57620, 55069, 51969, 51595, 54060, 56252, 55474, 52482, 50401, 50977, 52731, 53041, 51360, 49643, 49778, 51380, 52464, 51745, 49678, 47445, 45721, 44705, 44710, 45946, 47813, 49180, 49719, 50458, 52399, 54767, 55431, 53395, 50362, 49183, 50849, 53569, 54896, 54109, 52090, 49531, 46396, 43358, 42576, 45757, 51456, 55459, 54698, 50488, 47204, 47768, 51071, 53916, 54588, 53734, 52337, 50351, 47991, 47186, 50039, 55541, 59179, 57183, 51036, 46606, 48220, 54133, 58512, 57562, 52885, 48841, 47747, 48565, 49667, 51279, 54388, 58097, 59513, 56720, 51215, 46736, 45831, 47974, 50812, 52560, 52937, 52496, 51884, 51753, 52603, 54195, 55426, 55318, 54116, 52939, 52281, 51390, 49544, 47580, 47397, 49727, 52982, 54803, 54525, 53536, 53261, 53487, 53067, 51708, 50223, 49099, 47793, 46048, 45203, 47026, 51079, 54303, 54117, 51499, 49926, 51120, 52986, 52621, 50615, 50735, 54855, 59580, 59488, 53661, 47431, 46698, 51327, 55540, 54830, 50978, 49266, 51680, 54705, 54217, 50999, 49783, 53130, 58009, 59425, 56523, 53576, 54714, 58920, 61247, 58557, 52953, 49297, 50179, 53931, 57062, 57616, 56077, 54050, 52912, 53386, 55295, 57327, 57684, 55761, 53081, 51928, 52889, 54291, 54411, 53709, 54096, 56030, 57343, 55666, 51561, 48255, 48011, 49634, 50295, 49346, 49103, 51448, 54691, 55105, 51609, 47682, 47641, 51426, 54268, 52001, 46001, 41979, 43958, 50268, 55781, 57393, 56442, 55935, 56597, 56533, 54321, 51229, 49759, 50597, 51923, 51861, 50844, 50931, 53038, 55631, 56463, 55135, 53339, 52637, 52694, 52174, 50992, 50875, 53242, 56929, 58701, 56352, 51044, 46264, 44507, 45269, 46338, 46640, 47265, 49661, 53311, 55808, 55349, 52764, 50642, 50445, 50871, 49450, 45742, 42522, 43331, 48674, 54848, 56952, 53394, 47502, 44552, 46972, 52317, 55864, 55159, 52006, 50150, 51274, 53461, 53476, 50250, 45944, 43830, 45424, 49637, 54133, 57000, 57348, 55199, 51558, 48387, 47605, 49477, 52215, 53692, 53603, 53459, 54242, 54696, 52763, 48676, 45637, 46569, 50492, 53143, 51651, 47906, 46398, 48898, 52362, 52707, 49895, 48015, 50198, 54646, 56787, 54922, 52322, 53080, 56842, 58958, 55963, 49757, 45541, 46138, 49410, 51351, 50611, 49169, 49164, 50421, 51463, 51880, 52511, 53461, 53201, 50384, 46012, 42914, 42978, 45519, 48530, 50799, 52190, 52454, 51067, 48664, 47572, 49635, 53607, 55927, 54787, 52619, 53313, 57037, 59226, 55787, 48620, 44441, 47515, 54568, 58040, 54232, 47098, 43538, 45882, 50157, 51506, 49877, 49387, 52747, 57733, 59711, 56868, 52044, 49516, 50748, 53584, 54957, 53706, 50891, 48205, 46581, 46039, 46235, 46916, 48033, 49552, 51119, 51958, 51446, 50008, 49150, 50074, 52136, 53195, 51964, 49801, 49524, 52200, 55570, 56229, 53283, 49294, 47437, 48201, 49402, 49167, 48116, 48255, 50287, 52777, 53956, 53640, 52952, 52575, 52019, 50663, 48926, 47948, 48408, 50099, 52505, 55130, 57064, 56957, 54335, 50883, 49471, 51193, 53758, 53733, 50555, 47568, 48340, 52248, 54695, 52131, 46335, 42958, 45776, 52903, 58807, 59546, 55631, 50350, 46467, 44962, 45927, 49165, 53452, 56251, 55476, 51823, 48608, 48608, 51154, 52870, 51381, 47888, 45724, 46624, 48909, 49499, 47342, 44488, 43858, 46369, 50400, 53754, 55658, 56755, 57585, 57666, 56297, 53937, 52247, 52486, 53994, 54693, 53320, 51015, 50340, 52552, 56045, 57769, 56254, 53006, 50835, 51022, 52417, 53134, 52588, 51553, 50675, 49733, 48597, 48190, 49546, 51858, 52390, 49277, 44366, 42343, 46238, 53836, 59186, 58098, 52094, 46768, 46356, 50073, 53640, 53892, 51570, 49858, 50752, 53174, 54467, 53343, 51188, 50547, 52469, 55412, 56791, 55589, 53306, 52211, 52791, 53342, 52324, 50695, 51112, 54476, 57923, 57267, 51726, 45479, 43713, 47248, 51853, 53141, 51300, 50107, 51659, 53553, 52165, 47851, 45242, 47869, 53662, 57108, 55637, 52468, 52437, 56014, 58905, 57539, 53457, 51170, 52457, 54412, 53537, 50394, 48812, 50814, 53922, 54231, 51178, 47956, 47396, 48866, 49976, 50207, 51247, 53712, 55265, 53345, 49143, 47141, 50053, 55344, 57980, 56243, 53382, 53103, 54833, 54964, 52372, 50631, 53508, 59317, 61852, 57567, 50366, 47683, 52022, 57995, 58514, 52859, 47158, 46983, 51112, 53672, 51560, 47913, 47722, 51413, 54348, 52601, 47723, 44803, 46513, 50392, 52286, 51310, 50371, 52157, 55879, 58594, 58944, 58434, 58881, 59606, 58127, 53498, 48120, 45679, 47510, 51416, 54085, 54245, 53306, 53233, 54305, 55037, 53999, 51407, 48982, 48391, 49779, 51632, 52012, 50180, 47282, 45444, 45933, 48035, 49775, 49869, 48929, 48642, 49805, 51431, 51969, 51226, 50678, 51671, 53539, 53938, 51307, 46863, 43917, 45026, 49738, 55054, 57981, 57691, 55530, 53318, 51816, 50624, 49207, 47840, 47591, 49409, 53096, 57021, 58964, 57693, 54130, 50983, 50761, 53632, 57071, 57868, 54925, 50259, 47166, 47222, 49019, 49867, 48734, 47348, 48297, 52113, 56437, 58227, 56701, 53930, 52521, 52960, 53475, 52410, 50299, 49281, 50576, 53011, 54304, 53609, 52388, 52634, 54523, 56130, 55466, 52545, 49426, 48428, 50426, 54493, 58616, 60691, 59531, 55810, 52130, 51404, 54323, 58321, 59635, 56820, 52122, 49076, 48912, 49774, 49509, 48520, 49116, 52184, 55468, 55881, 53216, 50699, 51489, 55137, 58109, 57674, 54612, 51896, 51323, 52135, 52687, 52488, 52084, 51579, 50306, 48148, 46328, 46135, 47172, 47742, 47227, 47311, 50019, 54738, 58103, 57367, 53436, 49933, 49388, 51031, 52508, 52892, 53077, 53459, 52553, 48878, 43757, 41028, 43342, 49227, 54441, 56104, 54901, 53230, 52073, 50499, 47894, 45537, 45296, 47217, 49256, 49539, 48355, 47574, 48496, 50792, 53337, 55317, 56224, 55492, 53068, 50347, 49591, 51668, 54633, 55421, 53225, 50656, 50951, 54223, 57120, 56441, 52615, 49151, 48876, 51290, 53688, 54192, 53114, 51654, 50245, 48630, 47038, 46377, 47083, 48361, 49106, 49408, 50541, 53288, 56646, 58612, 58186, 56286, 54637, 54128, 54446, 54993, 55602, 56191, 56197, 54921, 52512, 50201, 49220, 49538, 49988, 49767, 49608, 51094, 54645, 58322, 59042, 55448, 49757, 46481, 48699, 55047};
const int lookup_table_2[1000] = {5059, 5047, 10333, 16255, 17924, 14713, 10496, 9508, 12193, 15315, 15920, 14590, 14460, 17310, 21276, 22690, 19902, 14937, 11468, 11494, 14000, 16367, 16649, 14769, 12237, 11085, 12541, 15919, 18610, 17835, 13206, 7668, 5341, 7915, 13089, 16761, 16816, 14694, 13373, 14137, 15533, 15217, 12437, 8637, 6048, 5995, 8359, 12069, 15759, 18205, 18740, 17628, 15974, 14930, 14765, 14662, 13561, 11374, 9452, 9698, 12832, 17262, 19853, 18427, 13966, 10186, 10345, 14069, 17487, 16914, 12500, 8080, 7313, 10189, 13459, 14207, 12523, 10695, 10439, 11462, 12463, 12685, 12152, 11043, 9704, 9223, 10986, 14910, 18488, 18630, 14896, 10379, 8734, 10461, 12689, 12697, 10919, 9915, 10877, 12217, 11922, 10481, 10475, 13325, 17347, 19516, 18674, 16318, 14471, 13681, 13391, 13397, 13893, 14271, 13170, 10507, 8754, 10823, 16310, 20809, 20021, 14469, 9299, 9051, 13192, 17283, 18059, 16390, 15181, 15474, 15683, 14353, 12378, 11781, 12945, 14122, 13793, 12642, 12540, 13979, 15354, 14919, 12805, 10619, 9439, 8931, 8476, 8434, 9668, 12029, 14012, 14216, 12869, 11607, 11835, 13490, 15285, 15821, 14521, 12003, 9879, 9803, 11996, 14554, 14826, 12089, 8691, 7914, 10465, 13565, 13939, 11474, 9282, 10060, 13052, 15133, 14557, 12803, 12493, 14174, 15891, 15756, 14154, 13070, 13789, 15856, 18089, 19813, 20829, 20824, 19588, 17721, 16354, 15815, 15076, 13135, 10767, 10103, 12139, 15172, 16351, 14685, 11965, 10638, 11425, 13444, 16003, 19138, 22176, 22958, 19777, 14078, 10075, 10836, 15046, 18330, 17766, 14463, 11618, 10946, 11800, 13197, 15183, 17543, 18348, 15495, 9993, 6355, 8351, 14498, 18928, 17314, 11401, 7022, 7972, 12649, 16693, 17787, 16969, 15867, 14255, 11146, 7437, 6021, 8814, 14319, 18798, 19703, 17560, 14722, 12974, 12597, 13038, 13650, 13781, 12931, 11339, 10108, 10216, 11418, 12595, 13253, 14144, 15990, 17954, 18137, 15807, 12552, 10731, 11009, 11998, 12273, 12047, 12467, 13826, 15311, 16435, 17741, 19486, 20331, 18508, 14455, 11231, 11417, 14035, 15608, 14414, 12548, 12913, 14919, 14799, 10679, 5954, 6046, 11873, 18103, 18836, 14207, 9994, 10704, 14556, 16268, 13854, 10829, 11317, 14535, 15751, 12404, 7525, 6109, 9011, 11958, 10820, 6614, 4287, 6977, 12581, 16237, 15355, 11603, 8472, 7918, 9457, 11484, 12622, 12148, 10239, 8207, 7850, 9773, 12453, 13606, 12645, 11243, 11123, 11825, 11438, 9504, 8191, 9868, 13973, 17279, 17523, 15900, 15239, 16188, 16431, 14086, 10843, 10429, 13925, 17852, 17893, 14108, 11266, 13533, 19533, 23555, 21576, 15266, 9753, 8141, 9092, 9599, 8937, 9001, 11213, 14305, 15775, 14937, 13611, 13673, 14690, 14603, 12496, 9903, 9071, 10399, 12195, 12897, 12840, 13360, 14542, 14659, 12292, 8605, 6833, 9182, 14490, 19270, 21027, 20265, 19196, 18894, 18269, 15873, 12223, 9726, 10185, 12770, 14892, 15089, 14663, 16084, 19792, 23012, 22203, 16860, 10686, 8582, 12140, 18032, 20982, 18562, 13217, 9670, 10375, 13528, 15384, 14260, 12073, 12058, 15284, 19590, 21704, 20110, 15897, 11401, 8534, 8186, 10335, 13976, 17147, 17929, 16057, 13319, 11842, 11931, 12075, 11292, 10838, 12771, 16858, 19897, 18965, 15042, 12391, 13883, 17628, 19136, 16636, 13172, 12827, 15639, 17519, 15336, 10888, 8747, 10612, 13212, 12454, 8692, 6576, 9571, 15777, 19993, 19396, 16124, 14183, 14923, 16365, 16370, 15113, 14071, 13654, 13023, 12073, 12333, 15150, 19522, 22620, 22556, 20015, 17004, 14735, 13344, 13080, 14467, 16870, 17962, 15899, 11927, 9607, 10747, 12915, 12221, 8497, 6308, 9747, 16914, 21093, 18019, 10966, 7494, 11302, 18436, 21996, 19652, 15377, 14028, 15657, 16147, 12787, 7801, 5823, 8666, 13490, 16018, 14786, 11950, 10577, 11828, 14643, 17329, 18953, 19495, 19299, 18618, 17501, 15982, 14415, 13576, 14088, 15526, 16384, 15460, 13398, 12339, 13570, 15733, 16010, 13334, 9964, 9305, 12251, 16240, 18097, 17333, 16132, 16367, 17418, 17279, 15247, 12757, 11578, 11894, 12582, 12959, 13515, 14871, 16608, 17586, 17126, 15435, 13029, 10404, 8470, 8577, 11199, 14651, 16063, 14341, 11776, 11801, 14979, 17966, 17194, 13286, 10839, 13403, 19199, 22576, 19758, 12631, 6727, 5988, 9903, 15184, 19067, 20572, 19607, 16369, 12082, 9283, 10058, 13711, 16926, 16926, 14380, 12664, 14153, 17646, 19810, 18910, 16625, 16035, 18192, 21046, 21676, 19165, 15134, 11780, 9876, 8779, 7949, 7868, 9237, 11597, 13287, 13048, 11540, 10841, 12214, 14519, 15194, 12982, 9489, 7765, 9280, 12528, 14699, 14512, 13117, 12400, 12946, 13981, 14859, 15801, 16937, 17344, 15988, 13676, 13144, 16257, 21168, 23191, 19296, 11698, 6348, 7436, 13440, 18766, 19178, 15461, 11894, 11642, 13964, 15676, 14782, 12360, 11067, 12191, 14346, 14987, 13065, 10122, 8743, 9945, 12228, 13323, 12743, 12192, 13210, 14839, 14422, 11058, 7611, 8267, 13808, 20055, 21821, 18286, 13681, 12393, 14234, 15427, 13875, 11981, 13369, 17683, 20292, 17665, 11872, 8426, 9897, 13066, 13097, 9520, 6629, 7995, 12077, 14364, 13077, 11122, 11997, 15140, 16722, 14720, 11612, 11303, 14356, 17578, 18138, 16668, 15558, 15327, 14287, 11822, 10269, 12189, 16391, 18393, 15481, 10190, 7554, 9291, 12152, 12431, 10701, 10570, 13162, 14903, 12026, 6217, 3682, 7967, 15168, 17851, 13512, 7539, 7020, 12675, 18376, 18551, 14422, 11862, 13974, 17621, 17812, 14121, 11186, 12933, 17811, 20731, 19021, 15142, 13248, 14224, 15163, 13377, 9655, 6940, 6590, 7100, 6653, 5928, 7407, 12036, 17414, 19855, 18024, 14118, 11449, 11289, 12228, 12301, 11229, 10497, 11535, 14127, 16552, 17143, 15755, 13902, 13522, 15354, 18252, 20090, 19601, 17499, 15759, 15665, 16605, 16892, 15729, 14103, 13565, 14308, 14856, 13863, 11995, 11596, 14227, 18671, 21617, 20508, 15796, 10480, 7553, 7843, 10004, 12216, 13718, 14910, 16327, 17694, 17943, 16125, 12438, 8513, 6570, 7841, 11404, 14551, 14773, 11902, 8463, 7666, 10609, 15275, 18368, 18312, 16515, 15667, 16800, 18186, 17237, 13421, 9097, 7307, 8833, 11604, 13013, 12574, 12059, 13124, 15235, 16253, 14958, 12523, 11230, 11925, 13146, 12900, 11011, 9404, 10068, 12989, 16108, 17106, 15108, 11041, 6918, 4843, 6003, 9724, 13404, 14130, 11211, 7213, 5899, 8714, 13412, 16458, 16622, 15760, 16025, 17014, 16418, 13350, 10066, 9653, 12318, 14835, 14045, 10397, 7425, 7860, 10922, 13638, 14211, 13399, 12847, 13046, 13450, 13955, 15228, 17386, 19206, 19399, 18362, 17792, 18307, 18223, 15515, 10796, 7431, 8167, 12098, 15601, 16146, 14503, 13026, 12642, 12457, 12075, 12981, 16656, 21777, 24506, 22192, 16307, 11093, 9430, 10633, 12304, 13461, 15035, 17640, 19997, 20238, 18317, 16278, 15995, 17179, 17960, 17038, 14742, 12197, 10217, 9310, 10042, 12466, 15187, 15943, 13866, 10901, 10113, 12309, 14878, 14583, 11311, 8414, 9093, 12919, 16383, 16658, 14244, 11846, 11342, 12420, 13880, 15289, 16904, 18496, 19157, 18451, 17173, 16428, 16126, 14993, 12246, 8888, 6922, 7348, 9248, 10897, 11406, 11088, 10541, 9932, 9343, 9337, 10613, 13022, 15333, 16314, 15996, 15546, 15767, 15926, 14440, 10889, 7212, 6492, 10086, 15777, 19053, 16835, 10487};

const float sin_table[200] = {0.00, 0.02, 0.03, 0.05, 0.06, 0.08, 0.09, 0.11, 0.12, 0.14, 0.15, 0.17, 0.18, 0.20, 0.21, 0.23, 0.24, 0.25, 0.27, 0.28, 0.29, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.39, 0.39, 0.40, 0.41, 0.42, 0.43, 0.44, 0.45, 0.45, 0.46, 0.46, 0.47, 0.48, 0.48, 0.48, 0.49, 0.49, 0.49, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.49, 0.49, 0.49, 0.48, 0.48, 0.48, 0.47, 0.47, 0.46, 0.45, 0.45, 0.44, 0.43, 0.42, 0.41, 0.40, 0.40, 0.39, 0.38, 0.36, 0.35, 0.34, 0.33, 0.32, 0.31, 0.29, 0.28, 0.27, 0.26, 0.24, 0.23, 0.21, 0.20, 0.18, 0.17, 0.16, 0.14, 0.13, 0.11, 0.09, 0.08, 0.06, 0.05, 0.03, 0.02, 0.00, -0.01, -0.03, -0.05, -0.06, -0.08, -0.09, -0.11, -0.12, -0.14, -0.15, -0.17, -0.18, -0.20, -0.21, -0.23, -0.24, -0.25, -0.27, -0.28, -0.29, -0.31, -0.32, -0.33, -0.34, -0.35, -0.36, -0.37, -0.38, -0.39, -0.40, -0.41, -0.42, -0.43, -0.44, -0.45, -0.45, -0.46, -0.46, -0.47, -0.48, -0.48, -0.48, -0.49, -0.49, -0.49, -0.50, -0.50, -0.50, -0.50, -0.50, -0.50, -0.50, -0.50, -0.50, -0.49, -0.49, -0.49, -0.48, -0.48, -0.48, -0.47, -0.47, -0.46, -0.45, -0.45, -0.44, -0.43, -0.42, -0.41, -0.41, -0.40, -0.39, -0.38, -0.37, -0.35, -0.34, -0.33, -0.32, -0.31, -0.30, -0.28, -0.27, -0.26, -0.24, -0.23, -0.21, -0.20, -0.19, -0.17, -0.16, -0.14, -0.13, -0.11, -0.10, -0.08, -0.06, -0.05, -0.03, -0.02};
// 1 wave in 200 points (If using control loop 100Hz then the wave will be with 0.5Hz)
const float sqrt_cos_table[200] = {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.99, 0.99, 0.99, 0.99, 0.98, 0.98, 0.98, 0.98, 0.97, 0.97, 0.96, 0.96, 0.95, 0.95, 0.94, 0.94, 0.93, 0.92, 0.92, 0.91, 0.90, 0.89, 0.88, 0.88, 0.87, 0.86, 0.84, 0.83, 0.82, 0.81, 0.79, 0.78, 0.76, 0.75, 0.73, 0.71, 0.68, 0.66, 0.63, 0.60, 0.55, 0.50, 0.42, 0.17, -0.42, -0.50, -0.55, -0.59, -0.63, -0.66, -0.68, -0.71, -0.73, -0.75, -0.76, -0.78, -0.79, -0.81, -0.82, -0.83, -0.84, -0.86, -0.87, -0.88, -0.88, -0.89, -0.90, -0.91, -0.92, -0.92, -0.93, -0.94, -0.94, -0.95, -0.95, -0.96, -0.96, -0.97, -0.97, -0.98, -0.98, -0.98, -0.98, -0.99, -0.99, -0.99, -0.99, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -0.99, -0.99, -0.99, -0.99, -0.99, -0.98, -0.98, -0.98, -0.97, -0.97, -0.96, -0.96, -0.95, -0.95, -0.94, -0.94, -0.93, -0.92, -0.92, -0.91, -0.90, -0.89, -0.89, -0.88, -0.87, -0.86, -0.85, -0.83, -0.82, -0.81, -0.79, -0.78, -0.76, -0.75, -0.73, -0.71, -0.69, -0.66, -0.63, -0.60, -0.56, -0.51, -0.43, -0.22, 0.41, 0.50, 0.55, 0.59, 0.63, 0.66, 0.68, 0.70, 0.73, 0.74, 0.76, 0.78, 0.79, 0.81, 0.82, 0.83, 0.84, 0.85, 0.87, 0.87, 0.88, 0.89, 0.90, 0.91, 0.92, 0.92, 0.93, 0.94, 0.94, 0.95, 0.95, 0.96, 0.96, 0.97, 0.97, 0.97, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00};

const float sqrt_sin_table[200] = {0.00, 0.42, 0.50, 0.55, 0.59, 0.63, 0.66, 0.68, 0.71, 0.73, 0.75, 0.76, 0.78, 0.79, 0.81, 0.82, 0.83, 0.84, 0.86, 0.87, 0.88, 0.88, 0.89, 0.90, 0.91, 0.92, 0.92, 0.93, 0.94, 0.94, 0.95, 0.95, 0.96, 0.96, 0.97, 0.97, 0.98, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.99, 0.99, 0.99, 0.99, 0.98, 0.98, 0.98, 0.98, 0.97, 0.97, 0.96, 0.96, 0.95, 0.95, 0.94, 0.94, 0.93, 0.92, 0.92, 0.91, 0.90, 0.89, 0.89, 0.88, 0.87, 0.86, 0.85, 0.83, 0.82, 0.81, 0.79, 0.78, 0.76, 0.75, 0.73, 0.71, 0.68, 0.66, 0.63, 0.60, 0.56, 0.50, 0.43, 0.20, -0.42, -0.50, -0.55, -0.59, -0.63, -0.66, -0.68, -0.70, -0.73, -0.74, -0.76, -0.78, -0.79, -0.81, -0.82, -0.83, -0.84, -0.85, -0.87, -0.88, -0.88, -0.89, -0.90, -0.91, -0.92, -0.92, -0.93, -0.94, -0.94, -0.95, -0.95, -0.96, -0.96, -0.97, -0.97, -0.98, -0.98, -0.98, -0.98, -0.99, -0.99, -0.99, -0.99, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -0.99, -0.99, -0.99, -0.99, -0.99, -0.98, -0.98, -0.98, -0.97, -0.97, -0.96, -0.96, -0.95, -0.95, -0.94, -0.94, -0.93, -0.92, -0.92, -0.91, -0.90, -0.89, -0.89, -0.88, -0.87, -0.86, -0.85, -0.83, -0.82, -0.81, -0.80, -0.78, -0.76, -0.75, -0.73, -0.71, -0.69, -0.66, -0.63, -0.60, -0.56, -0.51, -0.43};

static const float square_table[200] = {0.00, 0.12, 0.25, 0.38, 0.50, 0.62, 0.75, 0.88, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.88, 0.75, 0.62, 0.50, 0.38, 0.25, 0.12, 0.00, -0.12, -0.25, -0.38, -0.50, -0.62, -0.75, -0.88, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -0.88, -0.75, -0.62, -0.50, -0.38, -0.25, -0.12};

const int table_size = sizeof(sin_table)/sizeof(sin_table[0]);

