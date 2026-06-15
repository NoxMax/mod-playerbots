/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

// ######################################################################################################################################### //
// To future developers: Wintergrasp is a non-linear PVP battle zone that deals with hundreds of bots with constantly changing objectives.
// Unlike the more linear Battlegrounds, a small change to BattlefieldTactics can have a radical effect on the balance of the match. Make
// sure you understand what you are changing, what the effects of your changes are, and to thoroughly test your changes.
// ######################################################################################################################################### //
#include "BattlefieldTactics.h"

#include "Battlefield.h"
#include "BattlefieldMgr.h"
#include "BattlefieldWG.h"
#include "GossipDef.h"
#include "PlayerbotAI.h"
#include "Playerbots.h"
#include "PositionValue.h"
#include "Vehicle.h"
#include "WorldPacket.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <atomic>
#include <mutex>
#include <queue>
#include <unordered_set>

// Snap radius: match a cannon creature to a known WG_DEFENDER_CANNON_POSITIONS entry
static constexpr float  WG_CANNON_SEARCH_RADIUS        = 5.0f;
// Base interval (ms) between WgMountTowerCannonAction scans, multiplied by level stagger
static constexpr uint32 WG_SCAN_INTERVAL               = 15000;
// Distance to Central Wall. Ensures defenders won't scan for cannons when they are far from the wall
static constexpr float  WG_CANNON_CENTRAL_WALL_RANGE   = 120.0f;
// If a defender scans for a cannon, it will only scan within this range of itself
static constexpr float  WG_OBJ_SCAN_RANGE              = 200.0f;
// Distance at which FollowWgRoute advances to the next A* waypoint (halved for vehicles)
static constexpr float  WG_NODE_SWITCH_DIST            = 20.0f;
// Bot must be within this distance of a noSkip waypoint before it can advance past it
static constexpr float  WG_NOSKIP_ARRIVE_DIST          = 5.0f;
// FollowWgRoute returns false (arrived) when bot is within this distance of the objective
static constexpr float  WG_OBJ_ARRIVE_DIST             = 10.0f;
// Workshop capture arrival latch triggers at this distance, enabling combat around the workshop
static constexpr float  WG_WORKSHOP_ARRIVE_DIST        = 80.0f;
// Attacker vehicle phases 1/3/5 advance when bot is within this distance of the staging point
static constexpr float  WG_STAGING_ARRIVE_DIST         = 10.0f;
// FindNearestGameObject range for locating a fortress wall GO to attack
static constexpr float  WG_WALL_SCAN_DIST              = 80.0f;
// Distance at which attacker vehicles begin casting melee spells at a wall
static constexpr float  WG_WALL_MELEE_DIST             = 10.0f;
// FindNearestGameObject range for locating a tower GO to attack
static constexpr float  WG_TOWER_SCAN_DIST             = 80.0f;
// Distance at which defender vehicles begin casting melee spells at a tower
// 35 yards melee range instead of 10 yards for attackers, because towers are not flat like walls
static constexpr float  WG_TOWER_MELEE_DIST            = 35.0f;
// Distance to fortress teleporters used to detect which workshop a defender vehicle came from
static constexpr float  FORTRESS_WS_DETECT_DIST        = 100.0f;
// FindNearestCreature range for locating the workshop engineer NPC to summon a vehicle
static constexpr float  ENGINEER_SCAN_RANGE            = 12.0f;

// ######################## //
// Wintergrasp Path Network
// ######################## //

// { Xf, Yf, Zf, #, bool}
// XYZ coordinates with # world state value of blocker to destroy, and bool for unskippable waypoints.
// World state values imported from the core at BattlefieldWG.h

// Ring Road North: The upper half of the Wintergrasp ring road
static WgPath const vPath_WG_Ring_Road_North = {
    { 4784.530f, 3291.680f, 365.614f },	         // Western connection to Ring Road South
    { 4879.030f, 3331.760f, 372.128f },
    { 4936.400f, 3333.610f, 376.881f },	         // Connection to NW Workshop
    { 4999.050f, 3309.470f, 376.573f },	         // Connection to Horde Route Part A from Horde Spawn
    { 5049.280f, 3228.850f, 358.029f },	         // Connection to Horde Route Part B to Fortress Wall West
    { 5056.060f, 3138.430f, 358.508f },
    { 5043.360f, 3077.020f, 366.621f },
    { 5051.850f, 3015.960f, 367.854f },
    { 5041.590f, 2950.240f, 378.512f },
    { 5051.660f, 2847.730f, 393.182f },	         // Connections to Outer Fortress Path and northern connection to Central Road
    { 5049.070f, 2772.040f, 381.497f },
    { 5011.900f, 2720.613f, 372.243f },
    { 5009.192f, 2670.899f, 363.185f },
    { 5023.000f, 2608.810f, 356.103f },
    { 5019.910f, 2540.720f, 345.466f },	         // Connections to Alliance Eastern Bypath and Alliance Western Bypath
    { 4964.320f, 2455.880f, 322.499f },	         // Connection to NE Workshop
    { 4906.290f, 2456.620f, 320.187f },
    { 4874.860f, 2445.190f, 320.399f },
    { 4850.280f, 2414.350f, 321.875f },
    { 4764.320f, 2428.530f, 350.697f },
    { 4687.390f, 2402.290f, 368.945f },	         // Eastern connection to Ring Road South
};

// Ring Road South: The lower half of the Wintergrasp ring road
static WgPath const vPath_WG_Ring_Road_South = {
	{ 4607.540f, 2366.920f, 379.028f },	         // Eastern connection to Ring Road North
	{ 4515.940f, 2327.430f, 369.028f },	         // Connections to SE Workshop and SE Tower Road
	{ 4468.230f, 2365.330f, 359.855f },
	{ 4441.990f, 2434.810f, 358.032f },
	{ 4454.170f, 2539.170f, 358.279f },
	{ 4472.150f, 2605.220f, 358.307f },
	{ 4467.140f, 2690.180f, 373.965f },
	{ 4454.320f, 2738.990f, 388.486f },
	{ 4504.510f, 2781.980f, 389.473f },
	{ 4509.100f, 2824.040f, 391.663f },	         // Connection to South Tower and southern connection to Central Road
	{ 4466.110f, 2888.600f, 389.463f },
	{ 4413.290f, 2922.470f, 384.222f },
	{ 4413.110f, 3006.020f, 363.123f },
	{ 4399.750f, 3103.200f, 358.638f },
	{ 4380.990f, 3202.650f, 363.351f },
	{ 4381.770f, 3252.550f, 372.235f },
	{ 4382.120f, 3296.330f, 372.429f, 0, true }, // Connections to SW Workshop and SW Tower Road
	{ 4444.060f, 3311.240f, 362.362f },
	{ 4518.720f, 3352.470f, 359.448f },
	{ 4596.720f, 3347.880f, 363.652f },	         // Connection to SW Tower Road
	{ 4689.620f, 3311.830f, 374.640f },	         // Western connection to Ring Road North
};

// Central Road: The road that split the ring road from north to south
static WgPath const vPath_WG_Central_Road = {
	{ 4975.560f, 2870.710f, 385.570f },	         // Connection to Ring Road North
	{ 4896.410f, 2887.980f, 379.237f },
	{ 4801.270f, 2892.340f, 373.895f },
	{ 4706.890f, 2867.940f, 387.224f },
	{ 4608.260f, 2845.990f, 396.896f },	         // Connection to Ring Road South
};

// Alliance Route: From Alliance spawn, to the NE graveyard, to the eastern wall of the fortress
static WgPath const vPath_WG_Alliance_Route = {

    { 5067.980f, 2203.720f, 356.622f },			 // Alliance Spawn
    { 5059.815f, 2261.002f, 356.533f },
    { 5075.046f, 2320.963f, 357.256f },			 // Connection to Alliance Bypath East
    { 5106.946f, 2416.543f, 357.371f },
    { 5137.308f, 2514.138f, 358.234f },
    { 5165.662f, 2608.387f, 382.992f },      	 // Connection to Alliance Bypath West
    { 5184.894f, 2665.395f, 397.137f },          // Connection to Fortress Bypath East (1)
    { 5195.485f, 2691.625f, 405.725f },          // Connection to Vehicle Teleporter Exit East
    { 5215.000f, 2740.100f, 409.190f, 3757 }, 	 // Fortress Wall East (destructible) and connections to Fortress Bypath East (2) and Inner Fortress Path
};

// Auxiliary path: For navigation form the NE graveyard and into Ring Road North towards the NE workshop
static WgPath const vPath_WG_Alliance_Bypath_East = {
	{ 5048.789f, 2350.387f, 360.484f },          // Connection to Alliance Route
	{ 5048.169f, 2429.670f, 360.649f },
	{ 5046.729f, 2511.246f, 356.988f },          // Connections to Ring Road North and Alliance Bypath West
};

// Auxiliary path: For navigation from the eastern side of the Fortress  and into Ring Road North towards the NE workshop
static WgPath const vPath_WG_Alliance_Bypath_West = {
    { 5093.815f, 2573.798f, 366.741f },          // Connections to Alliance Route, Ring Road North, and Alliance Bypath East
};

// Horde Route A: From Horde spawn spawn, to the NW graveyard, and into Ring Road North near NW Workshop
static WgPath const vPath_WG_Horde_Route_Part_A = {
	{ 5005.520f, 3644.310f, 360.585f },	         // Horde Spawn and connection to Far SW Path
	{ 5026.290f, 3584.060f, 356.285f },
	{ 4997.950f, 3516.910f, 356.520f },
	{ 5033.730f, 3435.770f, 359.015f },          // Connection to Far NW Path
	{ 5006.879f, 3375.797f, 375.745f },	         // Connection to Ring Road North
};

// Horde Route B: From the NW side of Ring Road North near NW Workshop, to the western wall of the fortress
static WgPath const vPath_WG_Horde_Route_Part_B = {
	{ 5097.330f, 3153.350f, 360.052f },		     // Connection to Ring Road North
	{ 5152.640f, 3074.960f, 380.069f },          // Connection to Far NW Path
	{ 5198.530f, 3000.700f, 404.440f },          // Connections to Vehicle Teleporter Exit West and Fortress Bypath West (1)
	{ 5215.000f, 2941.900f, 409.192f, 3754 },	 // Fortress Wall West (destructible) and connections to Fortress Bypath West (2) and Inner Fortress Path
};

// Defenders Route: From the fortress graveyard down to Fortress Central Court
static WgPath const vPath_WG_Defenders_Route = {
	{ 5543.799f, 2825.115f, 515.386f },	         // Fortress Graveyard
	{ 5548.690f, 2745.170f, 509.186f },
	{ 5527.080f, 2718.280f, 491.877f },
	{ 5499.430f, 2737.160f, 471.548f },
	{ 5476.960f, 2711.460f, 451.789f },
	{ 5400.870f, 2757.510f, 409.240f },          // Connection to Fortress Workshop East
};

// Inner Fortress Path: From Fortress Front Court to Vault Door
static WgPath const vPath_WG_Inner_Fortress_Path = {
	{ 5215.000f, 2841.200f, 409.192f, 0 },		 // Fortress Front Court and connections to Alliance Route, Horde Route Part B,
												 //      Fortress SW Exit Path, Fortress SE Exit Path, and Outer Fortress Path
	{ 5272.000f, 2841.200f, 409.192f, 3768 },	 // Fortress Central Wall (destructible)
	{ 5342.800f, 2841.200f, 409.240f, 0, true }, // Fortress Central Court and connections to Fortress Workshop West and East
	{ 5393.500f, 2841.200f, 418.675f, 3773 },	 // Inner vault door (destructible)
};

// Auxiliary path: For exiting the fortress through its SE tower
static WgPath const vPath_WG_Fortress_SE_Exit_Path = {
	{ 5187.440f, 2759.230f, 413.492f },	         // Connection to Inner Fortress Path
	{ 5187.590f, 2738.650f, 413.492f },
	{ 5168.230f, 2717.280f, 413.492f },	         // Fortress SE Exit and connection to Alliance Route
};

// Auxiliary path: For exiting the fortress through its SW tower
static WgPath const vPath_WG_Fortress_SW_Exit_Path = {
	{ 5186.490f, 2921.640f, 413.494f },	         // Connection to Inner Fortress Path
	{ 5186.100f, 2944.320f, 413.494f },
	{ 5167.230f, 2964.020f, 413.494f },	         // Fortress SW Exit and connection to Horde Route Part B
};

// NE Workshop
static WgPath const vPath_WG_NE_Workshop = {
	{ 4943.000f, 2388.200f, 324.045f },	         // NE Workshop Engineer
	{ 4954.340f, 2414.220f, 320.176f },	         // Connection to Ring Road North
};

// NW Workshop
static WgPath const vPath_WG_NW_Workshop = {
	{ 4961.000f, 3384.500f, 380.800f },	         // NW Workshop Engineer
	{ 4946.560f, 3361.500f, 376.877f, 0, true }, // Connection to Ring Road North
};

// SE Workshop
static WgPath const vPath_WG_SE_Workshop = {
	{ 4357.700f, 2353.700f, 379.896f },	         // SE Workshop Engineer
	{ 4383.600f, 2348.460f, 376.318f },
	{ 4458.200f, 2327.260f, 367.101f },	         // Connection to Ring Road South

};

// SW Workshop
static WgPath const vPath_WG_SW_Workshop = {
	{ 4353.200f, 3308.600f, 375.937f },	         // SW Workshop Engineer and connection to Ring Road South
};

// SE Tower: Straight road from the ring road to the tower
static WgPath const vPath_WG_SE_Tower_Road = {
	{ 4467.910f, 1964.130f, 439.296f },	         // SE Tower
	{ 4503.460f, 2047.800f, 413.551f },          // Connection to Far SE Path
	{ 4541.210f, 2139.570f, 378.752f },
	{ 4555.610f, 2236.460f, 358.642f },	         // Connections to Ring Road South and Far East Path
};

// South Tower: Single coordinate that immediately connects to the ring road
static WgPath const vPath_WG_South_Tower = {
	{ 4420.040f, 2823.010f, 409.931f },	         // South Tower and connection to Ring Road South
};

// SW Tower: Diverges from the ring road on the upper side, heads to the tower, then turns back to merge
// into the ring road at the bottom.
static WgPath const vPath_WG_SW_Tower_Road = {
	{ 4576.070f, 3408.500f, 359.965f },	         // Connection to Ring Road South towards NW Workshop
	{ 4555.770f, 3476.150f, 363.415f },
	{ 4555.940f, 3556.160f, 386.661f },
	{ 4533.640f, 3595.070f, 397.198f },	         // SW Tower
	{ 4486.850f, 3608.960f, 386.464f },
	{ 4437.850f, 3573.030f, 367.624f },          // Connection to Far SW Path
	{ 4399.420f, 3483.940f, 359.024f },
	{ 4384.300f, 3389.680f, 361.563f },	         // Connection to Ring Road South towards SW Workshop
};

// Auxiliary path: For navigation from the eastern side of the fortress to the front, while outside it
static WgPath const vPath_WG_Fortress_Bypath_East = {
    { 5164.035f, 2698.225f, 404.097f },	         // Connections to SE Exit Path and double to Alliance Route
	{ 5133.060f, 2722.050f, 409.182f },
	{ 5113.960f, 2755.300f, 408.008f },          // Double connection to Outer Fortress Path
};

// Auxiliary path: For navigation from the western side of the fortress to the front, while outside it
static WgPath const vPath_WG_Fortress_Bypath_West = {
    { 5163.267f, 2984.374f, 409.141f },	         // Connections to SW Exit Path and double to Horde Route Part B
	{ 5131.660f, 2961.040f, 409.082f },
	{ 5113.960f, 2927.860f, 408.599f },          // Double connection to Outer Fortress Path
};

// Outer Fortress Path: From Ring Road North to Fortress Gate
static WgPath const vPath_WG_Outer_Fortress_Path = {
	{ 5158.000f, 2841.200f, 408.799f, 3763 }, 	 // Fortress Gate (destructible) and connections to Inner Fortress Path
	{ 5134.000f, 2841.200f, 407.847f }, 		 // Connections Fortress Bypath East and Fortress Bypath West
	{ 5087.274f, 2844.859f, 398.176f },		     // Connections to Ring Road North, Fortress Bypath East, and Fortress Bypath West
};

// Auxiliary path: From the far SE edges to the rest of the map
static WgPath const vPath_WG_Far_SE_Path = {
    { 4277.794f, 1827.926f, 350.595f },
    { 4371.759f, 1810.238f, 354.242f },
    { 4454.418f, 1852.759f, 373.527f },
    { 4538.867f, 1899.108f, 397.904f },
    { 4552.810f, 1970.557f, 410.607f },		     // Connections to SE Tower Road and Far East SE Bypath
};

// Auxiliary path: From the far SW edges to the rest of the map
static WgPath const vPath_WG_Far_SW_Path = {
    { 4918.395f, 3686.224f, 352.428f },		     // Connection to Horde Route Part A
    { 4831.536f, 3728.451f, 350.424f },
    { 4741.445f, 3757.678f, 355.753f },
    { 4686.047f, 3822.695f, 352.459f },
    { 4647.447f, 3907.755f, 355.973f },
    { 4613.291f, 3990.165f, 376.090f },
    { 4584.142f, 4077.007f, 410.790f },
    { 4472.855f, 4045.411f, 413.114f },
    { 4428.973f, 3955.076f, 411.535f },
    { 4409.458f, 3869.009f, 395.363f },
    { 4417.413f, 3774.362f, 362.861f },
    { 4439.825f, 3684.148f, 362.966f },
    { 4386.598f, 3610.822f, 357.952f },		     // Connection to SW Tower Road
};

// Auxiliary path: From the far NW edges to the rest of the map
static WgPath const vPath_WG_Far_NW_Path = {
    { 5104.822f, 3398.345f, 356.628f },          // Connection to Horde Route Part A
    { 5184.610f, 3395.535f, 356.526f },
    { 5274.457f, 3345.488f, 356.526f },
    { 5330.025f, 3276.951f, 356.746f },
    { 5292.549f, 3193.753f, 369.141f },
    { 5228.770f, 3129.303f, 386.228f },          // Connection to Horde Route Part B
};

// Auxiliary path: From the far east edges to the rest of the map
static WgPath const vPath_WG_Far_East_Path = {
    { 4837.589f, 1887.022f, 446.530f },
    { 4745.480f, 1882.260f, 446.206f },
    { 4689.004f, 1951.491f, 427.119f },          // Connection to Far East SE Bypath
    { 4723.038f, 2008.358f, 426.284f },
    { 4771.914f, 2080.041f, 422.000f },
    { 4711.448f, 2123.804f, 398.465f },
    { 4648.592f, 2198.310f, 359.948f },          // Connection to SE Tower Road
};

// Auxiliary path: Connecting the Far SE and Far East paths
static WgPath const vPath_WG_Far_SE_East_Bypath = {
    { 4619.401f, 1952.173f, 423.072f },          // Connections to Far SE Path and Far East Path
};

// Fortress Workshop West
static WgPath const vPath_WG_Fortress_Workshop_West = {
    { 5392.900f, 2980.000f, 413.113f },          // Fortress Workshop West Engineer
    { 5342.800f, 2984.800f, 409.192f, 0, true },
    { 5342.800f, 2917.800f, 409.192f, 0, true }, // Connection to Inner Fortress Path
};

// Fortress Workshop East
static WgPath const vPath_WG_Fortress_Workshop_East = {
    { 5391.800f, 2712.400f, 412.942f },          // Fortress Workshop East Engineer
    { 5342.800f, 2718.600f, 409.167f, 0, true },
    { 5342.800f, 2762.000f, 409.191f, 0, true }, // Connections to Inner Fortress Path and Defenders Route
};

// Auxiliary path: Connecting the vehicle teleporter exit point to the rest of the map
static WgPath const vPath_WG_Vehicle_Tele_Con_East = {
    { 5256.993f, 2704.333f, 409.191f },          // Connection to Alliance Route
};

// Auxiliary path: Connecting the vehicle teleporter exit point to the rest of the map
static WgPath const vPath_WG_Vehicle_Tele_Con_West = {
    { 5257.326f, 2976.304f, 409.191f },          // Connection to Horde Route B
};

// General objectives positions:
// Infantry objectives:
static Position const WG_OBJ_INI_CONF_EAST  = { 5165.662f, 2608.387f, 382.992f, 0.0f };   // Eastern pre-wall-fall conflict
static Position const WG_OBJ_INI_CONF_WEST  = { 5152.640f, 3074.960f, 380.069f, 0.0f };   // Western pre-wall-fall conflict
static Position const WG_OBJ_CENTRAL_COURT  = { 5342.800f, 2841.200f, 409.240f, 0.0f };   // Attackers goal
static Position const WG_OBJ_FRONT_COURT    = { 5215.000f, 2841.200f, 409.192f, 0.0f };   // Defenders goal
// Vehicle objectives:
static Position const WG_OBJ_CENTRAL_STAGE  = { 5051.660f, 2847.730f, 393.182f, 0.0f };   // Central staging area
static Position const WG_OBJ_DEF_GUARD_EAST = { 5195.485f, 2691.625f, 405.725f, 0.0f };   // East wall defender vehicles guard point
static Position const WG_OBJ_DEF_GUARD_WEST = { 5198.530f, 3000.700f, 404.440f, 0.0f };   // West wall defender vehicles guard point
static Position const WG_OBJ_DEF_GUARD_GATE = { 5134.000f, 2841.200f, 407.847f, 0.0f };   // Fotress Gate defender vehicles guard point
static Position const WG_OBJ_FORTRESS_GATE  = { 5158.000f, 2841.200f, 408.799f, 0.0f };   // Goal A (Gate)
static Position const WG_OBJ_CENTRAL_WALL   = { 5272.000f, 2841.200f, 409.192f, 0.0f };   // Goal B (Central Wall)
static Position const WG_OBJ_VAULT_DOOR     = { 5393.500f, 2841.200f, 418.675f, 0.0f };   // Goal C (Vault Door)
static Position const WG_OBJ_EAST_WALL      = { 5215.000f, 2740.100f, 409.190f, 0.0f };   // Alternative goal A (Eastern Wall)
static Position const WG_OBJ_WEST_WALL      = { 5215.000f, 2941.900f, 409.192f, 0.0f };   // Alternative goal A (Western Wall)
static Position const WG_OBJ_WS_TELE_WEST   = { 5316.250f, 2977.040f, 408.539f, 0.0f };   // Fortress Vehicle Teleporter (Workshop West)
static Position const WG_OBJ_WS_TELE_EAST   = { 5314.510f, 2703.690f, 408.550f, 0.0f };   // Fortress Vehicle Teleporter (Workshop East)
static Position const WG_OBJ_SE_TOWER	    = { 4467.910f, 1964.130f, 439.296f, 0.0f };   // Attackers Tower (SE)
static Position const WG_OBJ_SOUTH_TOWER    = { 4420.040f, 2823.010f, 409.931f, 0.0f };   // Attackers Tower (South)
static Position const WG_OBJ_SW_TOWER       = { 4533.640f, 3595.070f, 397.198f, 0.0f };   // Attackers Tower (SW)

// Workshop data for navigation. Maps workshop IDs to their WgPath.
struct WgWorkshopData { uint8 workshopId; WgPath const* path; };
static WgWorkshopData const WG_WORKSHOPS[] = {
    { 0, &vPath_WG_NE_Workshop },           // NE - Sunken Ring
    { 1, &vPath_WG_NW_Workshop },           // NW - Broken Temple
    { 2, &vPath_WG_SE_Workshop },           // SE - Eastspark
    { 3, &vPath_WG_SW_Workshop },           // SW - Westspark
    { 4, &vPath_WG_Fortress_Workshop_West}, // West - Fortress
    { 5, &vPath_WG_Fortress_Workshop_East}, // East - Fortress
};

// ################# //
// A* Waypoint Graph
// ################# //

// Path blocking mechanics:
// Node block (WgNode::pathBlock): building stands AT the waypoint. Vehicles hold and attack it, but infantry pass.
// Edge block (WgEdge::blockWorldState): passage BETWEEN waypoints (junctions) is sealed for everyone until destroyed.
// Solid barriers use both, meanwhile blockers with infantry gaps use only the node block.
struct WgEdge { uint32 target; uint32 blockWorldState = 0; };

struct WgNode
{
    float    x, y, z;
    uint32   pathBlock;     // WorldState ID of the building standing at this node (0 = none)
    bool     noSkip;        // true = next waypoint skipping logic is limited
    bool     noVehicle;     // true = A* skips this node when routing vehicles
    std::vector<WgEdge> adj;
};
static std::vector<WgNode> g_WgGraph;
static std::once_flag      g_WgGraphOnce;

static WgPath const* const g_AllWgPaths[] = {
    &vPath_WG_Ring_Road_North,          // Path  0 - Waypoints: 21
    &vPath_WG_Ring_Road_South,          // Path  1 - Waypoints: 21
    &vPath_WG_Central_Road,             // Path  2 - Waypoints: 5
    &vPath_WG_Alliance_Route,           // Path  3 - Waypoints: 9
    &vPath_WG_Alliance_Bypath_East,     // Path  4 - Waypoints: 3
    &vPath_WG_Alliance_Bypath_West,     // Path  5 - Waypoints: 1
    &vPath_WG_Horde_Route_Part_A,       // Path  6 - Waypoints: 5
    &vPath_WG_Horde_Route_Part_B,       // Path  7 - Waypoints: 4
    &vPath_WG_Defenders_Route,          // Path  8 - Waypoints: 6
    &vPath_WG_Inner_Fortress_Path,      // Path  9 - Waypoints: 4
    &vPath_WG_Fortress_SE_Exit_Path,    // Path 10 - Waypoints: 3
    &vPath_WG_Fortress_SW_Exit_Path,    // Path 11 - Waypoints: 3
    &vPath_WG_NE_Workshop,              // Path 12 - Waypoints: 2
    &vPath_WG_NW_Workshop,              // Path 13 - Waypoints: 2
    &vPath_WG_SE_Workshop,              // Path 14 - Waypoints: 3
    &vPath_WG_SW_Workshop,              // Path 15 - Waypoints: 1
    &vPath_WG_SE_Tower_Road,            // Path 16 - Waypoints: 4
    &vPath_WG_South_Tower,              // Path 17 - Waypoints: 1
    &vPath_WG_SW_Tower_Road,            // Path 18 - Waypoints: 8
    &vPath_WG_Outer_Fortress_Path,      // Path 19 - Waypoints: 3
    &vPath_WG_Fortress_Bypath_East,     // Path 20 - Waypoints: 3
    &vPath_WG_Fortress_Bypath_West,     // Path 21 - Waypoints: 3
    &vPath_WG_Far_SE_Path,              // Path 22 - Waypoints: 5
    &vPath_WG_Far_SW_Path,              // Path 23 - Waypoints: 13
    &vPath_WG_Far_NW_Path,              // Path 24 - Waypoints: 6
    &vPath_WG_Far_East_Path,            // Path 25 - Waypoints: 7
    &vPath_WG_Far_SE_East_Bypath,       // Path 26 - Waypoints: 1
    &vPath_WG_Fortress_Workshop_West,   // Path 27 - Waypoints: 3
    &vPath_WG_Fortress_Workshop_East,   // Path 28 - Waypoints: 3
    &vPath_WG_Vehicle_Tele_Con_East,    // Path 29 - Waypoints: 1
    &vPath_WG_Vehicle_Tele_Con_West,    // Path 30 - Waypoints: 1
};
static constexpr uint8 WG_PATH_COUNT = 31;  // Total waypoints: 155

// WorldState IDs for the four passable fortress obstacles.
// These are the IDs broadcast via UpdateWorldState when building state changes,
// allowing state checks from anywhere on the map without range-limited GO scanning.
static constexpr uint32 WG_WS_FORTRESS_GATE  = 3763;  // Entry 190375 - Fortress Gate
static constexpr uint32 WG_WS_EAST_WALL      = 3757;  // Entry 190372 - Eastern Wall
static constexpr uint32 WG_WS_WEST_WALL      = 3754;  // Entry 190371 - Western Wall
static constexpr uint32 WG_WS_CENTRAL_WALL   = 3768;  // Entry 191805 - Central Wall
static constexpr uint32 WG_WS_VAULT_DOOR     = 3773;  // Entry 191810 - Vault Door

// WorldState IDs for the three attacker towers.
static constexpr uint32 WG_WS_TOWER_SE       = 3706;  // Entry 190358 - SE Tower (Flamewatch)
static constexpr uint32 WG_WS_TOWER_SOUTH    = 3705;  // Entry 190357 - South Tower (Winter's Edge)
static constexpr uint32 WG_WS_TOWER_SW       = 3704;  // Entry 190356 - SW Tower (Shadowsight)

// Cross-path junction edges connecting waypoints across different paths.
// oneWay=true: only the A->B edge is added (pathA is the source direction).
// blockWorldState!=0: the edge is impassable to all until the wall with that WorldState ID is destroyed. Vehicles
// can still approach (without crossing) a standing blocker, as their attack objective is the wall node itself.
struct WgJunctionDef { uint8 pathA, wpA, pathB, wpB; bool oneWay = false; uint32 blockWorldState = 0; };
static WgJunctionDef const WG_JUNCTIONS[] = {
    // Bi-directional junctions:
    {  0,  0,   1, 20 },  // RRN[0]      <->    RRS[20]     Western connection of ring roads
    {  0, 20,   1,  0 },  // RRN[20]     <->    RRS[0]      Eastern connection of ring roads
    {  0,  9,   2,  0 },  // RRN[9]      <->    CRoad[0]    Central Road North
    {  0, 14,   4,  2 },  // RRN[14]     <->    A_By_E[2]   Alliance Bypath East
    {  0, 14,   5,  0 },  // RRN[14]     <->    A_By_W[0]   Alliance Bypath West
    {  0,  3,   6,  4 },  // RRN[3]      <->    HR_A[4]     Horde Part A
    {  0,  4,   7,  0 },  // RRN[4]      <->    HR_B[0]     Horde Part B
    {  0, 15,  12,  1 },  // RRN[15]     <->    NEWork[1]   To NE Workshop
    {  0,  2,  13,  1 },  // RRN[2]      <->    NWWork[1]   To NW Workshop
    {  0,  9,  19,  2 },  // RRN[9]      <->    OFP[2]      Outer Fortress Path

    {  1,  9,   2,  4 },  // RRS[9]      <->    CRoad[4]    Central Road South
    {  1,  1,  14,  2 },  // RRS[1]      <->    SEWork[2]   To SE Workshop
    {  1, 16,  15,  0 },  // RRS[16]     <->    SWWork[0]   To SW Workshop
    {  1,  1,  16,  3 },  // RRS[1]      <->    SETwr[3]    To SE Tower
    {  1,  9,  17,  0 },  // RRS[9]      <->    STwr[0]     South Tower
    {  1, 19,  18,  0 },  // RRS[19]     <->    SWTwr[0]    To SW Tower (top)
    {  1, 16,  18,  7 },  // RRS[16]     <->    SWTwr[7]    To SW Tower (bottom)

    {  3,  2,   4,  0 },  // AR[2]       <->    A_By_E[0]   Alliance Bypath East
    {  3,  5,   5,  0 },  // AR[5]       <->    A_By_W[0]   Alliance Bypath West
    {  3,  6,  20,  0 },  // AR[6]       <->    F_By_E[0]   Fortress Bypath East (first connection)
    {  3,  7,  29,  0 },  // AR[7]       <->    Tel_Ex_E[0] Vehicle Teleporter Exit East
    {  3,  8,  20,  0 },  // AR[8]       <->    F_By_E[0]   Fortress Bypath East (second connection)
    {  4,  2,   5,  0 },  // A_By_E[2]   <->    A_By_W[0]   Alliance bypaths join near Ring Road North

    {  6,  0,  23,  0 },  // HR_A[0]     <->    Far_SW[0]   To the far SW edges of the map
    {  6,  3,  24,  0 },  // HR_A[3]     <->    Far_NW[0]   To the far NW edges of the map
    {  7,  1,  24,  5 },  // HR_B[1]     <->    Far_NW[5]   To the far NW edges of the map
    {  7,  2,  21,  0 },  // HR_B[2]     <->    F_By_W[0]   Fortress Bypath West (first connection)
    {  7,  2,  30,  0 },  // HR_B[2]     <->    Tel_Ex_W[0] Vehicle Teleporter Exit West
    {  7,  3,  21,  0 },  // HR_B[3]     <->    F_By_W[0]   Fortress Bypath West (second connection)

    { 16,  1,  22,  4 },  // SETwr[1]    <->    Far_SE[4]   To the far SE edges of the map
    { 16,  3,  25,  6 },  // SETwr[3]    <->    Far_E[6]    To the far east edges of the map
    { 18,  5,  23, 12 },  // SWTwr[5]    <->    Far_SW[12]  To the far SW edges of the map

    { 19,  1,  20,  2 },  // OFP[1]      <->    F_By_E[2]   Fortress Bypath East (first connection)
    { 19,  2,  20,  2 },  // OFP[2]      <->    F_By_E[2]   Fortress Bypath East (second connection)
    { 19,  1,  21,  2 },  // OFP[1]      <->    F_By_W[2]   Fortress Bypath West (first connection)
    { 19,  2,  21,  2 },  // OFP[2]      <->    F_By_W[2]   Fortress Bypath West (second connection)

    { 26,  0,  22,  4 },  // Far_E_SE[0] <->    Far_SE[4]   Far East and Far SE bypath connection
    { 26,  0,  25,  2 },  // Far_E_SE[0] <->    Far_E[2]    Far East and Far SE bypath connection

    { 27,  2,   9,  2 },  // F_WS_W[2]   <->    IFP[2]      From Fortress Workshop West to Central Fortress Court
    { 28,  2,   8,  5 },  // F_WS_E[2]   <->    DefR[5]     From Fortress Workshop East to the Defenders Route
    { 28,  2,   9,  2 },  // F_WS_E[2]   <->    IFP[2]      From Fortress Workshop East to Central Fortress Court

    // Mono-directional junctions:
    {  9,  0,  10,  0, true },  // IFP[0]      ->    SE_Exit[0]  Inner Fortress Path to SE Exit
    {  9,  0,  11,  0, true },  // IFP[0]      ->    SW_Exit[0]  Inner Fortress Path to SW Exit
    { 10,  2,   3,  6, true },  // SEExit[2]   ->    AR[6]       From SE fortress tower to Alliance Route
    { 10,  2,  20,  0, true },  // SEExit[2]   ->    F_By_E[0]   From SE fortress tower to Fortress Bypath East
    { 11,  2,   7,  2, true },  // SWExit[2]   ->    HR_B[2]     From SW fortress tower to Horde Route B
    { 11,  2,  21,  0, true },  // SWExit[2]   ->    F_By_W[0]   From SW fortress tower to Fortress Bypath West

    // Object-blocked junctions:
    {  9,  0,   3,  8,  false, WG_WS_EAST_WALL },       // IFP[0]   <->   AR[8]     East wall
    {  9,  0,   7,  3,  false, WG_WS_WEST_WALL },       // IFP[0]   <->   HR_B[3]   West wall
    {  9,  0,  19,  0,  false, WG_WS_FORTRESS_GATE },   // IFP[0]   <->   OFP[0]    Front Gate
};

// Builds the A* waypoint graph from path and junction definitions. Called once on first use via std::call_once.
static void BuildWgGraph()
{
    std::call_once(g_WgGraphOnce, []()
    {
        uint32 pathOffset[WG_PATH_COUNT] = {};

        // Step 1: Add all nodes, recording the start index of each path.
        for (uint8 i = 0; i < WG_PATH_COUNT; ++i)
        {
            pathOffset[i] = static_cast<uint32>(g_WgGraph.size());
            for (WgWaypoint const& wp : *g_AllWgPaths[i])
                g_WgGraph.push_back({ wp.x, wp.y, wp.z, wp.pathBlock, wp.noSkip, false, {} });
        }

        // Mark vehicle-excluded paths (infantry-only shortcuts).
        static constexpr uint8 WG_NO_VEHICLE_PATHS[] = { 10, 11 };
        for (uint8 p : WG_NO_VEHICLE_PATHS)
        {
            uint32 base = pathOffset[p];
            uint32 len  = static_cast<uint32>(g_AllWgPaths[p]->size());
            for (uint32 j = 0; j < len; ++j)
                g_WgGraph[base + j].noVehicle = true;
        }

        // Step 2: Within-path sequential edges (bidirectional).
        for (uint8 i = 0; i < WG_PATH_COUNT; ++i)
        {
            uint32 base = pathOffset[i];
            uint32 len  = static_cast<uint32>(g_AllWgPaths[i]->size());
            for (uint32 j = 0; j < len - 1; ++j)
            {
                g_WgGraph[base + j    ].adj.push_back({ base + j + 1, 0 });
                g_WgGraph[base + j + 1].adj.push_back({ base + j,     0 });
            }
        }

        // Step 3: Cross-path junction edges. Bidirectional unless oneWay is set,in which case only the A->B edge
        // is added (pathA is the source).
        for (auto const& junc : WG_JUNCTIONS)
        {
            uint32 nodeA = pathOffset[junc.pathA] + junc.wpA;
            uint32 nodeB = pathOffset[junc.pathB] + junc.wpB;
            g_WgGraph[nodeA].adj.push_back({ nodeB, junc.blockWorldState });
            if (!junc.oneWay)
                g_WgGraph[nodeB].adj.push_back({ nodeA, junc.blockWorldState });
        }
    });
}

// Fortress Cannons: The fortress has 24 cannons, but only 12 are useful to intercept typical paths of hostiles.
static Position const WG_DEFENDER_CANNON_POSITIONS[] = {
    { 5264.585f, 2819.800f, 421.739f, 0.0f },
    { 5264.236f, 2861.381f, 421.669f, 0.0f },
    { 5264.887f, 2704.792f, 421.783f, 0.0f },
    { 5236.105f, 2732.727f, 421.732f, 0.0f },
    { 5163.863f, 2721.933f, 439.928f, 0.0f },
    { 5137.889f, 2747.527f, 439.928f, 0.0f },
    { 5148.564f, 2820.538f, 421.704f, 0.0f },
    { 5147.750f, 2861.868f, 421.713f, 0.0f },
    { 5136.843f, 2935.265f, 439.930f, 0.0f },
    { 5163.509f, 2960.821f, 439.930f, 0.0f },
    { 5234.786f, 2948.732f, 420.963f, 0.0f },
    { 5265.910f, 2976.459f, 421.149f, 0.0f },
};
static constexpr uint8  WG_DEFENDER_CANNON_COUNT    = 12;
static constexpr float  CANNON_TARGET_SCAN          = 85.0f;    // Lock on targets somewhat beyond cannon's range (70).
static constexpr float  VEHICLE_TARGET_SCAN         = 65.0f;    // Don't look at targets beyond 70 yards. That will cause
                                                                    // fallback of hurl boulder to ram instead.

// Summoning Vehicles: Don't send every eligible bot to get a vehicle. Send only an amount relative to how many
// available vehicles there are to summon.
static constexpr float  WS_GO_ATK_MULTIPLIER        = 1.0f;     // Multiplier for attacker bots going to summon a vehicle.
static constexpr float  WS_GO_DEF_MULTIPLIER        = 3.0f;     // Multiplier for defender bots going to summon a vehicle.
static std::atomic<int32_t> s_WgAtkGoingToWorkshop{0};
static std::atomic<int32_t> s_WgDefGoingToWorkshop{0};
static constexpr uint8  VEHICLE_FORT_GUARD          = 6;        // Defender vehicles assigned to fortress defense instead of attacking towers.
static std::atomic<int32_t> s_WgFortGuardVehicles{0};           // Counter for VEHICLE_FORT_GUARD. Used for tracking defender vehicle phases.
static std::atomic<int32_t> s_WgFortGuardAtStage{0};            // Counter for defender vehicles assigned to hisGuardAtStage.
static std::atomic<int32_t> s_WgFortGuardAtGate{0};             // Counter for defender vehicles assigned to hisGuardAtGate.
static std::atomic<int32_t> s_WgFortGuardAtOtherSide{0};        // Counter for defender vehicles assigned to hisGuardAtOtherSide, which is the
                                                                    // side opposing hisGuardAtStage.
static constexpr uint32 GUARD_REBALANCE_PERIOD      = 10000u;   // Period to re-evaluate fort guard positions.
static std::mutex           s_WgFortGuardPosMtx;                // Sequences fort guard slot claims and Stage/Gate assignment.
static constexpr uint8  MAX_TOWER_SQUAD             = 3;        // Max defender vehicles assigned to attack any single tower.
static std::atomic<int32_t> s_WgTowerSquad[3]{};                // Per-tower squad counters, indexed by DEF_TOWERS[].
static std::mutex           s_WgTowerSquadMtx;                  // Sequences tower squad assignment to prevent races on tower reassignments.
// Tracks bots currently navigating to capture a hostile/neutral workshop.
// Set by TryCaptureWorkshop; cleared when capture ends or WG battle resets.
static std::mutex                   s_WgCapturingWorkshopMtx;
static std::unordered_set<uint64_t> s_WgCapturingWorkshop;

// Workshop Capture: percentage of infantry bots diverted to capture workshops.
// Only the capturable workshops are considered for calculating this percentage.
static constexpr uint8  WG_CAPTURE_PCT_HIGH           = 60;     // Team owns < 2 workshops
static constexpr uint8  WG_CAPTURE_PCT_MID            = 40;     // Team owns 2 workshops
static constexpr uint8  WG_CAPTURE_PCT_LOW            = 20;     // Team owns 3 workshops
static constexpr uint8  WG_CAPTURABLE_WORKSHOP_COUNT  = 4;      // Only workshops 0-3 are capturable

// Due to the map typography and the pathways, the Horde has an edge in capturing workshops. These additive modifiers to the percentage of bots
// sent to capture workshops (capturePct) can help balance things. Keep values within range of 0 to 40.
static constexpr int8   WG_CAP_MOD_ALLIANCE_ATK       = 15;     // capturePct modifier: Alliance attacker
static constexpr int8   WG_CAP_MOD_ALLIANCE_DEF       = 20;     // capturePct modifier: Alliance defender
static constexpr int8   WG_CAP_MOD_HORDE_ATK          = 0;      // capturePct modifier: Horde attacker
static constexpr int8   WG_CAP_MOD_HORDE_DEF          = 5;      // capturePct modifier: Horde defender

// ############# //
// Match Balance
// ############# //
// WS_GO_ATK_MULTIPLIER and WS_GO_DEF_MULTIPLIER along with the four capturePct modifiers, can be tweaked to tune match balance. Here
// "match balance" is defined such that by the end of a default 30 minute match of 120 v. 120, the result is between WG_OBJ_CENTRAL_WALL
// having some damage, all the way to WG_OBJ_VAULT_DOOR destroyed but having no more than 5 minutes left on the timer when that happens.
// If the any of the values need to be modified based on the current state of the code logic, make sure to run several battles and test the
// effect of any changes. Small tweaks can have large effects on match balance.
// TODO: Consider exposing these values as a config, with the default values being as close to match balance as possible.

// Entries tracking all possible hostile vehicles:
static uint32 const WG_HOSTILE_VEHICLE_ENTRIES[] = {
    NPC_WINTERGRASP_TOWER_CANNON,
    NPC_WINTERGRASP_CATAPULT,
    NPC_WINTERGRASP_DEMOLISHER,
    NPC_WINTERGRASP_SIEGE_ENGINE_ALLIANCE,
    NPC_WINTERGRASP_SIEGE_ENGINE_HORDE,
    NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_ALLIANCE,
    NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_HORDE,
};

// ####### //
// Helpers
// ####### //

// Bitfield mapping for cached building destroyed states. All 8 destructible buildings fit in a single uint8.
static constexpr uint8 WG_BLDG_FORTRESS_GATE = 1 << 0;
static constexpr uint8 WG_BLDG_EAST_WALL     = 1 << 1;
static constexpr uint8 WG_BLDG_WEST_WALL     = 1 << 2;
static constexpr uint8 WG_BLDG_CENTRAL_WALL  = 1 << 3;
static constexpr uint8 WG_BLDG_VAULT_DOOR    = 1 << 4;
static constexpr uint8 WG_BLDG_TOWER_SE      = 1 << 5;
static constexpr uint8 WG_BLDG_TOWER_SOUTH   = 1 << 6;
static constexpr uint8 WG_BLDG_TOWER_SW      = 1 << 7;

static uint8 WgWsToBit(uint32 ws)
{
    switch (ws)
    {
        case WG_WS_FORTRESS_GATE: return WG_BLDG_FORTRESS_GATE;
        case WG_WS_EAST_WALL:     return WG_BLDG_EAST_WALL;
        case WG_WS_WEST_WALL:     return WG_BLDG_WEST_WALL;
        case WG_WS_CENTRAL_WALL:  return WG_BLDG_CENTRAL_WALL;
        case WG_WS_VAULT_DOOR:    return WG_BLDG_VAULT_DOOR;
        case WG_WS_TOWER_SE:      return WG_BLDG_TOWER_SE;
        case WG_WS_TOWER_SOUTH:   return WG_BLDG_TOWER_SOUTH;
        case WG_WS_TOWER_SW:      return WG_BLDG_TOWER_SW;
        default:                  return 0;
    }
}

// Translates a WorldState ID to its corresponding GO entry for FindNearestGameObject calls.
// Used by vehicle engagement logic which needs an actual GameObject* for spell targeting.
static uint32 WgWsToGoEntry(uint32 ws)
{
    switch (ws)
    {
        case WG_WS_FORTRESS_GATE: return 190375;
        case WG_WS_EAST_WALL:     return 190372;
        case WG_WS_WEST_WALL:     return 190371;
        case WG_WS_CENTRAL_WALL:  return 191805;
        case WG_WS_VAULT_DOOR:    return 191810;
        case WG_WS_TOWER_SE:      return 190358;
        case WG_WS_TOWER_SOUTH:   return 190357;
        case WG_WS_TOWER_SW:      return 190356;
        default:                  return 0;
    }
}

// Returns true if the building with the given WorldState ID is in a destroyed state.
// Internally caches a bitfield of all building states, refreshed at most once per second by
// querying BattlefieldWG::IsBuildingDestroyed for each tracked WorldState.
static bool WgIsBuildingDestroyed(BattlefieldWG* wg, uint32 worldState)
{
    static constexpr uint32 WG_TRACKED_WORLDSTATES[] = {
        WG_WS_FORTRESS_GATE, WG_WS_EAST_WALL, WG_WS_WEST_WALL,   WG_WS_CENTRAL_WALL,
        WG_WS_VAULT_DOOR,    WG_WS_TOWER_SE,  WG_WS_TOWER_SOUTH, WG_WS_TOWER_SW,
    };

    static uint8  s_destroyedMask = 0;
    static uint32 s_cacheTime     = 0;

    uint32 now = getMSTime();
    if (now - s_cacheTime > 1000 || s_cacheTime == 0)
    {
        s_cacheTime = now;
        // Build into a local and assign once, so readers never see a partially rebuilt mask.
        uint8 mask = 0;
        for (uint32 ws : WG_TRACKED_WORLDSTATES)
            if (wg->IsBuildingDestroyed(ws))
                mask |= WgWsToBit(ws);
        s_destroyedMask = mask;
    }

    return (s_destroyedMask & WgWsToBit(worldState)) != 0;
}

static uint32 WgFindNearestNode(float x, float y, float z)
{
    uint32 best   = 0;
    float  bestD2 = std::numeric_limits<float>::max();
    for (uint32 i = 0; i < static_cast<uint32>(g_WgGraph.size()); ++i)
    {
        float dx = g_WgGraph[i].x - x;
        float dy = g_WgGraph[i].y - y;
        float dz = g_WgGraph[i].z - z;
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestD2)
        {
            bestD2 = d2;
            best   = i;
        }
    }
    return best;
}

// Returns an ordered list of graph node indices from start to goal using A*.
// wg = nullptr skips wall checks. isVehicle = true skips infantry-only nodes. Returns empty if no path exists.
static std::vector<uint32> WgAStarPath(uint32 start, uint32 goal, BattlefieldWG* wg = nullptr, bool isVehicle = false)
{
    if (start == goal)
        return {};

    uint32 const N = static_cast<uint32>(g_WgGraph.size());
    thread_local std::vector<float>  gCost;
    thread_local std::vector<uint32> parent;
    thread_local std::vector<bool>   closed;
    gCost.assign(N, std::numeric_limits<float>::infinity());
    parent.assign(N, UINT32_MAX);
    closed.assign(N, false);

    auto heuristic = [&](uint32 node) -> float
    {
        float dx = g_WgGraph[node].x - g_WgGraph[goal].x;
        float dy = g_WgGraph[node].y - g_WgGraph[goal].y;
        float dz = g_WgGraph[node].z - g_WgGraph[goal].z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    using PQEntry = std::pair<float, uint32>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> openSet;

    gCost[start] = 0.0f;
    openSet.push({ heuristic(start), start });

    while (!openSet.empty())
    {
        auto [f, cur] = openSet.top();
        openSet.pop();

        if (closed[cur])
            continue;
        closed[cur] = true;

        if (cur == goal)
        {
            std::vector<uint32> path;
            for (uint32 n = goal; n != UINT32_MAX; n = parent[n])
                path.push_back(n);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (WgEdge const& e : g_WgGraph[cur].adj)
        {
            if (closed[e.target])
                continue;
            if (isVehicle && g_WgGraph[e.target].noVehicle)
                continue;
            if (wg && e.blockWorldState && !WgIsBuildingDestroyed(wg, e.blockWorldState))
                continue;
            float dx    = g_WgGraph[e.target].x - g_WgGraph[cur].x;
            float dy    = g_WgGraph[e.target].y - g_WgGraph[cur].y;
            float dz    = g_WgGraph[e.target].z - g_WgGraph[cur].z;
            float tentG = gCost[cur] + std::sqrt(dx*dx + dy*dy + dz*dz);
            if (tentG < gCost[e.target])
            {
                gCost[e.target]  = tentG;
                parent[e.target] = cur;
                openSet.push({ tentG + heuristic(e.target), e.target });
            }
        }
    }

    return {};  // No path found. The prominent example is destination is inside the fortress and walls are still standing.
}

static BattlefieldWG* GetBattlefieldWG()
{
    return dynamic_cast<BattlefieldWG*>(sBattlefieldMgr->GetBattlefieldByBattleId(BATTLEFIELD_BATTLEID_WG));
}

// Counts the capturable workshops (0-3, excludes fortress workshops 4-5) owned by the given team.
static uint8 WgCountCapturedWorkshops(BattlefieldWG* wg, TeamId team)
{
    uint8 count = 0;
    for (uint8 i = 0; i < WG_CAPTURABLE_WORKSHOP_COUNT; ++i)
        if (wg->GetWorkshopTeam(WG_WORKSHOPS[i].workshopId) == team)
            ++count;
    return count;
}

// Assigns a capturable workshop (indices 0-3 only) not owned by the giventeam, distributed by bot GUID.
// Returns the WG_WORKSHOPS[] index, or 0xFF if none found.
static uint8 WgPickCapturableWorkshop(BattlefieldWG* wg, Player* bot, TeamId team)
{
    uint8 candidates[WG_CAPTURABLE_WORKSHOP_COUNT];
    uint8 count = 0;
    for (uint8 i = 0; i < WG_CAPTURABLE_WORKSHOP_COUNT; ++i)
    {
        if (wg->GetWorkshopTeam(WG_WORKSHOPS[i].workshopId) == team)
            continue;
        candidates[count++] = i;
    }
    if (count == 0)
        return 0xFF;
    return candidates[bot->GetGUID().GetCounter() % count];
}

// Find the nearest mounted hostile vehicle within scanRange yards.
static Creature* WgFindNearestHostileVehicle(Player* bot, float scanRange)
{
    Creature* nearest = nullptr;
    float nearestDist = FLT_MAX;
    for (uint32 entry : WG_HOSTILE_VEHICLE_ENTRIES)
    {
        std::list<Creature*> found;
        bot->GetCreatureListWithEntryInGrid(found, entry, scanRange);
        for (Creature* c : found)
        {
            if (!c->IsAlive() || !bot->IsHostileTo(c))
                continue;
            Vehicle* vKit = c->GetVehicleKit();
            if (!vKit || !vKit->GetPassenger(0))
                continue;
            float d = bot->GetDistance(c);
            if (d < nearestDist)
            {
                nearestDist = d;
                nearest = c;
            }
        }
    }
    return nearest;
}

// ######################################################################################################################################### //
// WgCheckFlagAction is the central decision-maker for bot behavior during a Wintergrasp battle.
// Each tick it determines what a bot should be doing based on its role (attacker/defender), combat rank (First Lieutenant or not),
// and state (Infantry, vehicle, or whatever else).
//
// For infantry, First Lieutenant bots are sent to friendly workshops to summon vehicles (with slot-capping to avoid overcrowding),
// a GUID-based percentage of remaining bots are diverted to capture hostile workshops via TryCaptureWorkshop, with taskless bots sent to
// a conflict point on the side of the fortress, before the outer walls are breached, and falling back to the fortress courtyards after.
//
// For vehicles, attackers run a phased order that sequences through fortress obstacles: pick a path (gate, east wall, or west wall),
// go to stage position, attack the obstacle, then advance into the fortress and all the way to the vault door. Meanwhile some defender
// vehicles guard the fortress, and others head to destroy attacker towers in squads of limited number of vehicles, per standing tower, in a
// sweep proximity order. And then after all the towers are destroyed, all fall back to the fortress to defend it from attackers.
//
// All navigation flows through FollowWgRoute, which uses the A* graph to produce waypoint-by-waypoint movement with look-ahead logic,
// wall-blocking awareness, jitter to avoid duplicate-move suppression, and a plethora of path junction-based rules.
// ######################################################################################################################################### //
WgCheckFlagAction::~WgCheckFlagAction()
{
    ClearSharedTracking();
}

// Releases the bot's claims on shared trackers: the workshop-goer, fort guard, and tower squad counters,
// plus the bot's s_WgCapturingWorkshop entry. If there are bot pointers to clear, they belong in ResetBattleState,
// never here, as the destructor calls ClearSharedTracking when the bot may already be deleted.
void WgCheckFlagAction::ClearSharedTracking()
{
    if (m_atkGoingToWorkshop)
    {
        m_atkGoingToWorkshop = false;
        --s_WgAtkGoingToWorkshop;
    }
    if (m_defGoingToWorkshop)
    {
        m_defGoingToWorkshop = false;
        --s_WgDefGoingToWorkshop;
    }
    if (m_isFortGuard)
    {
        m_isFortGuard = false;
        --s_WgFortGuardVehicles;
    }
    if (m_defGuardFortress == 1)
        --s_WgFortGuardAtStage;
    else if (m_defGuardFortress == 2)
        --s_WgFortGuardAtGate;
    else if (m_defGuardFortress == 3)
        --s_WgFortGuardAtOtherSide;
    m_defGuardFortress = 0;
    if (m_isTowerAttacker)
    {
        m_isTowerAttacker = false;
        --s_WgTowerSquad[m_targetTowerIdx];
    }
    m_targetTowerIdx   = 0xFF;
    if (m_botGuidRaw)
    {
        std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
        s_WgCapturingWorkshop.erase(m_botGuidRaw);
    }
}

// Resets all per-bot battle state: shared counters, route, phases, and capture assignment.
// Called on death and by BfStrategyCheckAction when it deactivates the strategy at end of battle.
void WgCheckFlagAction::ResetBattleState()
{
    ClearSharedTracking();
    m_route.clear();
    m_routeStep        = 0;
    m_atkVehiclePhase  = 0;
    m_defVehiclePhase  = 0;
    m_workshopIdx      = 0xFF;
    m_captureWsIdx     = 0xFF;
    m_arrivedAtCapture = false;
}

// Routes a percentage of infantry bots to capture workshops. Once assigned, a bot is committed to its target workshop
// until its team captures it, its death, or end of battle. The capture force size adapts to how many workshops the team
// owns, but only when handing out new assignments.
bool WgCheckFlagAction::TryCaptureWorkshop(BattlefieldWG* wg)
{
    TeamId team = bot->GetTeamId();

    // Assigned (en route or arrived).
    if (m_captureWsIdx != 0xFF)
    {
        // The team captured the target: release. A fresh roll may hand out a new assignment next tick.
        if (wg->GetWorkshopTeam(WG_WORKSHOPS[m_captureWsIdx].workshopId) == team)
        {
            m_captureWsIdx     = 0xFF;
            m_arrivedAtCapture = false;
            std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
            s_WgCapturingWorkshop.erase(m_botGuidRaw);
            return false;
        }

        if (m_arrivedAtCapture)
        {
            // Combat Yielding: Check for both active combat and nearby enemy presence.
            // IsCapturingWorkshop is cleared on arrival, so "enemy player target" can find targets. This makes sure bots
            // would engage hostiles around the workshop, while they are trying to capture it.
            Unit* enemyTarget = AI_VALUE(Unit*, "enemy player target");
            if (bot->IsInCombat() || enemyTarget)
                return false;

            // Not fighting: navigate back if displaced. Block lower-priority routing.
            WgPath const& wsPath = *WG_WORKSHOPS[m_captureWsIdx].path;
            Position const pos(wsPath[0].x, wsPath[0].y, wsPath[0].z, 0.0f);
            FollowWgRoute(pos, false);
            return true;
        }
    }
    else
    {
        uint8 ownedCount = WgCountCapturedWorkshops(wg, team);
        uint8 capturePct = 0;
        if (ownedCount < 2)
            capturePct = WG_CAPTURE_PCT_HIGH;
        else if (ownedCount == 2)
            capturePct = WG_CAPTURE_PCT_MID;
        else if (ownedCount == 3)
            capturePct = WG_CAPTURE_PCT_LOW;

        // Add capture percentage modifier, depending on faction, and faction role as attacker/defender.
        bool isAttacker = (team != wg->GetDefenderTeam());
        if (team == TEAM_ALLIANCE)
            capturePct += isAttacker ? WG_CAP_MOD_ALLIANCE_ATK : WG_CAP_MOD_ALLIANCE_DEF;
        else
            capturePct += isAttacker ? WG_CAP_MOD_HORDE_ATK : WG_CAP_MOD_HORDE_DEF;

        // Not rolled into capture duty: let main routing handle this bot.
        if (capturePct == 0 ||
            static_cast<uint8>(bot->GetGUID().GetCounter() % 100) >= capturePct)
        {
            std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
            s_WgCapturingWorkshop.erase(m_botGuidRaw);
            return false;
        }

        uint8 wsIdx = WgPickCapturableWorkshop(wg, bot, team);
        if (wsIdx == 0xFF)
        {
            std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
            s_WgCapturingWorkshop.erase(m_botGuidRaw);
            return false;
        }

        m_captureWsIdx = wsIdx;
    }

    // Heading to the assigned workshop.
    WgPath const& wsPath = *WG_WORKSHOPS[m_captureWsIdx].path;
    Position const pos(wsPath[0].x, wsPath[0].y, wsPath[0].z, 0.0f);

    // Set the latch when within the workshop combat zone. Here WG_WORKSHOP_ARRIVE_DIST is used instead of the standard
    // WG_OBJ_ARRIVE_DIST, so combat suppression lifts at a larger distance before the bot arrives to the workshop.
    if (!m_arrivedAtCapture &&
        bot->GetDistance(pos.GetPositionX(), pos.GetPositionY(), pos.GetPositionZ()) < WG_WORKSHOP_ARRIVE_DIST)
        m_arrivedAtCapture = true;

    // Keep navigating toward the workshop waypoint until FollowWgRoute says arrived.
    if (!m_arrivedAtCapture)
        FollowWgRoute(pos, false);

    // Track bots going to capture workshops so other actions can suppress combat while navigating.
    // Remove from tracking once latched (arrived at workshop).
    {
        std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
        if (!m_arrivedAtCapture)
            s_WgCapturingWorkshop.insert(m_botGuidRaw);
        else
            s_WgCapturingWorkshop.erase(m_botGuidRaw);
    }
    return true;   // Always true: main routing never fires as long as the bot is on a capture assignment.
}

// Returns true if the bot is currently heading to capture a workshop and should have combat suppressed.
bool WgCheckFlagAction::IsCapturingWorkshop(Player* bot)
{
    // InBattlefield() check prevents possible stale entry (a bot whose ResetBattleState never ran) from suppressing
    // combat outside Wintergrasp.
    // Suppression only applies to First Lieutenants. Lower rank bots are allowed combat engagement to reach higher rank.
    if (!bot || !bot->InBattlefield() || !bot->HasAura(SPELL_LIEUTENANT))
        return false;

    std::lock_guard<std::mutex> lock(s_WgCapturingWorkshopMtx);
    return s_WgCapturingWorkshop.count(bot->GetGUID().GetRawValue()) > 0;
}

// Fire all applicable vehicle spells at a destructible building GO.
// Sets "bg siege" position internally so CastVehicleSpell targets the GO. Returns true if any spell was cast.
static bool WgVehicleFireSpells(PlayerbotAI* botAI, Creature* creature,
                                Unit* vehicleBase, GameObject* go,
                                float dist, float rangedDist, bool inMelee)
{
    // TARGET_FLAG_DEST_LOCATION is a flag for spells that fire at some coordinate. Spells like Hurl Boulder as opposed to Ram.
    // CastVehicleSpell(spellId, vehicleBase) uses the "bg siege" position as the spell destination for DEST_LOCATION
    // spells when target == self.
    botAI->GetAiObjectContext()->GetValue<PositionMap&>("position")->Get()["bg siege"].Set(
        go->GetPositionX(), go->GetPositionY(), go->GetPositionZ(),
        vehicleBase->GetMapId());

    bool spellFired = false;
    for (uint32 i = 0; i < MAX_CREATURE_SPELLS; ++i)
    {
        uint32 spellId = creature->m_spells[i];
        if (!spellId)
            continue;
        SpellInfo const* info = sSpellMgr->GetSpellInfo(spellId);
        if (!info || info->IsPassive())
            continue;

        if (info->Targets & TARGET_FLAG_DEST_LOCATION)
        {
            if (dist <= rangedDist)
                spellFired |= botAI->CastVehicleSpell(spellId, vehicleBase);
        }
        else if (inMelee)
        {
            for (uint8 eff = 0; eff < MAX_SPELL_EFFECTS; ++eff)
            {
                if (info->Effects[eff].Effect == SPELL_EFFECT_GAMEOBJECT_DAMAGE)
                {
                    // Direct CastSpell at the GO's position: CastVehicleSpell can't target locations for non-DEST_LOCATION spells.
                    vehicleBase->CastSpell(
                        go->GetPositionX(), go->GetPositionY(), go->GetPositionZ(),
                        spellId, false);
                    spellFired = true;
                    break;
                }
            }
        }
    }
    return spellFired;
}

// ######################################################################################################################################### //
// FollowWgRoute is the main navigation function. It routes the bot to its objective using the A* waypoint graph. Several pathing systems
// were tested for Wintergrasp. Wintergrasp battles can have hundreds of PVP players at a time, many objectives to capture and interact with,
// and paths that need to be blocked as mmaps doesn't work correctly with bot pathing and destructible structures (bots can walk through them
// like they're not there). Given all that, A* was the most reliable pathing system.
// FollowWgRoute is used by the function Execute to determine how to get to Execute's objectives.
// ######################################################################################################################################### //
bool WgCheckFlagAction::FollowWgRoute(Position const& objective, bool checkPathBlock)
{
    BuildWgGraph();

    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg)
        return false;

    uint32 goalNode = WgFindNearestNode(
        objective.GetPositionX(), objective.GetPositionY(), objective.GetPositionZ());

    // Kiteo, his eyes closed: The bot's pathing data is stale.
    // Replan if route is empty, goal changed, index is out of range, or bot strayed far.
    bool hisEyesClosed = m_route.empty()    ||
                        m_route.back() != goalNode  ||
                        m_routeStep >= static_cast<uint32>(m_route.size());

    if (!hisEyesClosed)
    {
        WgNode const& cur = g_WgGraph[m_route[m_routeStep]];
        float dx = cur.x - bot->GetPositionX();
        float dy = cur.y - bot->GetPositionY();
        float dz = cur.z - bot->GetPositionZ();
        if (dx*dx + dy*dy + dz*dz > 100.0f * 100.0f)
            hisEyesClosed = true;
    }

    if (hisEyesClosed)
    {
        uint32 startNode = WgFindNearestNode(
            bot->GetPositionX(), bot->GetPositionY(), bot->GetPositionZ());
        m_route     = WgAStarPath(startNode, goalNode, wg, checkPathBlock);
        m_routeStep = 0;
        if (m_route.empty())
            return MoveTo(bot->GetMapId(),
                objective.GetPositionX(), objective.GetPositionY(), objective.GetPositionZ());
    }

    // The advance m_routeStep logic ensures that bots don't pause at each waypoint (most of the time) by making them re-route
    // to the following waypoint just as they are within WG_NODE_SWITCH_DIST of the next node. Higher values switch sooner,
    // giving the bot more runway to chain movement commands before arriving at a waypoint.
    while (m_routeStep + 1 < static_cast<uint32>(m_route.size()))
    {
        WgNode const& current = g_WgGraph[m_route[m_routeStep]];
        if (checkPathBlock && current.pathBlock != 0 && !WgIsBuildingDestroyed(wg, current.pathBlock))
            break;

        // noSkip nodes (waypoints) require the bot to physically reach them before advancing. Once arrived, advance immediately
        // and restart the loop to re-evaluate from the new position. Falling through would use stale `current` data and could
        // advance an extra step past the next node.
        if (current.noSkip)
        {
            float dx = current.x - bot->GetPositionX();
            float dy = current.y - bot->GetPositionY();
            float dz = current.z - bot->GetPositionZ();
            if (dx*dx + dy*dy + dz*dz > WG_NOSKIP_ARRIVE_DIST * WG_NOSKIP_ARRIVE_DIST)
                break;
            ++m_routeStep;
            continue;
        }

        WgNode const& next = g_WgGraph[m_route[m_routeStep + 1]];
        float dx = next.x - bot->GetPositionX();
        float dy = next.y - bot->GetPositionY();
        float dz = next.z - bot->GetPositionZ();

        // Half node switching distance for vehicles (they are slow relative to infantry)
        float switchDist = bot->GetVehicle() ? WG_NODE_SWITCH_DIST * 0.5f : WG_NODE_SWITCH_DIST;
        if (dx*dx + dy*dy + dz*dz <= switchDist * switchDist)
            ++m_routeStep;
        else
            break;
    }

    // Look-ahead: target one node past m_routeStep so the bot's MoveTo destination is always ahead, preventing stop-and-wait
    // at each node. Exceptions that target m_routeStep directly instead of looking ahead:
    //   1. A standing destructible wall that vehicles must engage.
    //   2. A noSkip node the bot hasn't reached yet - must land on it first.
    uint32 targetIdx;
    {
        WgNode const& node = g_WgGraph[m_route[m_routeStep]];

        bool mustVisit = false;

        // Exception 1: standing wall (vehicles only).
        if (checkPathBlock && node.pathBlock != 0)
            mustVisit = !WgIsBuildingDestroyed(wg, node.pathBlock);

        // Exception 2: noSkip node not yet reached.
        if (!mustVisit && node.noSkip)
        {
            float dx = node.x - bot->GetPositionX();
            float dy = node.y - bot->GetPositionY();
            float dz = node.z - bot->GetPositionZ();
            mustVisit = (dx*dx + dy*dy + dz*dz >
                WG_NOSKIP_ARRIVE_DIST * WG_NOSKIP_ARRIVE_DIST);
        }

        targetIdx = mustVisit
            ? m_routeStep
            : std::min(m_routeStep + 1,
                static_cast<uint32>(m_route.size() - 1));
    }

    WgNode const& target = g_WgGraph[m_route[targetIdx]];

    // Standing wall: hold position at the pathblock node. Building attacks are handled by the attacker/defender vehicle logic
    if (checkPathBlock && target.pathBlock != 0 && !WgIsBuildingDestroyed(wg, target.pathBlock))
        return true;

    // Check arrival at objective.
    if (bot->GetDistance(objective.GetPositionX(), objective.GetPositionY(), objective.GetPositionZ()) < WG_OBJ_ARRIVE_DIST)
        return false;

    // Jitter prevents IsDuplicateMove() from blocking repeated MoveTo calls to the same node. Infantry spread out (±1 yard).
    // Vehicles use minimal jitter (±0.5 yard) just to vary the coordinates enough to pass the 0.25 yard duplicate check without
    // meaningfully altering their path.
    float jx = target.x + (checkPathBlock ? frand(-0.5f, 0.5f) : frand(-1.0f, 1.0f));
    float jy = target.y + (checkPathBlock ? frand(-0.5f, 0.5f) : frand(-1.0f, 1.0f));
    // Clear the previous movement's delay so IsWaitingForLastMove() won't block this MoveTo. Without this, the bot is locked into its
    // prior movement for up to maxWaitForMove (5s) and can't be redirected mid-travel, causing visible pauses at each waypoint arrival.
    AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
    return MoveTo(bot->GetMapId(), jx, jy, target.z);
}

// ######################################################################################################################################### //
// Execute is the entry point of WgCheckFlagAction. It is called every tick by the bot AI to handle most of the bot's logic, by reading the
// current battle state and delegating to the appropriate section: attacker vehicles, defender vehicles, infantry defenders, or infantry
// attackers, returning false if nothing needs to be done this tick.
// ######################################################################################################################################### //
bool WgCheckFlagAction::Execute(Event /*event*/)
{
    BattlefieldWG* wg = GetBattlefieldWG();
    // Defensive reset. The main reset outside battle is done by BfStrategyCheckAction.
    if (!wg || !wg->IsWarTime())
    {
        ResetBattleState();
        return false;
    }

    if (bot->isDead())
    {
        ResetBattleState();
        return false;
    }

    Vehicle* vehicle  = bot->GetVehicle();
    bool isDriver     = botAI->IsInVehicle(true);
    bool inVehicle    = vehicle != nullptr;
    bool isAttacker   = (bot->GetTeamId() != wg->GetDefenderTeam());

    // Passengers let the driver navigate and do nothing.
    // In practice, there shouldn't be a passive passenger due to the changes in VehicleAction.cpp.
    if (inVehicle && !isDriver)
        return false;

    // Siege engine turret operators are handled by WgFireCannonAction, not here.
    // Unclear how IsInVehicle(true) parses the turrets, so this is just in-case.
    if (inVehicle)
    {
        uint32 entry = vehicle->GetBase()->GetEntry();
        if (entry == NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_ALLIANCE ||
            entry == NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_HORDE)
            return false;
    }

    // Shaka, when the walls fell. Check if the fortress has been breached.
    bool whenTheWallsFell = WgIsBuildingDestroyed(wg, WG_WS_FORTRESS_GATE) ||
                            WgIsBuildingDestroyed(wg, WG_WS_EAST_WALL)     ||
                            WgIsBuildingDestroyed(wg, WG_WS_WEST_WALL);

    // Uzani, his army at stage: Bots will first stage their fight to either the east or west of the fortress.
    Position const& hisArmyAtStage = (wg->GetAttackerTeam() == TEAM_ALLIANCE)
        ? WG_OBJ_INI_CONF_EAST : WG_OBJ_INI_CONF_WEST;

    // Defender vehicles reserved to guard the fortress (VEHICLE_FORT_GUARD) will either be at hisGuardAtStage, which is a closer
    // point to the walls than hisArmyAtStage, or they will be at hisGuardAtGate, which is right in front of Fortress Gate.
    // If there's overflow beyond VEHICLE_FORT_GUARD for EACH of these position, then it goes to hisGuardAtOtherSide which is the
    // side opposing hisGuardAtStage.
    Position const& hisGuardAtStage     = (wg->GetAttackerTeam() == TEAM_ALLIANCE)
        ? WG_OBJ_DEF_GUARD_EAST : WG_OBJ_DEF_GUARD_WEST;
    Position const& hisGuardAtGate      = WG_OBJ_DEF_GUARD_GATE;
    Position const& hisGuardAtOtherSide = (wg->GetAttackerTeam() == TEAM_ALLIANCE)
        ? WG_OBJ_DEF_GUARD_WEST : WG_OBJ_DEF_GUARD_EAST;

    // ################# //
    // Attacker Vehicles
    // ################# //

    // Release attacker vehicle state if vehicle was destroyed but bot survived.
    if (!isDriver && m_atkVehiclePhase > 0)
    {
        m_atkVehiclePhase = 0;
        m_route.clear();
        m_routeStep = 0;
    }

    if (isDriver && isAttacker)
    {
        // Clear workshop state on vehicle entry.
        if (m_atkGoingToWorkshop)
        {
            m_atkGoingToWorkshop = false;
            --s_WgAtkGoingToWorkshop;
            m_atkVehiclePhase = 0;
            m_route.clear();
        }

        // Phase transition logic. Phase 0 = decision: lock path once, immediately.
        // Staging phases (1, 3, 5) advance on arrival; attack phases advance on destruction.
        // Attacking phases (2, 4, 6) only check their own obstacle.
        switch (m_atkVehiclePhase)
        {
            case 0:     // Decision: choose path immediately on vehicle entry, and stick to it
            {
                if (whenTheWallsFell)
                    m_atkVehiclePhase = 7; // Skip to central wall

                else if (urand(0, 4) < 2)  // 2 out of 5 chances to attack the gate.
                    m_atkVehiclePhase = 1; // Path A: head to the central staging area.

                else
                {
                    float dE = bot->GetDistance(WG_OBJ_INI_CONF_EAST.GetPositionX(),
                                                WG_OBJ_INI_CONF_EAST.GetPositionY(),
                                                WG_OBJ_INI_CONF_EAST.GetPositionZ());
                    float dW = bot->GetDistance(WG_OBJ_INI_CONF_WEST.GetPositionX(),
                                                WG_OBJ_INI_CONF_WEST.GetPositionY(),
                                                WG_OBJ_INI_CONF_WEST.GetPositionZ());
                    // Path B: otherwise determine which is closer, the eastern or western
                    // staging area.
                    m_atkVehiclePhase = (dE <= dW) ? 3 : 5;
                }
                break;
            }
            case 1:     // Path A: wait until at the central staging, then check the gate
            {
                float d = bot->GetDistance(WG_OBJ_CENTRAL_STAGE.GetPositionX(),
                                           WG_OBJ_CENTRAL_STAGE.GetPositionY(),
                                           WG_OBJ_CENTRAL_STAGE.GetPositionZ());
                if (d < WG_STAGING_ARRIVE_DIST)
                    m_atkVehiclePhase = WgIsBuildingDestroyed(wg, WG_WS_FORTRESS_GATE) ? 7 : 2;
                break;
            }
            case 2:     // Path A: attacking the gate; advance only when the gate falls
                if (WgIsBuildingDestroyed(wg, WG_WS_FORTRESS_GATE))
                    m_atkVehiclePhase = 7;
                break;
            case 3:     // Path B-East: wait until at the eastern staging, then check the east wall
            {
                float d = bot->GetDistance(WG_OBJ_INI_CONF_EAST.GetPositionX(),
                                           WG_OBJ_INI_CONF_EAST.GetPositionY(),
                                           WG_OBJ_INI_CONF_EAST.GetPositionZ());
                if (d < WG_STAGING_ARRIVE_DIST)
                    m_atkVehiclePhase = WgIsBuildingDestroyed(wg, WG_WS_EAST_WALL) ? 7 : 4;
                break;
            }
            case 4:     // Path B-East: attacking the east wall; advance only when the east wall falls
                if (WgIsBuildingDestroyed(wg, WG_WS_EAST_WALL))
                    m_atkVehiclePhase = 7;
                break;
            case 5:     // Path B-West: wait until at the western staging, then check the west wall
            {
                float d = bot->GetDistance(WG_OBJ_INI_CONF_WEST.GetPositionX(),
                                           WG_OBJ_INI_CONF_WEST.GetPositionY(),
                                           WG_OBJ_INI_CONF_WEST.GetPositionZ());
                if (d < WG_STAGING_ARRIVE_DIST)
                    m_atkVehiclePhase = WgIsBuildingDestroyed(wg, WG_WS_WEST_WALL) ? 7 : 6;
                break;
            }
            case 6:     // Path B-West: attacking the west wall; advance only when the west wall falls
                if (WgIsBuildingDestroyed(wg, WG_WS_WEST_WALL))
                    m_atkVehiclePhase = 7;
                break;
            case 7:     // Outer entry fallen; attacking central wall; advance to vault door once it falls
                if (WgIsBuildingDestroyed(wg, WG_WS_CENTRAL_WALL))
                    m_atkVehiclePhase = 8;
                break;
            default:    // Phase 8+: terminal, no more transitions. FollowWgRoute handles routing and attacking the vault door
                break;
        }

        static Position const* const WG_VEHICLE_PHASES[] = {
            nullptr,                // 0  Decision phase. Never used as a destination
            &WG_OBJ_CENTRAL_STAGE,  // 1  Path A staging
            &WG_OBJ_FORTRESS_GATE,  // 2  Path A gate attack
            &WG_OBJ_INI_CONF_EAST,  // 3  Path B-East staging
            &WG_OBJ_EAST_WALL,      // 4  Path B-East wall attack
            &WG_OBJ_INI_CONF_WEST,  // 5  Path B-West staging
            &WG_OBJ_WEST_WALL,      // 6  Path B-West wall attack
            &WG_OBJ_CENTRAL_WALL,   // 7  All paths converge here after breaching outer entry
            &WG_OBJ_VAULT_DOOR,     // 8  Terminal: attack vault door until battle ends
        };
        static constexpr uint8 WG_VEHICLE_PHASE_COUNT = 9;  // Sanity guard. Shouldn't happen in practice.
        if (m_atkVehiclePhase >= WG_VEHICLE_PHASE_COUNT)
            return false;

        Position const* dest = WG_VEHICLE_PHASES[m_atkVehiclePhase];
        if (!dest)
            return false;

        // Attack phases: fire at the target building while approaching.
        // Maps attack phases to their building WorldState; 0 = staging/no target.
        static constexpr uint32 ATK_PHASE_WS[] = {
            0,                      // 0  Decision
            0,                      // 1  Path A staging
            WG_WS_FORTRESS_GATE,    // 2  Gate attack
            0,                      // 3  Path B-East staging
            WG_WS_EAST_WALL,        // 4  East wall attack
            0,                      // 5  Path B-West staging
            WG_WS_WEST_WALL,        // 6  West wall attack
            WG_WS_CENTRAL_WALL,     // 7  Central wall attack
            WG_WS_VAULT_DOOR,       // 8  Vault door attack
        };
        uint32 targetWs = (m_atkVehiclePhase < 9) ? ATK_PHASE_WS[m_atkVehiclePhase] : 0;
        if (targetWs != 0 && !WgIsBuildingDestroyed(wg, targetWs))
        {
            uint32 goEntry = WgWsToGoEntry(targetWs);
            GameObject* go = bot->FindNearestGameObject(goEntry, WG_WALL_SCAN_DIST);
            if (go)
            {
                Unit* vehicleBase = vehicle ? vehicle->GetBase() : nullptr;
                Creature* creature = vehicleBase ? vehicleBase->ToCreature() : nullptr;
                if (!creature)
                    return false;

                float dist = bot->GetDistance(
                    go->GetPositionX(), go->GetPositionY(), go->GetPositionZ());
                bool atWall = (dist <= WG_WALL_MELEE_DIST);

                WgVehicleFireSpells(
                    botAI, creature, vehicleBase, go,
                    dist, WG_WALL_SCAN_DIST, atWall);

                if (!atWall)
                {
                    AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
                    return MoveTo(bot->GetMapId(),
                        dest->GetPositionX() + frand(-0.1f, 0.1f),
                        dest->GetPositionY() + frand(-0.1f, 0.1f),
                        dest->GetPositionZ());
                }
                return true;
            }

            // Wall GO not found within range - clear any stale siege position.
            AI_VALUE(PositionMap&, "position")["bg siege"].Reset();
        }

        return FollowWgRoute(*dest, true);
    }

    // ################# //
    // Defender Vehicles
    // ################# //

    // Release defender vehicle state if vehicle was destroyed but bot survived.
    if (!isDriver && m_defVehiclePhase > 0)
    {
        if (m_isFortGuard)
        {
            m_isFortGuard = false;
            --s_WgFortGuardVehicles;
            if (m_defGuardFortress == 1)      --s_WgFortGuardAtStage;
            else if (m_defGuardFortress == 2) --s_WgFortGuardAtGate;
            else if (m_defGuardFortress == 3) --s_WgFortGuardAtOtherSide;
            m_defGuardFortress = 0;
        }
        if (m_isTowerAttacker)
        {
            m_isTowerAttacker = false;
            --s_WgTowerSquad[m_targetTowerIdx];
            m_targetTowerIdx = 0xFF;
        }
        m_defVehiclePhase = 0;
        m_route.clear();
        m_routeStep = 0;
    }

    if (isDriver && !isAttacker)
    {
        // Clear workshop state on vehicle entry.
        if (m_defGoingToWorkshop)
        {
            m_defGoingToWorkshop = false;
            --s_WgDefGoingToWorkshop;
            m_defVehiclePhase = 0;
            m_route.clear();
        }

        // Defender tower targets paired with their WorldState.
        struct DefTowerTarget { Position const* pos; uint32 ws; };
        static DefTowerTarget const DEF_TOWERS[] = {
            { &WG_OBJ_SE_TOWER,    WG_WS_TOWER_SE    },
            { &WG_OBJ_SOUTH_TOWER, WG_WS_TOWER_SOUTH },
            { &WG_OBJ_SW_TOWER,    WG_WS_TOWER_SW    },
        };
        static constexpr uint8 DEF_TOWER_COUNT = 3;

        // Phase transition logic.
        switch (m_defVehiclePhase)
        {
            // Decision: detect if vehicle came from a fortress workshop (needs teleporter detour).
            // This is legacy code from when defender bots would get vehicles from inside or outside the fortress, depending on
            // the state of attack towers and whenTheWallsFell. It might be reused in more complex tactics. Now however, defenders
            // always get vehicles from the fortess to streamline everything. The workshop detection logic is kept for now though.
            case 0:
            {
                // Assign fort guard role: reset any previous assignment then claim a slot if one is available.
                {
                    // Locked so the concurrent sequence (check then claim) can't overshoot VEHICLE_FORT_GUARD, when multiple
                    // drivers deciding at once.
                    std::lock_guard<std::mutex> lock(s_WgFortGuardPosMtx);
                    if (m_isFortGuard)
                    {
                        m_isFortGuard = false;
                        --s_WgFortGuardVehicles;
                    }
                    if (s_WgFortGuardVehicles < VEHICLE_FORT_GUARD)
                    {
                        m_isFortGuard = true;
                        ++s_WgFortGuardVehicles;
                    }
                }

                // Use teleporter proximity to detect which fortress workshop the vehicle came from, rather than m_workshopIdx,
                // which can be updated by the on-foot routing logic between the summon and the vehicle entry.
                float dW = bot->GetDistance(WG_OBJ_WS_TELE_WEST.GetPositionX(),
                                            WG_OBJ_WS_TELE_WEST.GetPositionY(),
                                            WG_OBJ_WS_TELE_WEST.GetPositionZ());
                float dE = bot->GetDistance(WG_OBJ_WS_TELE_EAST.GetPositionX(),
                                            WG_OBJ_WS_TELE_EAST.GetPositionY(),
                                            WG_OBJ_WS_TELE_EAST.GetPositionZ());
                if (dW < FORTRESS_WS_DETECT_DIST)
                {
                    m_workshopIdx     = 4;
                    m_defVehiclePhase = 1;  // Go to WG_OBJ_WS_TELE_WEST first
                }
                else if (dE < FORTRESS_WS_DETECT_DIST)
                {
                    m_workshopIdx     = 5;
                    m_defVehiclePhase = 1;  // Go to WG_OBJ_WS_TELE_EAST first
                }
                else
                    m_defVehiclePhase = m_isFortGuard ? 4 : 2;  // Fort guards skip towers. Others proceed to tower targeting.
                break;
            }
            case 1:     // Teleporter: navigate to the fortress vehicle teleporter
            {
                Vehicle* veh = bot->GetVehicle();
                Unit* vBase = veh ? veh->GetBase() : nullptr;
                // Mirab, his sails unfurled: Vehicle has passed through the teleporter and launched outside.
                bool hisSailsUnfurled = vBase && vBase->HasAura(SPELL_VEHICLE_TELEPORT);
                if (hisSailsUnfurled)
                {
                    m_defVehiclePhase = m_isFortGuard ? 4 : 2;  // Fort guards skip towers. Others proceed to tower targeting.
                    break;
                }
                break;
            }
            case 2:     // Tower targeting: No more than MAX_TOWER_SQUAD vehicles are assigned to attack a standing tower.
                        // Excess number of vehicles are assigned to another standing tower that has < MAX_TOWER_SQUAD vehicles,
                        // otherwise they fall back as fortress guards.
            {
                // Sweep order: nearest end tower (SE or SW) first, through South, to the other end. Take the first tower with a
                // free squad slot. An end to end sweep, instead of simply going to the nearest tower, ensures that there won't
                // be a situation where defender vehicles go to the South tower first, then SW, then all the way back to SE.
                float dSE = bot->GetDistance(
                    DEF_TOWERS[0].pos->GetPositionX(),
                    DEF_TOWERS[0].pos->GetPositionY(),
                    DEF_TOWERS[0].pos->GetPositionZ());
                float dSW = bot->GetDistance(
                    DEF_TOWERS[2].pos->GetPositionX(),
                    DEF_TOWERS[2].pos->GetPositionY(),
                    DEF_TOWERS[2].pos->GetPositionZ());
                uint8 sweepA = (dSW < dSE) ? 2 : 0;    // Near end
                uint8 sweepC = (dSW < dSE) ? 0 : 2;    // Far end
                uint8 const sweepOrder[DEF_TOWER_COUNT] = { sweepA, 1, sweepC };

                uint8 nextIdx = 0xFF;
                {
                    std::lock_guard<std::mutex> lock(s_WgTowerSquadMtx);
                    for (uint8 i = 0; i < DEF_TOWER_COUNT; ++i)
                    {
                        uint8 idx = sweepOrder[i];
                        if (WgIsBuildingDestroyed(wg, DEF_TOWERS[idx].ws))
                            continue;
                        if (s_WgTowerSquad[idx] < MAX_TOWER_SQUAD)
                        {
                            nextIdx = idx;
                            ++s_WgTowerSquad[nextIdx];
                            break;
                        }
                    }
                }

                if (nextIdx == 0xFF)
                {
                    // All squads full or all towers destroyed: fall back to fort guard.
                    if (!m_isFortGuard)
                    {
                        m_isFortGuard = true;
                        ++s_WgFortGuardVehicles;
                    }
                    m_defVehiclePhase = 4;
                }
                else
                {
                    m_targetTowerIdx  = nextIdx;
                    m_isTowerAttacker = true;
                    m_defVehiclePhase = 3;
                }
                break;
            }
            case 3:     // Attack tower: route to target, reassign squad on destruction
            {
                if (WgIsBuildingDestroyed(wg, DEF_TOWERS[m_targetTowerIdx].ws))
                {
                    // Release and reassign under one lock: each bot's release is visible to the others' scans, so surviving
                    // squads show accurate post-destruction counts.
                    uint8 nextIdx = 0xFF;
                    {
                        std::lock_guard<std::mutex> lock(s_WgTowerSquadMtx);
                        if (m_isTowerAttacker)
                        {
                            m_isTowerAttacker = false;
                            --s_WgTowerSquad[m_targetTowerIdx];
                        }
                        for (uint8 i = 0; i < DEF_TOWER_COUNT; ++i)
                        {
                            if (WgIsBuildingDestroyed(wg, DEF_TOWERS[i].ws))
                                continue;
                            if (s_WgTowerSquad[i] < MAX_TOWER_SQUAD)
                            {
                                if (nextIdx == 0xFF || s_WgTowerSquad[i] < s_WgTowerSquad[nextIdx])
                                    nextIdx = i;
                            }
                        }
                        if (nextIdx != 0xFF)
                            ++s_WgTowerSquad[nextIdx];
                    }

                    if (nextIdx != 0xFF)
                    {
                        m_targetTowerIdx  = nextIdx;
                        m_isTowerAttacker = true;
                        // Stay in phase 3, new target set above.
                    }
                    else
                    {
                        // No room in any surviving squad: fall back to fort guard.
                        m_targetTowerIdx = 0xFF;
                        if (!m_isFortGuard)
                        {
                            m_isFortGuard = true;
                            ++s_WgFortGuardVehicles;
                        }
                        m_defVehiclePhase = 4;
                    }
                    break;
                }
                break;
            }
            // Phase 4: all towers destroyed, now go fight the attackers.
            // If the fortress is not breached, defender vehicles should protect the front and side of the fortress.
            // If the fortress is breached, go to the Front Court.
            // If the fortress is breached and the Central Wall has fallen, go to the Central Court.
            default:
            {
                AI_VALUE(PositionMap&, "position")["bg siege"].Reset();
                if (!whenTheWallsFell)
                {
                    // Assign a guard position, re-evaluating every GUARD_REBALANCE_PERIOD to rebalance as vehicles join or are lost.
                    // The simpler bot GUID distribution is intentionally not used. VEHICLE_FORT_GUARD is a small number, and GUID
                    // distribution is unlikely to be precise on this scale.
                    // The full sequence is to distribute the minimum number of VEHICLE_FORT_GUARD vehicles between hisGuardAtStage
                    // and hisGuardAtGate. If there's more available, the number continues to equalize until the number of vehicles at
                    // each of the positions reaches VEHICLE_FORT_GUARD. The excess gets sent to hisGuardAtOtherSide, and pulled back if
                    // needed again at hisGuardAtStage or hisGuardAtGate.
                    uint32 now = getMSTime();
                    if (m_defGuardFortress == 0 || now - m_defGuardFortressTime >= GUARD_REBALANCE_PERIOD)
                    {
                        std::lock_guard<std::mutex> lock(s_WgFortGuardPosMtx);
                        if (m_defGuardFortress == 1)      --s_WgFortGuardAtStage;
                        else if (m_defGuardFortress == 2) --s_WgFortGuardAtGate;
                        else if (m_defGuardFortress == 3) --s_WgFortGuardAtOtherSide;
                        bool stageIsPreferred        = (s_WgFortGuardAtStage <= s_WgFortGuardAtGate);
                        std::atomic<int32_t>& countPreferred = stageIsPreferred ? s_WgFortGuardAtStage : s_WgFortGuardAtGate;
                        std::atomic<int32_t>& countFallback  = stageIsPreferred ? s_WgFortGuardAtGate  : s_WgFortGuardAtStage;
                        uint8 slotPreferred = stageIsPreferred ? 1 : 2;
                        uint8 slotFallback  = stageIsPreferred ? 2 : 1;
                        if (countPreferred < VEHICLE_FORT_GUARD)
                        { m_defGuardFortress = slotPreferred; ++countPreferred; }
                        else if (countFallback < VEHICLE_FORT_GUARD)
                        { m_defGuardFortress = slotFallback; ++countFallback; }
                        else
                        { m_defGuardFortress = 3; ++s_WgFortGuardAtOtherSide; }
                        m_defGuardFortressTime = now;
                    }
                    Position const& guardPos = (m_defGuardFortress == 1) ? hisGuardAtStage
                                            : (m_defGuardFortress == 2) ? hisGuardAtGate
                                                                        : hisGuardAtOtherSide;
                    return FollowWgRoute(guardPos, true);
                }
                //Fortress breached: release guard slot if held.
                if (m_defGuardFortress == 1)
                    --s_WgFortGuardAtStage;
                else if (m_defGuardFortress == 2)
                    --s_WgFortGuardAtGate;
                else if (m_defGuardFortress == 3)
                    --s_WgFortGuardAtOtherSide;
                m_defGuardFortress = 0;
                Position const& defVehObj = WgIsBuildingDestroyed(wg, WG_WS_CENTRAL_WALL) ? WG_OBJ_CENTRAL_COURT
                                                                                           : WG_OBJ_FRONT_COURT;
                return FollowWgRoute(defVehObj, true);
            }
        }

        // Determine destination for current phase.
        Position const* dest = nullptr;
        switch (m_defVehiclePhase)
        {
            case 1:
                dest = (m_workshopIdx == 4) ? &WG_OBJ_WS_TELE_WEST : &WG_OBJ_WS_TELE_EAST;
                break;
            case 2:
            case 3:
                if (m_targetTowerIdx < DEF_TOWER_COUNT)
                    dest = DEF_TOWERS[m_targetTowerIdx].pos;
                break;
            default:
                break;
        }
        if (!dest)
            return false;

        // Phase 3: engage the tower when close enough.
        // Cast every non-passive vehicle spell at the GO position so that SCHOOL_DAMAGE / WEAPON_DAMAGE effects hit
        // the destructible building.
        if (m_defVehiclePhase == 3 && m_targetTowerIdx < DEF_TOWER_COUNT)
        {
            uint32 towerEntry = WgWsToGoEntry(DEF_TOWERS[m_targetTowerIdx].ws);
            GameObject* go = bot->FindNearestGameObject(towerEntry, WG_TOWER_SCAN_DIST);
            if (go)
            {
                Unit* vehicleBase = vehicle ? vehicle->GetBase() : nullptr;
                Creature* creature = vehicleBase ? vehicleBase->ToCreature() : nullptr;
                if (!creature)
                    return false;

                float dist = bot->GetDistance(
                    go->GetPositionX(), go->GetPositionY(), go->GetPositionZ());
                bool atTower = (dist <= WG_TOWER_MELEE_DIST);

                bool spellFired = WgVehicleFireSpells(
                    botAI, creature, vehicleBase, go,
                    dist, WG_TOWER_SCAN_DIST, atTower);

                if (!atTower)
                {
                    AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
                    return MoveTo(bot->GetMapId(),
                        dest->GetPositionX() + frand(-0.1f, 0.1f),
                        dest->GetPositionY() + frand(-0.1f, 0.1f),
                        dest->GetPositionZ());
                }
                return spellFired;
            }
        }

        // Phase 1 (teleporter): direct MoveTo to the nearby teleporter.
        if (m_defVehiclePhase == 1)
        {
            AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
            return MoveTo(bot->GetMapId(),
                dest->GetPositionX() + frand(-0.1f, 0.1f),
                dest->GetPositionY() + frand(-0.1f, 0.1f),
                dest->GetPositionZ());
        }

        return FollowWgRoute(*dest, true);
    }

    // Infantry: Clear any leftover siege position from vehicle driving.
    AI_VALUE(PositionMap&, "position")["bg siege"].Reset();

    // ################## //
    // Infantry Defenders
    // ################## //
    if (!isAttacker)
    {
        // First Lieutenant defenders head to a fortress workshop for a vehicle.
        if (bot->HasAura(SPELL_LIEUTENANT))
        {
            TeamId team = bot->GetTeamId();
            uint32 dataVeh = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_VEHICLE_H : BATTLEFIELD_WG_DATA_VEHICLE_A;
            uint32 dataMax = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_MAX_VEHICLE_H : BATTLEFIELD_WG_DATA_MAX_VEHICLE_A;

            // Before fortress breach, the fortress is exit-only. Bots should not be able to access the fortress workshops.
            // Only route there if the bot can already reach the central court (which happens with respawn at Fortress Graveyard).
            // After fortress breach, all defenders can enter freely.
            bool canEnterFortress = whenTheWallsFell;
            if (!canEnterFortress)
            {
                uint32 sN = WgFindNearestNode(bot->GetPositionX(), bot->GetPositionY(), bot->GetPositionZ());
                uint32 gN = WgFindNearestNode(
                    WG_OBJ_CENTRAL_COURT.GetPositionX(),
                    WG_OBJ_CENTRAL_COURT.GetPositionY(),
                    WG_OBJ_CENTRAL_COURT.GetPositionZ());
                // sN == gN means bot is already inside the fortress and near WG_OBJ_CENTRAL_COURT.
                canEnterFortress = (sN == gN) || !WgAStarPath(sN, gN, wg).empty();
            }

            if (canEnterFortress)
            {
                // Alliance uses Fort. W (wsIdx 4); Horde uses Fort. E (wsIdx 5), to increase chances they run into enemy vehicles.
                uint8 wsIdx = (team == TEAM_ALLIANCE) ? 4 : 5;

                if (wg->GetWorkshopTeam(WG_WORKSHOPS[wsIdx].workshopId) == team)
                {
                    uint32 vehCount   = wg->GetData(dataVeh);
                    uint32 vehMax     = wg->GetData(dataMax);
                    uint32 availSlots = (vehMax > vehCount) ? vehMax - vehCount : 0;

                    if (availSlots > 0)
                    {
                        // Pre-breach: Bot has rank & !whenTheWallsFell & canEnterFortress, that means it has recently respawned at the
                        // fortress graveyard and it's inside the fortress. If there's a vehicle slot, it should go get one.
                        // Post-breach: everyone can freely enter the fortress, so limit concurrent trips with WS_GO_DEF_MULTIPLIER.
                        bool capOk = !whenTheWallsFell ||
                                     m_defGoingToWorkshop ||
                                     (s_WgDefGoingToWorkshop < static_cast<int32>(WS_GO_DEF_MULTIPLIER * availSlots));
                        if (capOk)
                        {
                            if (!m_defGoingToWorkshop)
                            {
                                m_defGoingToWorkshop = true;
                                ++s_WgDefGoingToWorkshop;
                            }
                            m_workshopIdx = wsIdx;

                            WgPath const& wsPath = *WG_WORKSHOPS[wsIdx].path;
                            Position const engineerPos(wsPath[0].x, wsPath[0].y, wsPath[0].z, 0.0f);
                            return FollowWgRoute(engineerPos, false);
                        }
                    }
                }
            }
        }

        // Not heading to a workshop. Release slot if held.
        if (m_defGoingToWorkshop)
        {
            m_defGoingToWorkshop = false;
            --s_WgDefGoingToWorkshop;
        }

        // Divert a percentage of defenders to capture hostile/neutral workshops.
        if (TryCaptureWorkshop(wg))
            return true;

        // Defender Bot arrived at capture workshop and yielding to combat. Also don't reroute to another objective.
        if (m_captureWsIdx != 0xFF && m_arrivedAtCapture)
            return false;

        // Yield to combat: Stop marching to objective while actively fighting, unless bot is heading to capture a workshop.
        // The narrower scope GetVictim is used instead of IsInCombat, so bots don't pickup new needless fights when there's
        // more useful things to do.
        if (bot->GetVictim())
            return false;

        // Chenza, go to the court, the court of silence. Abandon conflict at hisArmyAtStage if the fortress is breached.
        return FollowWgRoute(whenTheWallsFell ? WG_OBJ_FRONT_COURT : hisArmyAtStage, false);
    }

    // ################## //
    // Infantry Attackers
    // ################## //
    if (bot->HasAura(SPELL_LIEUTENANT))
    {
        TeamId team = bot->GetTeamId();
        uint32 dataVeh = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_VEHICLE_H : BATTLEFIELD_WG_DATA_VEHICLE_A;
        uint32 dataMax = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_MAX_VEHICLE_H : BATTLEFIELD_WG_DATA_MAX_VEHICLE_A;

        uint32 vehCount   = wg->GetData(dataVeh);
        uint32 vehMax     = wg->GetData(dataMax);
        uint32 availSlots = (vehMax > vehCount) ? vehMax - vehCount : 0;

        if (availSlots > 0)
        {
            // Workshop priority for attackers: First friendly wins. Prioritize getting a vehicle from the south of the map.
            static constexpr uint8 ATK_WS_PRIORITY_A[] = { 0, 2, 1, 3 };    // Alliance: NE, SE, NW, SW.
            static constexpr uint8 ATK_WS_PRIORITY_H[] = { 1, 3, 0, 2 };    // Horde:    NW, SW, NE, SE.
            uint8 const* ATK_WS_PRIORITY = (team == TEAM_ALLIANCE) ? ATK_WS_PRIORITY_A : ATK_WS_PRIORITY_H;

            uint8 bestWsIdx = 0xFF;
            for (uint8 p = 0; p < WG_CAPTURABLE_WORKSHOP_COUNT && bestWsIdx == 0xFF; ++p)
            {
                uint8 wsIdx = ATK_WS_PRIORITY[p];
                if (wg->GetWorkshopTeam(WG_WORKSHOPS[wsIdx].workshopId) != team)
                    continue;

                bestWsIdx = wsIdx;
            }

            if (bestWsIdx != 0xFF)
            {
                bool capOk = m_atkGoingToWorkshop ||
                             (s_WgAtkGoingToWorkshop < static_cast<int32>(WS_GO_ATK_MULTIPLIER * availSlots));
                if (capOk)
                {
                    if (!m_atkGoingToWorkshop)
                    {
                        m_atkGoingToWorkshop = true;
                        ++s_WgAtkGoingToWorkshop;
                    }
                    m_workshopIdx = bestWsIdx;

                    WgPath const& wsPath = *WG_WORKSHOPS[bestWsIdx].path;
                    Position const engineerPos(wsPath[0].x, wsPath[0].y, wsPath[0].z, 0.0f);
                    return FollowWgRoute(engineerPos, false);
                }
            }
        }
    }

    // Not heading to a workshop. Release slot if held.
    if (m_atkGoingToWorkshop)
    {
        m_atkGoingToWorkshop = false;
        --s_WgAtkGoingToWorkshop;
    }

    // Divert a percentage of attackers to capture hostile/neutral workshops.
    if (TryCaptureWorkshop(wg))
        return true;

    // Attacker bot arrived at capture workshop and yielding to combat. Also don't reroute to another objective.
    if (m_captureWsIdx != 0xFF && m_arrivedAtCapture)
        return false;

    // Yield to combat: Stop marching to objective while actively fighting, unless bot is heading to capture a workshop.
    // The narrower scope GetVictim is used instead of IsInCombat, so bots don't pickup new needless fights when there's
    // more useful things to do.
    if (bot->GetVictim())
        return false;

    // Uzani, his army with fists closed: Attackers have breached the fortress. Go into the fortress. Abandon conflict at hisArmyAtStage.
    return FollowWgRoute(whenTheWallsFell ? WG_OBJ_CENTRAL_COURT : hisArmyAtStage, false);
}

// Summons a vehicle by interacting with the workshop engineer: Siege engines for attackers, demolishers for defenders.
bool WgSummonVehicleAction::Execute(Event /*event*/)
{
    // Check that WG is in wartime.
    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg || !wg->IsWarTime())
        return false;

    // Must have First Lieutenant rank to summon a Demolisher.
    if (!bot->HasAura(SPELL_LIEUTENANT))
        return false;

    // Already in a vehicle; EnterVehicleAction will handle entry.
    if (bot->GetVehicle())
        return false;

    TeamId team = bot->GetTeamId();
    bool isAttacker = (team != wg->GetDefenderTeam());
    uint32 dataVeh = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_VEHICLE_H : BATTLEFIELD_WG_DATA_VEHICLE_A;
    uint32 dataMax = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_MAX_VEHICLE_H : BATTLEFIELD_WG_DATA_MAX_VEHICLE_A;
    if (wg->GetData(dataVeh) >= wg->GetData(dataMax))
        return false;

    // Find the workshop engineer NPC nearby. WgCheckFlagAction navigates the bot
    // to the workshop, and they should easily find the engineer within scan range.
    uint32 engineerEntry = (team == TEAM_HORDE) ? NPC_WG_GOBLIN_MECHANIC : NPC_WG_GNOMISH_ENGINEER;
    Creature* engineer = bot->FindNearestCreature(engineerEntry, ENGINEER_SCAN_RANGE, true);
    if (!engineer)
        return false;

    // Defenders may only summon at a fortress workshop (WG_WORKSHOPS indices 4 and 5). The defender vehicle phase
    // system assumes a fortress-teleporter launch, so reject summoning at an outer workshop a defender happens to
    // stand near. ENGINEER_SCAN_RANGE here doubles as the fortress-waypoint match tolerance.
    if (!isAttacker)
    {
        WgPath const& wsWest = *WG_WORKSHOPS[4].path;
        WgPath const& wsEast = *WG_WORKSHOPS[5].path;
        bool atFortressWorkshop =
            engineer->GetDistance(wsWest[0].x, wsWest[0].y, wsWest[0].z) < ENGINEER_SCAN_RANGE ||
            engineer->GetDistance(wsEast[0].x, wsEast[0].y, wsEast[0].z) < ENGINEER_SCAN_RANGE;
        if (!atFortressWorkshop)
            return false;
    }

    if (bot->GetDistance(engineer) > INTERACTION_DISTANCE)
        return MoveTo(engineer);

    // Dismount before interacting. Remounting is blocked in CheckMountStateAction.cpp.
    if (bot->IsMounted())
    {
        if (bot->isMoving())
            bot->StopMoving();

        WorldPacket emptyPacket;
        bot->GetSession()->HandleCancelMountAuraOpcode(emptyPacket);
    }

    // Open the gossip menu.
    WorldPacket hello;
    hello << engineer->GetGUID();
    bot->GetSession()->HandleGossipHelloOpcode(hello);

    if (!bot->PlayerTalkClass)
        return false;

    GossipMenu& menu = bot->PlayerTalkClass->GetGossipMenu();

    // When at First Lieutenant rank the engineer shows 3 options:
    //   slot 0 = Catapult
    //   slot 1 = Demolisher
    //   slot 2 = Siege Engine
    // Attackers use siege engines to breach walls. Defenders use demolishers.
    uint32 slot = isAttacker ? 2 : 1;

    if (!menu.GetItem(slot))
        return false;

    WorldPacket select;
    std::string code;
    select << engineer->GetGUID();
    select << menu.GetMenuId() << slot;
    select << code;
    bot->GetSession()->HandleGossipSelectOptionOpcode(select);

    return true;
}

void WgMountTowerCannonAction::ResetCannonState()
{
    m_cannonScanTime = 0;
    m_targetCannon.Clear();
}

bool WgMountTowerCannonAction::Execute(Event /*event*/)
{
    // Already got in a cannon or a vehicle.
    if (bot->GetVehicle())
        return false;

    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg || !wg->IsWarTime())
    {
        ResetCannonState();
        return false;
    }
    if (bot->isDead())
    {
        ResetCannonState();
        return false;
    }
    if (bot->GetTeamId() != wg->GetDefenderTeam())
        return false;

    // If the bot has chosen a cannon, navigate to it and board it.
    if (!m_targetCannon.IsEmpty())
    {
        Creature* cannon = bot->GetMap()->GetCreature(m_targetCannon);
        if (cannon && cannon->IsAlive() &&
            !cannon->HasUnitFlag(UNIT_FLAG_NOT_SELECTABLE) &&
            cannon->GetVehicleKit() &&
            cannon->GetVehicleKit()->GetAvailableSeatCount() > 0)
        {
            // HandleSpellClick is processed server-side with no strict range enforcement for WG tower cannons, so boarding
            // succeeds once the bot is anywhere near the cannon. Tower cannons (z ~439) are too high to reach via MMAPS,
            // and without manual waypoints, GetExactDist2d is needed. The result is that bots are able to pathfind and mount
            // normally on some lower level cannons, but on other cannons they jump up in a cartoonish way, but they do mount.
            if (bot->GetExactDist2d(cannon) > 20.0f)
                return MoveTo(cannon);

            cannon->HandleSpellClick(bot);
            if (bot->IsOnVehicle(cannon))
            {
                if (bot->IsMounted())
                {
                    WorldPacket emptyPacket;
                    bot->GetSession()->HandleCancelMountAuraOpcode(emptyPacket);
                }
                return true;
            }
            // Boarding failed. Drop target, wait for next scan.
            m_targetCannon.Clear();
            return false;
        }
        // Cannon is destroyed or taken. Return to normal waypoint movement until next scan.
        m_targetCannon.Clear();
        return false;
    }

    // Scan gate: only when the bot is close enough to the fortress central wall.
    if (bot->GetDistance(WG_OBJ_CENTRAL_WALL.GetPositionX(),
                         WG_OBJ_CENTRAL_WALL.GetPositionY(),
                         WG_OBJ_CENTRAL_WALL.GetPositionZ()) > WG_CANNON_CENTRAL_WALL_RANGE)
        return false;

    // Level-priority stagger: normalize bot levels into 3 groups (0–2). Higher level bots have higher initial delay to scan for available
    // cannons, and have less frequent scans. Why waste a level 80 on the cannon when a level 75 can operate it exactly the same way?
    uint32 minLvl   = sWorld->getIntConfig(CONFIG_WINTERGRASP_PLR_MIN_LVL);
    uint32 lvlRange = (DEFAULT_MAX_LEVEL > minLvl) ? (DEFAULT_MAX_LEVEL - minLvl) : 1u;
    uint32 aboveMin = (bot->GetLevel() > minLvl) ? uint32(bot->GetLevel() - minLvl) : 0u;
    uint32 stagger  = (aboveMin * 2u) / lvlRange;   // 0–2

    uint32 now = getMSTime();
    if (m_cannonScanTime == 0)
        m_cannonScanTime = now + (WG_SCAN_INTERVAL * stagger);

    if (now < m_cannonScanTime)
        return false;

    m_cannonScanTime = now + (WG_SCAN_INTERVAL * (1u + stagger));

    // Collect all NPC_WINTERGRASP_TOWER_CANNON creatures that are within WG_OBJ_SCAN_RANGE yards from the bot.
    // Cannons must be among those listed in WG_DEFENDER_CANNON_POSITIONS.
    std::list<Creature*> nearby;
    bot->GetCreatureListWithEntryInGrid(nearby, NPC_WINTERGRASP_TOWER_CANNON, WG_OBJ_SCAN_RANGE);

    std::vector<Creature*> candidates;
    for (Creature* c : nearby)
    {
        if (!c->IsAlive() || c->HasUnitFlag(UNIT_FLAG_NOT_SELECTABLE))
            continue;
        Vehicle* veh = c->GetVehicleKit();
        if (!veh || !veh->GetAvailableSeatCount())
            continue;

        for (uint8 i = 0; i < WG_DEFENDER_CANNON_COUNT; ++i)
        {
            if (c->GetExactDist(WG_DEFENDER_CANNON_POSITIONS[i].GetPositionX(),
                                WG_DEFENDER_CANNON_POSITIONS[i].GetPositionY(),
                                WG_DEFENDER_CANNON_POSITIONS[i].GetPositionZ())
                <= WG_CANNON_SEARCH_RADIUS)
            {
                candidates.push_back(c);
                break;
            }
        }
    }

    if (candidates.empty())
        return false;

    // Random selection, so bots spread across available cannons.
    Creature* chosen = candidates[urand(0, candidates.size() - 1)];
    m_targetCannon   = chosen->GetGUID();
    return MoveTo(chosen);
}

// Target acquisition for cannons and siege turrets
Unit* WgFireCannonAction::GetTarget()
{
    // Prioritize mounted hostile vehicle.
    Creature* nearest = WgFindNearestHostileVehicle(bot, CANNON_TARGET_SCAN);
    if (nearest)
        return nearest;

    // No mounted hostile vehicle in range. Fall back to normal hostile targets.
    return CastVehicleSpellAction::GetTarget();
}

// Target acquisition for demolishers' hurl boulder attack
Unit* WgHurlBoulderAction::GetTarget()
{
    // Prioritize mounted hostile vehicle.
    Creature* nearest = WgFindNearestHostileVehicle(bot, VEHICLE_TARGET_SCAN);
    if (nearest)
        return nearest;

    // No mounted hostile vehicle in range. Fall back to normal hostile targets.
    return CastVehicleSpellAction::GetTarget();
}
