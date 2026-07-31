/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

// ######################################################################################################################################### //
// To future developers: Wintergrasp is a non-linear PVP battle zone that deals with hundreds of bots with constantly changing objectives.
// Unlike the more linear Battlegrounds, a small change to BattlefieldTactics can have a radical effect on the balance of the match. Make
// sure you understand what you are changing, what the effects of your changes are, and to thoroughly test your changes.
// Darmok, few of the named terms have Star Trek references; don't drop them, it would make Kiazi's children cry.
// ######################################################################################################################################### //

#include "BattlefieldTactics.h"
#include "Battlefield.h"
#include "BattlefieldMgr.h"
#include "BattlefieldWG.h"
#include "BattlefieldPaths.h"  // Wintergrasp path network, objectives, and A* junction tables
#include "CellImpl.h"
#include "GossipDef.h"
#include "GridNotifiers.h"
#include "GridNotifiersImpl.h"
#include "PlayerbotAI.h"
#include "Playerbots.h"
#include "PositionValue.h"
#include "Vehicle.h"
#include "WorldPacket.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>

// Snap radius: match a cannon creature to a known g_WgCannonPaths entry.
static constexpr float  WG_CANNON_SEARCH_RADIUS        = 5.0f;
// Base interval (ms) between WgMountTowerCannonAction scans, multiplied by level stagger.
static constexpr uint32 WG_SCAN_INTERVAL               = 15000;
// Distance to Central Wall. Ensures defenders won't scan for cannons when they are far from the wall.
static constexpr float  WG_CANNON_CENTRAL_WALL_RANGE   = 150.0f;
// If a defender scans for a cannon, it will only scan within this range of itself.
static constexpr float  WG_OBJ_SCAN_RANGE              = 250.0f;
// Distance at which FollowWgRoute advances to the next A* waypoint (halved for vehicles).
static constexpr float  WG_NODE_SWITCH_DIST            = 20.0f;
// Bot must be within this distance of a noSkip waypoint before it can advance past it.
static constexpr float  WG_NOSKIP_ARRIVE_DIST          = 5.0f;
// FollowWgRoute returns false (arrived) when bot is within this distance of the objective.
static constexpr float  WG_OBJ_ARRIVE_DIST             = 10.0f;
// Workshop capture arrival latch triggers at this distance, enabling combat around the workshop.
static constexpr float  WG_WORKSHOP_ARRIVE_DIST        = 80.0f;
// Attacker vehicle phases 1/3/5 advance when bot is within this distance of the staging point.
static constexpr float  WG_STAGING_ARRIVE_DIST         = 10.0f;
// FindNearestGameObject range for locating a fortress wall GO to attack.
static constexpr float  WG_WALL_SCAN_DIST              = 80.0f;
// Distance at which attacker vehicles begin casting melee spells at a wall.
static constexpr float  WG_WALL_MELEE_DIST             = 10.0f;
// FindNearestGameObject range for locating a tower GO to attack.
static constexpr float  WG_TOWER_SCAN_DIST             = 80.0f;
// Distance at which defender vehicles begin casting melee spells at a tower.
// 35 yards melee range instead of 10 yards for attackers, because towers are not flat like walls.
static constexpr float  WG_TOWER_MELEE_DIST            = 35.0f;
// FindNearestCreature range for locating the workshop engineer NPC to summon a vehicle.
static constexpr float  ENGINEER_SCAN_RANGE            = 12.0f;
// Phase 4 defender vehicles keep at least this distance from each other.
static constexpr float  WG_DEF_VEH_DISPERSE_DIST       = 8.0f;
// Defender vehicles dispersion only kicks in within this minimum radius of the objective, or
// 2 * WG_DEF_VEH_DISPERSE_DIST (whichever is highest) to reduce movement oscillation.
static constexpr float  WG_DEF_VEH_MIN_GUARD_RADIUS    = 20.0f;

// ################# //
// A* Waypoint Graph
// ################# //

// Path blocking mechanics:
// Node block (WgNode::pathBlock): building stands AT the waypoint. Vehicles hold and attack it, but infantry pass.
// Edge block (WgEdge::blockWorldState): passage BETWEEN waypoints (junctions) is sealed for everyone until destroyed.
// Solid barriers use both, meanwhile blockers with infantry gaps use only the node block.
struct WgEdge { uint32 target; uint32 blockWorldState = 0; uint32 blockAura = 0; };

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

        // Paths A* never routes vehicles through:
        //      SE/SW and NE/NW fortress tower exits (10, 11, 31, 32).
        //      Fortress workshop paths (27, 28). Vehicles spawn here but leave only by a direct MoveTo to the teleporters, never A*.
        //      Defender cannon waypoints (33-44). Each sits on a wall where vehicles never path.
        static constexpr uint8 WG_NO_VEHICLE_PATHS[] = { 10, 11, 27, 28, 31, 32,
                                                         33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 };
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

        // Step 3: Cross-path junction edges. Bidirectional unless oneWay is set, in which case only the A->B edge
        // is added (pathA is the source).
        for (auto const& junc : WG_JUNCTIONS)
        {
            uint32 nodeA = pathOffset[junc.pathA] + junc.wpA;
            uint32 nodeB = pathOffset[junc.pathB] + junc.wpB;
            g_WgGraph[nodeA].adj.push_back({ nodeB, junc.blockWorldState, junc.blockAura });
            if (!junc.oneWay)
                g_WgGraph[nodeB].adj.push_back({ nodeA, junc.blockWorldState, junc.blockAura });
        }
    });
}

static constexpr float  CANNON_TARGET_SCAN          = 85.0f;    // Lock on targets somewhat beyond cannon's range (70).
static constexpr float  VEHICLE_TARGET_SCAN         = 65.0f;    // Don't look at targets beyond 70 yards. That will cause
                                                                    // fallback of hurl boulder to ram instead.

// For defender vehicles, priority one is defending the fortress, then sending vehicles to attack the towers if needed, finally the
// overflow is sent to actively hunt attacker vehicles. Each hunt target is tagged by the first defender hunter that finds it.
static constexpr uint8  VEHICLE_FORT_GUARD_MIN      = 3;        // Pre-breach: min defender vehicles guarding the gate or wall (each).
static constexpr uint8  VEHICLE_FORT_GUARD_MAX      = 4;        // Pre-breach: max guarding the gate or wall (each).
static constexpr uint8  VEHICLE_COURT_GUARD         = 8;        // Post-breach: max defender vehicles holding the court.

// Counter for defender vehicles assigned to...
static std::atomic<int32_t> s_WgFortGuardAtStage{0};            // Guard the fortress wall most likely to be attacked (faction dependent),
static std::atomic<int32_t> s_WgFortGuardAtGate{0};             // guard the fortress gate,
static std::atomic<int32_t> s_WgGuardAtCourt{0};                // guard a fortress court,
static std::atomic<int32_t> s_WgGuardAtWar{0};                  // or to hunt attacker vehicles.

static constexpr uint32 GUARD_REBALANCE_PERIOD      = 10000u;   // Period to re-evaluate fort guard positions.
static std::mutex           s_WgFortGuardPosMtx;                // Sequences fort guard slot claims and Stage/Gate/Court/War assignment.

static constexpr uint32 WG_WAR_SCAN_PERIOD          = 10000u;   // Period between hunt target scans while a hunter has no owned target.
static constexpr float  WG_WAR_NO_TAR_SCAN_RANGE    = 750.0f;   // Scan range for a hunter with no target at all.
static constexpr float  WG_WAR_ALT_TAR_SCAN_RANGE   = 250.0f;   // Scan range for a hunter chasing a target it did not tag,
                                                                    // as it seeks a closer untagged target.
static constexpr float  WG_WAR_STANDOFF_DIST        = 35.0f;    // Hunters hold this far from their target and let "wg hurl boulder" fire.
static std::mutex                             s_WgWarTargetMtx; // Sequences hunt target tag claims/releases.
static std::unordered_map<uint64_t, uint64_t> s_WgWarTargets;   // Tagged attacker vehicle GUID (raw) -> hunter bot GUID (raw).

// Battle-wide attacker vehicle snapshot shared by all hunters. Rebuilt at most once per WG_ATK_VEH_SCAN_PERIOD, by
// whichever hunter reads it first after that period; every hunter reads the same list and applies its own leash range
// (WG_WAR_*_SCAN_RANGE) to it. Stored positions are snapshot-time; a hunter chases the live target each tick, so drift
// between scans only affects which vehicle is picked, not the chase.
// Note that no battle end reset is needed, as each rebuild clears the list, and a new battle is realistically past the scan period.
struct WgAttackerVehicle
{
    ObjectGuid guid;
    float      x, y, z;                                         // Snapshot-time position, for leash range and nearest selection.
};
static constexpr uint32 WG_ATK_VEH_SCAN_PERIOD       = 5000u;   // Period between battle-wide attacker vehicle scans.
static std::mutex                     s_WgAtkVehScanMtx;        // Guards the snapshot list and its timestamp.
static uint32                         s_WgAtkVehScanTime = 0;   // getMSTime() of the last snapshot rebuild (0 = never).
static std::vector<WgAttackerVehicle> s_WgAttackerVehicles;     // Crewed hostile mobile vehicles found by the last scan.

static constexpr uint8  MAX_TOWER_SQUAD             = 3;        // Max defender vehicles assigned to attack any single tower.
static std::atomic<int32_t> s_WgTowerSquad[3]{};                // Per-tower squad counters, indexed by DEF_TOWERS[].
static std::mutex           s_WgTowerSquadMtx;                  // Sequences tower squad assignment to prevent races on tower reassignments.

// Summoning Vehicles: Don't send every eligible bot to get a vehicle. Send only an amount relative to how many
// available vehicles there are to summon.
static constexpr float  WS_GO_ATK_MULTIPLIER        = 1.0f;     // Multiplier for attacker bots going to summon a vehicle.
static constexpr float  WS_GO_DEF_MULTIPLIER        = 3.0f;     // Multiplier for defender bots going to summon a vehicle.
static std::atomic<int32_t> s_WgAtkGoingToWorkshop{0};
static std::atomic<int32_t> s_WgDefGoingToWorkshop{0};
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
// wg = nullptr skips wall checks. isVehicle = true skips infantry-only nodes. bot != nullptr applies per-bot
// aura blocks (edges with blockAura set). Returns empty if no path exists.
static std::vector<uint32> WgAStarPath(uint32 start, uint32 goal, BattlefieldWG* wg = nullptr, bool isVehicle = false, Player* bot = nullptr)
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
            // Walls appear in the graph in two ways: as junction edges (caught by the check above) and as a pathBlock on
            // the wall's own waypoint. A* only inspects edge blocks, not a node's pathBlock, so it would route a vehivle
            // straight through a standing wall node. FollowWgRoute then halts at that node, which can cause a vehicle to
            // get stuck within the wall. So treat a standing wall node as impassable to vehicles while it's only an
            // intermediate step. Allow it as the goal though, so attacker vehicles can reach a standing wall to attack it.

            if (isVehicle && wg && e.target != goal && g_WgGraph[e.target].pathBlock &&
                !WgIsBuildingDestroyed(wg, g_WgGraph[e.target].pathBlock))
                continue;
            if (e.blockAura && bot && bot->HasAura(e.blockAura))
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

// Returns true if the bot can reach a point through A* pathing, or if the nearest node to the bot is the nearest node to
// the point (meaning the bot is essentially at the point, which also makes WgAStarPath return an empty path).
static bool WgCanReach(Player* bot, Position const& goal, BattlefieldWG* wg)
{
    uint32 startNode = WgFindNearestNode(bot->GetPositionX(), bot->GetPositionY(), bot->GetPositionZ());
    uint32 goalNode  = WgFindNearestNode(goal.GetPositionX(), goal.GetPositionY(), goal.GetPositionZ());
    return !WgAStarPath(startNode, goalNode, wg).empty() || startNode == goalNode;
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

static uint32 const WG_FIXED_VEHICLE_ENTRIES[] = {
    NPC_WINTERGRASP_TOWER_CANNON,
    NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_ALLIANCE,
    NPC_WINTERGRASP_SIEGE_ENGINE_TURRET_HORDE,
};

static uint32 const WG_MOBILE_VEHICLE_ENTRIES[] = {
    NPC_WINTERGRASP_CATAPULT,
    NPC_WINTERGRASP_DEMOLISHER,
    NPC_WINTERGRASP_SIEGE_ENGINE_ALLIANCE,
    NPC_WINTERGRASP_SIEGE_ENGINE_HORDE,
};

// hisGuardAtWar tag map helpers. A hunter "tags" the attacker vehicle (WG_MOBILE_VEHICLE_ENTRIES only) it is
// going after so other hunters prefer untagged targets when any are within scanRange.
// Tags release on target death/despawn, hunter vehicle loss, or battle end.
static uint64_t WgVehicleTaggedBy(uint64_t vehicleRaw)
{
    std::lock_guard<std::mutex> lock(s_WgWarTargetMtx);
    auto it = s_WgWarTargets.find(vehicleRaw);
    return it != s_WgWarTargets.end() ? it->second : 0;
}
static bool WgTryTagVehicle(uint64_t vehicleRaw, uint64_t botRaw)
{
    std::lock_guard<std::mutex> lock(s_WgWarTargetMtx);
    return s_WgWarTargets.emplace(vehicleRaw, botRaw).second;
}
static void WgReleaseVehicleTag(uint64_t vehicleRaw, uint64_t botRaw)
{
    std::lock_guard<std::mutex> lock(s_WgWarTargetMtx);
    auto it = s_WgWarTargets.find(vehicleRaw);
    if (it != s_WgWarTargets.end() && it->second == botRaw)
        s_WgWarTargets.erase(it);
}
// Instead of having s_WgWarTargetMtx locked and unlocked as each individual candidate vehicle in the
// scan is checked to see whether it has a tag or not, the tagged attacker vehicle GUIDs are copied
// under a single lock. Then, an untagged-only scan can test each candidate against the copy, lock-free.
static std::unordered_set<uint64_t> WgSnapshotTaggedVehicles()
{
    std::lock_guard<std::mutex> lock(s_WgWarTargetMtx);
    std::unordered_set<uint64_t> tagged;
    tagged.reserve(s_WgWarTargets.size());
    for (auto const& kv : s_WgWarTargets)
        tagged.insert(kv.first);
    return tagged;
}

template <std::size_t N>
static bool WgIsEntryIn(uint32 const (&entries)[N], uint32 entry)
{
    for (uint32 e : entries)
        if (e == entry)
            return true;
    return false;
}

// Match filter parameters for WgFindNearestVehicle.
struct WgVehicleCheck
{
    Player*    bot;
    float      range;           // Handles proximity, scan range, or whatever range the bot vehicle is concerned with.
    bool       includeFixed;    // True for fixed vehicles (siege turrets are considered fixed).
    bool       wantHostile;     // True for hostile vehicles.
    ObjectGuid ignore;          // Skip this creature (the bot's own vehicle), if set.

    bool operator()(Creature* c) const
    {
        uint32 entry = c->GetEntry();
        if (!WgIsEntryIn(WG_MOBILE_VEHICLE_ENTRIES, entry) &&
            !(includeFixed && WgIsEntryIn(WG_FIXED_VEHICLE_ENTRIES, entry)))
            return false;
        if (!c->IsAlive() || bot->IsHostileTo(c) != wantHostile)
            return false;
        if (ignore && c->GetGUID() == ignore)
            return false;
        Vehicle* vKit = c->GetVehicleKit();
        if (!vKit || !vKit->GetPassenger(0))
            return false;
        return bot->IsWithinDist(c, range);
    }
};

// Returns the nearest vehicle matching check within range. The named finders below wrap this.
static Creature* WgFindNearestVehicle(Player* bot, float range, bool includeFixed, bool wantHostile,
                                      ObjectGuid ignore = ObjectGuid::Empty)
{
    std::list<Creature*> found;
    WgVehicleCheck check{bot, range, includeFixed, wantHostile, ignore};
    Acore::CreatureListSearcher<WgVehicleCheck> searcher(bot, found, check);
    Cell::VisitObjects(bot, searcher, range);

    Creature* nearest = nullptr;
    float nearestDist = FLT_MAX;
    for (Creature* c : found)
    {
        float d = bot->GetDistance(c);
        if (d < nearestDist)
        {
            nearestDist = d;
            nearest = c;
        }
    }
    return nearest;
}

// Find the nearest mounted hostile vehicle (fixed or mobile) within scanRange yards.
// Used by fixed cannons, siege turrets, and by the demolisher boulder attack.
static Creature* WgFindNearestHostileVehicle(Player* bot, float scanRange)
{
    return WgFindNearestVehicle(bot, scanRange, /*includeFixed*/ true, /*wantHostile*/ true);
}

// Find the nearest mounted friendly mobile vehicle within scanRange yards, ignoring the bot's own vehicle.
// Used by defender vehicles to disperse their guarding positions slightly away from each other.
static Creature* WgFindNearestFriendlyVehicle(Player* bot, float scanRange)
{
    ObjectGuid ownBase = bot->GetVehicle() && bot->GetVehicle()->GetBase()
        ? bot->GetVehicle()->GetBase()->GetGUID() : ObjectGuid::Empty;
    return WgFindNearestVehicle(bot, scanRange, /*includeFixed*/ false, /*wantHostile*/ false, ownBase);
}

// Rebuilds the shared attacker vehicle snapshot once it is older than WG_ATK_VEH_SCAN_PERIOD. Every siege vehicle is
// player-driven, and driving one requires being in the war, so every attacker vehicle is found by checking which
// players in the attacker team's PlayersInWar set are driving one.
static void WgRefreshAttackerVehiclesIfStale(Player* anchorBot, BattlefieldWG* wg)
{
    std::lock_guard<std::mutex> lock(s_WgAtkVehScanMtx);
    uint32 now = getMSTime();
    if (s_WgAtkVehScanTime != 0 && now - s_WgAtkVehScanTime < WG_ATK_VEH_SCAN_PERIOD)
        return;
    s_WgAtkVehScanTime = now;

    s_WgAttackerVehicles.clear();
    for (ObjectGuid const& guid : wg->GetPlayersInWarSet(wg->GetAttackerTeam()))
    {
        Player* p = ObjectAccessor::FindPlayer(guid);
        if (!p)
            continue;
        Unit* base = p->GetVehicleBase();
        Creature* c = base ? base->ToCreature() : nullptr;
        if (!c || !WgIsEntryIn(WG_MOBILE_VEHICLE_ENTRIES, c->GetEntry()))
            continue;
        // Record each vehicle once, through its seat-0 driver, and only while it is a live enemy.
        Vehicle* vKit = c->GetVehicleKit();
        if (!vKit || vKit->GetPassenger(0) != p)
            continue;
        if (!c->IsAlive() || !anchorBot->IsHostileTo(c))    // anchorBot is any defender. Used only as the hostility reference.
            continue;
        s_WgAttackerVehicles.push_back({ c->GetGUID(), c->GetPositionX(), c->GetPositionY(), c->GetPositionZ() });
    }
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
// vehicles guard the fortress in stationary positions, and others head to destroy attacker towers in squads of limited number of vehicles,
// per standing tower, in a sweep proximity order. The excess number of vehicles scan for and actively hunt attacker vehicles. If there's no
// more needed vehicles to attack the towers, they fall back to any needed stationary guard slots at the fortress, or go on the hunt.
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
    if (m_defGuardFortress == 1)
        --s_WgFortGuardAtStage;
    else if (m_defGuardFortress == 2)
        --s_WgFortGuardAtGate;
    else if (m_defGuardFortress == 3)
    {
        --s_WgGuardAtWar;
        if (m_warTarget)
            WgReleaseVehicleTag(m_warTarget.GetRawValue(), m_botGuidRaw);
    }
    else if (m_defGuardFortress == 4)
        --s_WgGuardAtCourt;
    m_defGuardFortress = 0;
    m_warTarget.Clear();
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

// A defender vehicle that found no tower to attack falls back to a fortress guard slot, or to hunt attacker vehicles
// (hisGuardAtWar) when the guard positions are full. Mutex locked so concurrent fallbacks can't overshoot a cap.
void WgCheckFlagAction::FallBackToGuardOrWar(bool whenTheWallsFell)
{
    std::lock_guard<std::mutex> lock(s_WgFortGuardPosMtx);
    if (!whenTheWallsFell)
    {
        // Pre-breach: top up the smaller of hisGuardAtWall/hisGuardAtGate toward VEHICLE_FORT_GUARD_MAX and overflow hunts.
        bool stageIsPreferred = (s_WgFortGuardAtStage <= s_WgFortGuardAtGate);
        std::atomic<int32_t>& countPreferred = stageIsPreferred ? s_WgFortGuardAtStage : s_WgFortGuardAtGate;
        std::atomic<int32_t>& countFallback  = stageIsPreferred ? s_WgFortGuardAtGate : s_WgFortGuardAtStage;
        uint8 slotPreferred = stageIsPreferred ? 1 : 2;
        uint8 slotFallback  = stageIsPreferred ? 2 : 1;
        if (countPreferred < VEHICLE_FORT_GUARD_MAX)
        {
            m_defGuardFortress = slotPreferred;
            ++countPreferred;
        }
        else if (countFallback < VEHICLE_FORT_GUARD_MAX)
        {
            m_defGuardFortress = slotFallback;
            ++countFallback;
        }
        else
        {
            m_defGuardFortress = 3;
            ++s_WgGuardAtWar;
        }
    }
    else
    {
        // Post-breach: hold the court up to VEHICLE_COURT_GUARD and overflow hunts.
        if (s_WgGuardAtCourt < VEHICLE_COURT_GUARD)
        {
            m_defGuardFortress = 4;
            ++s_WgGuardAtCourt;
        }
        else
        {
            m_defGuardFortress = 3;
            ++s_WgGuardAtWar;
        }
    }
    m_defGuardFortressTime = getMSTime();
}

// Picks the attacker mobile vehicle this hunter should chase next (updating m_warTarget and the tag map), or nullptr.
Creature* WgCheckFlagAction::AcquireWarTarget()
{
    // Validate the current target. Drop it if it's gone, dead, or uncrewed.
    Creature* target = m_warTarget ? bot->GetMap()->GetCreature(m_warTarget) : nullptr;
    if (target && (!target->IsAlive() || !target->GetVehicleKit() ||
                   !target->GetVehicleKit()->GetPassenger(0)))
        target = nullptr;
    if (!target && m_warTarget)
    {
        WgReleaseVehicleTag(m_warTarget.GetRawValue(), m_botGuidRaw);
        m_warTarget.Clear();
    }

    // If a hunter has a target, and it was the first to find it (ownTag) it should keep it; otherwise rescan, no more
    // than once per WG_WAR_SCAN_PERIOD.
    bool ownTag = target && WgVehicleTaggedBy(m_warTarget.GetRawValue()) == m_botGuidRaw;
    uint32 now = getMSTime();
    if ((target && ownTag) || now - m_warScanTime < WG_WAR_SCAN_PERIOD)
        return target;
    m_warScanTime = now;

    // Take local copies of the shared attacker vehicle snapshot and of the tag map, so the candidate for loop below runs
    // without holding either lock.
    if (BattlefieldWG* wg = GetBattlefieldWG())
        WgRefreshAttackerVehiclesIfStale(bot, wg);
    std::vector<WgAttackerVehicle> candidates;
    {
        std::lock_guard<std::mutex> lock(s_WgAtkVehScanMtx);
        candidates = s_WgAttackerVehicles;
    }
    std::unordered_set<uint64_t> tagged = WgSnapshotTaggedVehicles();

    // Range for target lookup in `candidates`, depending on whether the hunter has no target, or has one it doesn't own (!ownTag).
    float untaggedRange = target ? WG_WAR_ALT_TAR_SCAN_RANGE : WG_WAR_NO_TAR_SCAN_RANGE;
    ObjectGuid nearestUntagged; float nearestUntaggedDist = untaggedRange;
    ObjectGuid nearestAny;      float nearestAnyDist      = WG_WAR_NO_TAR_SCAN_RANGE;
    for (WgAttackerVehicle const& cand : candidates)
    {
        float dist = bot->GetDistance(cand.x, cand.y, cand.z);
        if (dist < nearestAnyDist)
        {
            nearestAnyDist = dist;
            nearestAny     = cand.guid;
        }
        if (dist < nearestUntaggedDist && !tagged.count(cand.guid.GetRawValue()))
        {
            nearestUntaggedDist = dist;
            nearestUntagged     = cand.guid;
        }
    }

    // Claim the nearest untagged vehicle and chase it; a successful tag means no other hunter owns it. A hunter that
    // claims nothing keeps the target it has, rather than swapping one unowned target for another that is only nearer.
    // Only with no claim and no current target does a hunter fall back to the nearest already-tagged vehicle.
    if (nearestUntagged && WgTryTagVehicle(nearestUntagged.GetRawValue(), m_botGuidRaw))
    {
        if (Creature* c = bot->GetMap()->GetCreature(nearestUntagged))
        {
            m_warTarget = nearestUntagged;
            return c;
        }
        WgReleaseVehicleTag(nearestUntagged.GetRawValue(), m_botGuidRaw);   // Vanished between the snapshot and the claim.
    }
    if (target)
        return target;
    if (nearestAny)
    {
        if (Creature* c = bot->GetMap()->GetCreature(nearestAny))
        {
            m_warTarget = nearestAny;
            return c;
        }
    }
    return nullptr;
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
        uint32 startNode = WgFindNearestNode(bot->GetPositionX(), bot->GetPositionY(), bot->GetPositionZ());
        m_route = WgAStarPath(startNode, goalNode, wg, checkPathBlock, bot);
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

    // Defender vehicles guarding the fortress hold either hisGuardAtWall (the side wall most likely to be attacked,
    // depending on faction) or hisGuardAtGate in front of Fortress Gate. Each holds up to VEHICLE_FORT_GUARD_MAX.
    Position const& hisGuardAtWall = (wg->GetAttackerTeam() == TEAM_ALLIANCE)
        ? WG_OBJ_DEF_GUARD_EAST : WG_OBJ_DEF_GUARD_WEST;
    Position const& hisGuardAtGate = WG_OBJ_DEF_GUARD_GATE;

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
        if (m_defGuardFortress == 1)
            --s_WgFortGuardAtStage;
        else if (m_defGuardFortress == 2)
            --s_WgFortGuardAtGate;
        else if (m_defGuardFortress == 3)
        {
            --s_WgGuardAtWar;
            if (m_warTarget)
                WgReleaseVehicleTag(m_warTarget.GetRawValue(), m_botGuidRaw);
        }
        else if (m_defGuardFortress == 4)
            --s_WgGuardAtCourt;

        m_defGuardFortress = 0;
        m_warTarget.Clear();
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

        // Pre-breach: if either hisGuardAtWall or hisGuardAtGate is below VEHICLE_FORT_GUARD_MIN, claim the smaller
        // one before doing anything else. Past MIN, go attacker towers. When no tower slot is available, fill the smaller
        // guard spot up to VEHICLE_FORT_GUARD_MAX. Past MAX on both, hunt attackers (hisGuardAtWar).
        // Post-breach: fill hisGuardAtCourt to VEHICLE_COURT_GUARD first, then towers if needed, then hunt attacker.
        // Higher priorities are replenished with new vehicles only, except from any fallback after destroying attacker towers.
        switch (m_defVehiclePhase)
        {
            case 0: // Phase 0: pick the nearest fortress vehicle teleporter to get the vehicle out of the fortress.
            {
                // Defenders always source vehicles from the fortress (for now), so they leave via a teleporter.
                // Phase 1 gates on the teleport aura, so later phases can't start until the vehicle is outside.
                float dE = bot->GetDistance(WG_OBJ_WS_TELE_EAST.GetPositionX(),
                                            WG_OBJ_WS_TELE_EAST.GetPositionY(),
                                            WG_OBJ_WS_TELE_EAST.GetPositionZ());
                float dW = bot->GetDistance(WG_OBJ_WS_TELE_WEST.GetPositionX(),
                                            WG_OBJ_WS_TELE_WEST.GetPositionY(),
                                            WG_OBJ_WS_TELE_WEST.GetPositionZ());
                m_workshopIdx     = (dE < dW) ? WG_WS_IDX_FORT_EAST : WG_WS_IDX_FORT_WEST;
                m_defVehiclePhase = 1;
                break;
            }
            case 1: // Phase 1: navigate to the fortress vehicle teleporter, and check which role to fulfill by priority.
            {
                Vehicle* veh = bot->GetVehicle();
                Unit* vBase = veh ? veh->GetBase() : nullptr;
                // Mirab, his sails unfurled: Vehicle has passed through the teleporter and launched outside.
                bool hisSailsUnfurled = vBase && vBase->HasAura(SPELL_VEHICLE_TELEPORT);
                if (hisSailsUnfurled)
                {
                    // Claim a guard slot if there's any, or attack the towers in phase 2.
                    bool becameGuard = false;
                    {
                        std::lock_guard<std::mutex> lock(s_WgFortGuardPosMtx);
                        if (!whenTheWallsFell)
                        {
                            bool stageIsSmaller = (s_WgFortGuardAtStage <= s_WgFortGuardAtGate);
                            std::atomic<int32_t>& countSmaller = stageIsSmaller ? s_WgFortGuardAtStage : s_WgFortGuardAtGate;
                            if (countSmaller < VEHICLE_FORT_GUARD_MIN)
                            {
                                m_defGuardFortress = stageIsSmaller ? 1 : 2;
                                ++countSmaller;
                                becameGuard = true;
                            }
                        }
                        else if (s_WgGuardAtCourt < VEHICLE_COURT_GUARD)
                        {
                            m_defGuardFortress = 4;
                            ++s_WgGuardAtCourt;
                            becameGuard = true;
                        }
                        if (becameGuard)
                            m_defGuardFortressTime = getMSTime();
                    }
                    m_defVehiclePhase = becameGuard ? 4 : 2;
                    break;
                }
                break;
            }
            case 2:     // Phase 2: take the first standing tower with a free squad slot (< MAX_TOWER_SQUAD), else fall back to phase 4.
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

                if (nextIdx == 0xFF)    // No free tower squad slot found, go to phase 4.
                {
                    FallBackToGuardOrWar(whenTheWallsFell);
                    m_defVehiclePhase = 4;
                }
                else                    // An empty tower squad slot found, go to phase 3.
                {
                    m_targetTowerIdx  = nextIdx;
                    m_isTowerAttacker = true;
                    m_defVehiclePhase = 3;
                }
                break;
            }
            case 3: // Phase 3: route to targeted tower and reassign squad on destruction.
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

                    if (nextIdx != 0xFF)    // Stay in phase 3, new target set above.
                    {
                        m_targetTowerIdx  = nextIdx;
                        m_isTowerAttacker = true;
                    }
                    else                    // No room in any surviving squad: fall back to phase 4.
                    {
                        m_targetTowerIdx = 0xFF;
                        FallBackToGuardOrWar(whenTheWallsFell);
                        m_defVehiclePhase = 4;
                    }
                    break;
                }
                break;
            }
            default:    // Phase 4: hold a guard slot, or hunt attacker vehicles.
            {
                AI_VALUE(PositionMap&, "position")["bg siege"].Reset();

                // Re-balance the guard roles (wall/gate) between themselves every GUARD_REBALANCE_PERIOD, and
                // immediately on breach, retire the wall/gate guard slots and start using the court guard slot.
                // Roles (m_defGuardFortress): 0=none 1=wall 2=gate 3=war(hunt) 4=court.
                if (m_defGuardFortress != 3)
                {
                    uint32 now = getMSTime();
                    bool breachObsoletesRole = whenTheWallsFell && (m_defGuardFortress == 1 || m_defGuardFortress == 2);
                    if (m_defGuardFortress == 0 || breachObsoletesRole || now - m_defGuardFortressTime >= GUARD_REBALANCE_PERIOD)
                    {
                        std::lock_guard<std::mutex> lock(s_WgFortGuardPosMtx);
                        // Release the current guard slot.
                        if (m_defGuardFortress == 1)
                            --s_WgFortGuardAtStage;
                        else if (m_defGuardFortress == 2)
                            --s_WgFortGuardAtGate;
                        else if (m_defGuardFortress == 4)
                            --s_WgGuardAtCourt;

                        // Pre-breach:
                        if (!whenTheWallsFell)
                        {
                            if (s_WgFortGuardAtStage <= s_WgFortGuardAtGate)
                            {
                                m_defGuardFortress = 1;
                                ++s_WgFortGuardAtStage;
                            }
                            else
                            {
                                m_defGuardFortress = 2;
                                ++s_WgFortGuardAtGate;
                            }
                        }
                        // Post-breach:
                        else if (s_WgGuardAtCourt < VEHICLE_COURT_GUARD)
                        {
                            m_defGuardFortress = 4;
                            ++s_WgGuardAtCourt;
                        }
                        // Pre/Post-breach: send vehicles to hunt when stationary guard slots are full.
                        else
                        {
                            m_defGuardFortress = 3;
                            ++s_WgGuardAtWar;
                        }
                        m_defGuardFortressTime = now;
                    }
                }

                // hisGuardAtWar: chase the hunted attacker vehicle to WG_WAR_STANDOFF_DIST and hold, letting
                // "wg hurl boulder" do the attacking. Return true to claim the tick, preventing default class combat movement.
                if (m_defGuardFortress == 3)
                {
                    if (Creature* target = AcquireWarTarget())
                    {
                        if (bot->GetDistance(target) > WG_WAR_STANDOFF_DIST)
                        {
                            Position tpos(target->GetPositionX(), target->GetPositionY(), target->GetPositionZ(), 0.0f);
                            FollowWgRoute(tpos, true);
                            return true;
                        }
                        if (Unit* vBase = vehicle->GetBase())
                        {
                            if (vBase->isMoving())
                            {
                                vBase->StopMoving();
                                vBase->GetMotionMaster()->Clear();
                            }
                        }
                        return true;
                    }
                    // No target in range: fall through to a stationary guard point.
                }

                // Hold destination for the stationary roles (hisGuardAtWall/Gate/Court) and for hunters with no target in range,
                // who wait at the stationary guard point while they keep scanning for a target.
                Position const* defDest;
                if (m_defGuardFortress == 1)
                    defDest = &hisGuardAtWall;
                else if (m_defGuardFortress == 2)
                    defDest = &hisGuardAtGate;
                else if (m_defGuardFortress == 3 && !whenTheWallsFell)  // Waiting at the gate for a target in range.
                    defDest = &hisGuardAtGate;
                else    // Post-breach, everyone goes to the court, including hunters that don't have a target yet.
                    defDest = WgIsBuildingDestroyed(wg, WG_WS_CENTRAL_WALL) ? &WG_OBJ_CENTRAL_COURT
                                                                           : &WG_OBJ_FRONT_COURT;

                // Dispersion only applies once the vehicle has reached its guard area. The guard radius is at least twice the
                // disperse distance, so a push (up to one disperse distance) won't bounce a vehicle out of the area and oscillate.
                float guardRadius = std::max(WG_DEF_VEH_MIN_GUARD_RADIUS, 2.0f * WG_DEF_VEH_DISPERSE_DIST);
                float distToDest = bot->GetDistance(defDest->GetPositionX(), defDest->GetPositionY(), defDest->GetPositionZ());
                if (distToDest >= guardRadius)
                    return FollowWgRoute(*defDest, true);

                // In the guard area: if another defender vehicle is closer than WG_DEF_VEH_DISPERSE_DIST, push directly away
                // from it to restore spacing so one attacker cannon shot can't damage a cluster of defender vehicles.
                if (Creature* neighbour = WgFindNearestFriendlyVehicle(bot, WG_DEF_VEH_DISPERSE_DIST))
                {
                    // (dx, dy) points from the neighbour to this vehicle (the direction to move away). len is the distance
                    // between them, used below to place the vehicle WG_DEF_VEH_DISPERSE_DIST away along that direction.
                    float dx = bot->GetPositionX() - neighbour->GetPositionX();
                    float dy = bot->GetPositionY() - neighbour->GetPositionY();
                    float len = std::sqrt(dx * dx + dy * dy);
                    if (len < 0.1f)
                    {
                        // Vehicles overlap: no "away" direction. Fall back to a fixed direction split by GUID parity so the two
                        // push opposite ways instead of in lockstep.
                        dx = (bot->GetGUID().GetCounter() & 1) ? 1.0f : -1.0f;
                        dy = 0.0f;
                        len = 1.0f;
                    }
                    float moveX = neighbour->GetPositionX() + WG_DEF_VEH_DISPERSE_DIST * dx / len;
                    float moveY = neighbour->GetPositionY() + WG_DEF_VEH_DISPERSE_DIST * dy / len;
                    AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
                    return MoveTo(bot->GetMapId(), moveX, moveY, bot->GetPositionZ());
                }

                // In the guard area and well-spaced: actively hold position. Stop any residual movement immediately.
                if (Unit* vBase = vehicle->GetBase())
                {
                    if (vBase->isMoving())
                    {
                        vBase->StopMoving();
                        vBase->GetMotionMaster()->Clear();
                    }
                }

                // Returning false would yield the tick to the bot's default class combat movement ("reach melee"/"reach spell") which drives
                // the vehicle toward an arbitrary enemy until FollowWgRoute drags it back next tick, constantly drifting around the objective.
                // Claiming the tick (return true) prevents that.
                // However, "wg hurl boulder" (relevance 39) outranks "wg check flag" (relevance 31) and fires when a target is in range.
                return true;
            }
        }

        // Determine destination for current phase.
        Position const* dest = nullptr;
        switch (m_defVehiclePhase)
        {
            case 1:
                dest = (m_workshopIdx == WG_WS_IDX_FORT_EAST) ? &WG_OBJ_WS_TELE_EAST : &WG_OBJ_WS_TELE_WEST;
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

        // Phase 1: direct MoveTo to the nearest teleporter found in phase 0.
        if (m_defVehiclePhase == 1)
        {
            AI_VALUE(LastMovement&, "last movement").lastdelayTime = 0;
            return MoveTo(bot->GetMapId(),
                dest->GetPositionX() + frand(-0.1f, 0.1f),
                dest->GetPositionY() + frand(-0.1f, 0.1f),
                dest->GetPositionZ());
        }

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

            // Alliance uses Fortress WS West and Horde uses Fortress WS East.
            uint8 wsIdx = (team == TEAM_ALLIANCE) ? WG_WS_IDX_FORT_WEST : WG_WS_IDX_FORT_EAST;
            WgPath const& wsPath = *WG_WORKSHOPS[wsIdx].path;
            Position const engineerPos(wsPath[0].x, wsPath[0].y, wsPath[0].z, 0.0f);

            // Defenders are made to get vehicles only from inside the fortress, thus they can only A* navigate to the fortress
            // workshop engineer if the fortress has been breached, or if they are inside the fortress already.
            // whenTheWallsFell is used instead of WgCanReach alone, because whenTheWallsFell is a cheaper check.
            bool canEnterFortress = whenTheWallsFell || WgCanReach(bot, engineerPos, wg);

            if (canEnterFortress)
            {
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
                        bool capOk = !whenTheWallsFell    ||
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
            // Workshop priority for attackers: First friendly wins.
            static constexpr uint8 ATK_WS_PRIORITY_A[] = { WG_WS_IDX_NE, WG_WS_IDX_SE,
                                                            WG_WS_IDX_NW, WG_WS_IDX_SW };    // Alliance
            static constexpr uint8 ATK_WS_PRIORITY_H[] = { WG_WS_IDX_NW, WG_WS_IDX_SW,
                                                            WG_WS_IDX_NE, WG_WS_IDX_SE };    // Horde
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
        WgPath const& wsEast = *WG_WORKSHOPS[WG_WS_IDX_FORT_EAST].path;
        WgPath const& wsWest = *WG_WORKSHOPS[WG_WS_IDX_FORT_WEST].path;
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
    if (!wg || !wg->IsWarTime() || bot->isDead())
    {
        ResetCannonState();
        return false;
    }

    // Fortress cannons are for defenders only.
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
            // The bot's own WgCheckFlagAction, borrowed for its A* route follower used below.
            WgCheckFlagAction* checkFlag = static_cast<WgCheckFlagAction*>(botAI->GetAiObjectContext()->GetAction("wg check flag"));

            // HandleSpellClick is processed server-side with no strict range enforcement for WG tower cannons, so boarding
            // succeeds once the bot is anywhere near the cannon. Some cannons are too high to reach via MMAPS, so GetExactDist2d
            // is needed. The result is that bots on some cannons may jump up in a cartoonish way, but they do mount anyway.
            if (bot->GetExactDist2d(cannon) > 20.0f)
            {
                if (checkFlag)
                    return checkFlag->FollowWgRoute(*cannon, false);
                return MoveTo(cannon);
            }

            cannon->HandleSpellClick(bot);
            if (bot->IsOnVehicle(cannon))
            {
                if (bot->IsMounted())
                {
                    WorldPacket emptyPacket;
                    bot->GetSession()->HandleCancelMountAuraOpcode(emptyPacket);
                }
                if (checkFlag)
                    checkFlag->ResetBattleState();  // Release the bot's other duties now that it's piloting a cannon.
                return true;
            }
            // HandleSpellClick may not seat the bot the same tick. Hold and retry next tick rather than abandoning the
            // cannon, which would let wg check flag pull the bot away and force a fresh scan.
            return true;
        }
        // Cannon is destroyed or taken. Return to normal waypoint movement until next scan.
        m_targetCannon.Clear();
        return false;
    }

    // Scan gate: only when the bot is within WG_CANNON_CENTRAL_WALL_RANGE of the fortress central wall.
    if (bot->GetDistance(WG_OBJ_CENTRAL_WALL.GetPositionX(),
                         WG_OBJ_CENTRAL_WALL.GetPositionY(),
                         WG_OBJ_CENTRAL_WALL.GetPositionZ()) > WG_CANNON_CENTRAL_WALL_RANGE)
        return false;

    // Level-priority stagger: normalize bot levels into 3 groups (0-2). Higher level bots have higher initial delay to scan for available
    // cannons, and have less frequent scans. Why waste a level 80 on the cannon when a level 75 can operate it exactly the same way?
    uint32 minLvl   = sWorld->getIntConfig(CONFIG_WINTERGRASP_PLR_MIN_LVL);
    uint32 lvlRange = (DEFAULT_MAX_LEVEL > minLvl) ? (DEFAULT_MAX_LEVEL - minLvl) : 1u;
    uint32 aboveMin = (bot->GetLevel() > minLvl) ? uint32(bot->GetLevel() - minLvl) : 0u;
    uint32 stagger  = (aboveMin * 2u) / lvlRange;   // 0-2

    uint32 now = getMSTime();
    if (m_cannonScanTime == 0)
        m_cannonScanTime = now + (WG_SCAN_INTERVAL * stagger);

    if (now < m_cannonScanTime)
        return false;

    m_cannonScanTime = now + (WG_SCAN_INTERVAL * (1u + stagger));

    // If the bot can't A* navigate to the Central Wall, it can't A* navigate to any fortress cannon.
    if (!WgCanReach(bot, WG_OBJ_CENTRAL_WALL, wg))
        return false;

    // Collect all NPC_WINTERGRASP_TOWER_CANNON creatures within WG_OBJ_SCAN_RANGE of the bot, keeping those that match
    // a known cannon position in g_WgCannonPaths (within WG_CANNON_SEARCH_RADIUS).
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
            WgWaypoint const& cp = g_WgCannonPaths[i][0];
            if (c->GetExactDist(cp.x, cp.y, cp.z) <= WG_CANNON_SEARCH_RADIUS)
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
    // Start the A* approach toward the cannon node and the navigation block above will continue it next tick.
    WgCheckFlagAction* checkFlag = static_cast<WgCheckFlagAction*>(botAI->GetAiObjectContext()->GetAction("wg check flag"));
    if (checkFlag)
        return checkFlag->FollowWgRoute(*chosen, false);

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
