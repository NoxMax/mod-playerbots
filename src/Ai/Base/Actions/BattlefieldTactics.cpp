/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

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

// NPC entries for vehicle engineers at Wintergrasp workshops (from zone_wintergrasp.cpp)
static constexpr uint32 NPC_WG_GOBLIN_MECHANIC  = 30400;  // Horde workshop engineer
static constexpr uint32 NPC_WG_GNOMISH_ENGINEER = 30499;  // Alliance workshop engineer

// Workshop Coordinates
// Indexing order must match WintergraspWorkshopIds in BattlefieldWG.h. Only the 4 capturable workshops
// (indices 0-3) are used; the uncapturable two (KEEP_WEST, KEEP_EAST) are inside the fortress.
// Coordinates place bots at interaction distance with the workshop engineer NPC.
// DISABLED: Navigate to the workshops using the inverted vehicle paths.
// static constexpr uint8 WG_WORKSHOP_COUNT = 4;
// static Position const WG_POS_WORKSHOP[WG_WORKSHOP_COUNT] = {
//     { 4943.000f, 2388.400f, 324.044f, 0.0f },    // NE - Sunken Ring
//     { 4961.000f, 3384.500f, 380.800f, 0.0f },    // NW - Broken Temple
//     { 4357.700f, 2353.700f, 379.896f, 0.0f },    // SE - Eastspark
//     { 4353.200f, 3308.600f, 375.937f, 0.0f },    // SW - Westspark
// };

// The fortress has 24 cannons total, but for now only the 8 that are in range of the likely attacker paths
// are used. Bots search within WG_CANNON_SEARCH_RADIUS yards of a particular coordinate to locate the cannon.
static Position const WG_DEFENDER_CANNON_POSITIONS[] = {
    { 5235.0f, 2949.0f, 421.0f, 0.0f },
    { 5164.0f, 2961.0f, 440.0f, 0.0f },
    // { 5137.0f, 2935.0f, 440.0f, 0.0f },      // Front facing tower cannon. Not quite in range.
    { 5148.0f, 2862.0f, 422.0f, 0.0f },
    { 5149.0f, 2821.0f, 422.0f, 0.0f },
    // { 5138.0f, 2748.0f, 440.0f, 0.0f },      // Front facing tower cannon. Not quite in range.
    { 5164.0f, 2722.0f, 440.0f, 0.0f },
    { 5236.0f, 2733.0f, 422.0f, 0.0f },
    { 5264.0f, 2861.0f, 422.0f, 0.0f },
    { 5265.0f, 2820.0f, 422.0f, 0.0f },
};
static constexpr uint8  WG_DEFENDER_CANNON_COUNT    = 8;
static constexpr float  WG_CANNON_SEARCH_RADIUS     = 5.0f;
static constexpr uint32 WG_SCAN_INTERVAL            = 15000;   // ms between scans for cannons (at minimum level)

// Bots will only scan for free cannons when they are within WG_CANNON_INNER_WALL_RANGE yards from the central
// inner wall (with passage), scanning WG_CANNON_SCAN_RANGE yards from their position.
static Position const WG_INNER_WALL_POS             = { 5279.000f, 2841.200f, 409.800f, 0.0f };
static constexpr float WG_CANNON_INNER_WALL_RANGE   = 100.0f;
static constexpr float WG_CANNON_SCAN_RANGE         = 200.0f;

// Vehicle paths from workshops through the fortress to the vault. The fortress walls are actually transparent
// to bots and their vehicles, so vehicle paths must be manually blocked at certrain waypoints until they
// destroy what's in front of them.
//   0      = no obstacle, proceed freely.
//   190375 = Fortress Gate  (BATTLEFIELD_WG_OBJECTTYPE_DOOR)
//   191805 = Inner Wall     (BATTLEFIELD_WG_OBJECTTYPE_WALL "with passage")
//   191810 = Vault Door     (BATTLEFIELD_WG_OBJECTTYPE_DOOR_LAST)
struct WgVehicleWp { Position pos; uint32 pathBlock; };

// Broken Temple:
static WgVehicleWp const WG_NW_VEHICLE_WAYPOINTS[] = {

    { { 4961.000f, 3384.500f, 380.800f, 0.0f }, 0      },   // Workshop Engineer
    { { 4946.680f, 3361.380f, 376.876f, 0.0f }, 0      },
    { { 4936.470f, 3333.580f, 376.882f, 0.0f }, 0      },
    { { 4967.200f, 3324.780f, 376.876f, 0.0f }, 0      },
    { { 4998.398f, 3309.240f, 376.605f, 0.0f }, 0      },
    { { 5026.900f, 3266.570f, 365.136f, 0.0f }, 0      },
    { { 5053.850f, 3213.200f, 356.928f, 0.0f }, 0      },
    { { 5056.060f, 3155.580f, 357.429f, 0.0f }, 0      },
    { { 5049.580f, 3091.680f, 365.667f, 0.0f }, 0      },
    { { 5044.520f, 3059.250f, 366.603f, 0.0f }, 0      },
    { { 5050.820f, 3025.020f, 367.488f, 0.0f }, 0      },
    { { 5042.980f, 2969.850f, 373.628f, 0.0f }, 0      },
    { { 5045.260f, 2917.360f, 385.360f, 0.0f }, 0      },
    { { 5047.570f, 2870.640f, 392.068f, 0.0f }, 0      },
    { { 5051.630f, 2847.530f, 393.187f, 0.0f }, 0      },   // Road Axis Point
    { { 5107.080f, 2843.570f, 402.537f, 0.0f }, 0      },
    { { 5158.000f, 2841.200f, 408.799f, 0.0f }, 190375 },   // Fortress Gate
    { { 5200.000f, 2841.200f, 409.000f, 0.0f }, 0      },
    { { 5238.000f, 2841.200f, 409.192f, 0.0f }, 0      },
    { { 5272.000f, 2841.200f, 409.192f, 0.0f }, 191805 },   // Inner Wall
    { { 5343.000f, 2841.200f, 409.200f, 0.0f }, 0      },
    { { 5369.000f, 2841.200f, 409.200f, 0.0f }, 0      },
    { { 5393.500f, 2841.200f, 418.676f, 0.0f }, 191810 },   // Vault Door
};
static constexpr uint8 WG_NW_VEHICLE_WAYPOINT_COUNT = 23;

// Sunken Ring:
static WgVehicleWp const WG_NE_VEHICLE_WAYPOINTS[] = {

    { { 4943.000f, 2388.400f, 324.045f, 0.0f }, 0      },   // Workshop Engineer
    { { 4954.210f, 2414.020f, 320.177f, 0.0f }, 0      },
    { { 4964.410f, 2456.260f, 322.519f, 0.0f }, 0      },
    { { 5001.920f, 2509.680f, 335.717f, 0.0f }, 0      },
    { { 5018.100f, 2560.520f, 350.434f, 0.0f }, 0      },
    { { 5019.450f, 2595.080f, 355.574f, 0.0f }, 0      },
    { { 5017.437f, 2633.034f, 357.711f, 0.0f }, 0      },
    { { 5010.923f, 2665.807f, 362.207f, 0.0f }, 0      },
    { { 5007.541f, 2701.449f, 369.659f, 0.0f }, 0      },
    { { 5023.667f, 2739.467f, 375.070f, 0.0f }, 0      },
    { { 5049.488f, 2778.485f, 382.408f, 0.0f }, 0      },
    { { 5046.450f, 2816.200f, 389.986f, 0.0f }, 0      },
    { { 5051.630f, 2847.530f, 393.187f, 0.0f }, 0      },   // Road Axis Point
    { { 5107.080f, 2843.570f, 402.537f, 0.0f }, 0      },
    { { 5158.000f, 2841.200f, 408.799f, 0.0f }, 190375 },   // Fortress Gate
    { { 5200.000f, 2841.200f, 409.000f, 0.0f }, 0      },
    { { 5238.000f, 2841.200f, 409.192f, 0.0f }, 0      },
    { { 5272.000f, 2841.200f, 409.192f, 0.0f }, 191805 },   // Inner Wall
    { { 5343.000f, 2841.200f, 409.200f, 0.0f }, 0      },
    { { 5369.000f, 2841.200f, 409.200f, 0.0f }, 0      },
    { { 5393.500f, 2841.200f, 418.676f, 0.0f }, 191810 },   // Vault Door
};
static constexpr uint8 WG_NE_VEHICLE_WAYPOINT_COUNT = 21;

// Horde attacker path, from the west spawn area and into the fortress.
static Position const WG_HORDE_ATTACKER_WAYPOINTS[] = {
    { 5005.370f, 3643.630f, 360.602f, 0.0f },
    { 5026.310f, 3583.950f, 356.314f, 0.0f },
    { 4997.940f, 3516.910f, 356.521f, 0.0f },
    { 5027.680f, 3426.270f, 361.439f, 0.0f },
    { 4999.040f, 3363.250f, 376.868f, 0.0f },
    { 4998.398f, 3309.240f, 376.605f, 0.0f },
    { 5026.900f, 3266.570f, 365.136f, 0.0f },
    { 5053.850f, 3213.200f, 356.928f, 0.0f },
    { 5121.620f, 3113.440f, 367.783f, 0.0f },
    { 5185.220f, 3012.660f, 399.303f, 0.0f },
    { 5187.390f, 2946.220f, 413.494f, 0.0f },
    { 5186.410f, 2919.200f, 413.494f, 0.0f },
    { 5238.000f, 2841.200f, 409.192f, 0.0f },       // Front Court
    { 5272.000f, 2841.200f, 409.192f, 0.0f },
    { 5343.000f, 2841.200f, 409.200f, 0.0f },       // Central Court

};
static constexpr uint8 WG_HORDE_ATTACKER_WAYPOINT_COUNT = 15;

// Alliance attacker path, from the east spawn area and into the fortress.
static Position const WG_ALLIANCE_ATTACKER_WAYPOINTS[] = {
    { 5072.320f, 2197.720f, 358.310f, 0.0f },
    { 5063.550f, 2234.020f, 356.534f, 0.0f },
    { 5068.660f, 2291.000f, 356.535f, 0.0f },
    { 5058.310f, 2331.560f, 359.560f, 0.0f },
    { 5023.930f, 2377.530f, 359.357f, 0.0f },
    { 5024.554f, 2457.148f, 359.357f, 0.0f },
    { 5059.315f, 2525.162f, 358.852f, 0.0f },
    { 5090.200f, 2573.880f, 366.270f, 0.0f },
    { 5136.660f, 2609.270f, 378.507f, 0.0f },
    { 5175.440f, 2694.140f, 404.381f, 0.0f },
    { 5186.720f, 2733.060f, 413.492f, 0.0f },
    { 5187.950f, 2761.220f, 413.492f, 0.0f },
    { 5238.000f, 2841.200f, 409.192f, 0.0f },
    { 5238.000f, 2841.200f, 409.192f, 0.0f },       // Front Court
    { 5272.000f, 2841.200f, 409.192f, 0.0f },
    { 5343.000f, 2841.200f, 409.200f, 0.0f },       // Central Court

};
static constexpr uint8 WG_ALLIANCE_ATTACKER_WAYPOINTS_COUNT = 16;

// Defender waypoint, from the graveyard behind the fortress to the fortress front court. Same for both factions.
static Position const WG_DEFENDER_WAYPOINTS[] = {
    { 5538.185f, 2890.228f, 517.055f, 0.0f },
    { 5542.540f, 2835.510f, 515.382f, 0.0f },
    { 5547.310f, 2778.200f, 517.650f, 0.0f },
    { 5546.610f, 2741.440f, 506.827f, 0.0f },
    { 5521.930f, 2721.900f, 488.751f, 0.0f },
    { 5499.600f, 2737.100f, 471.601f, 0.0f },
    { 5475.600f, 2708.640f, 451.204f, 0.0f },
    { 5430.040f, 2731.270f, 423.359f, 0.0f },
    { 5400.930f, 2757.500f, 409.239f, 0.0f },
    { 5343.000f, 2762.000f, 409.191f, 0.0f },
    { 5343.000f, 2841.200f, 409.200f, 0.0f },       // Central Court
    { 5272.000f, 2841.200f, 409.192f, 0.0f },
    { 5238.000f, 2841.200f, 409.192f, 0.0f },       // Front Court
};
static constexpr uint8 WG_DEFENDER_WAYPOINT_COUNT = 13;

// Don't send every attacker bot to the workshop just because they have rank First Lieutenant. Send no more than
// WORKSHOP_GO_MULTIPLIER * availSlots (how many available vehicles can be acquired).
static constexpr uint32 WORKSHOP_GO_MULTIPLIER  = 5;
static int32            s_WgBotsGoingToWorkshop = 0;    // Counts how many attacker bots are going to a workshop.

static BattlefieldWG* GetBattlefieldWG()
{
    return dynamic_cast<BattlefieldWG*>(sBattlefieldMgr->GetBattlefieldByBattleId(BATTLEFIELD_BATTLEID_WG));
}

WgCheckFlagAction::~WgCheckFlagAction()
{
    if (m_goingToWorkshop)
        --s_WgBotsGoingToWorkshop;
}

bool WgCheckFlagAction::Execute(Event /*event*/)
{
    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg || !wg->IsWarTime())
        return false;

    // On death, reset all pathing data, in case the bot tries to navigate to a stale pre-death waypoint.
    if (bot->isDead())
    {
        m_attackerWpIdx  = 0;
        m_defenderWpIdx  = 0;
        m_vehicleWpIdx   = 0;
        m_workshopWpIdx  = 0xFF;
        m_workshopPathId = 0xFF;
        if (m_goingToWorkshop)
        {
            m_goingToWorkshop = false;
            --s_WgBotsGoingToWorkshop;
        }
        return false;
    }

    bool isDriver   = botAI->IsInVehicle(true);
    bool inVehicle  = bot->GetVehicle() != nullptr;
    bool isAttacker = (bot->GetTeamId() != wg->GetDefenderTeam());

    // Passengers let the driver navigate; nothing to do.
    if (inVehicle && !isDriver)
        return false;

    // Attacker vehicle driver: navigate from the workshop to the fortress gate.
    // When a path block is detected within VEHICLE_RANGE_DIST, begin firing.
    // Spell filtering is done by the type of target they can attack. WG vehicle ranged attacks (e.g. Hurl Boulder)
    // will have the bit mask TARGET_FLAG_DEST_LOCATION, whereas the melee attacks (e.g. Ram) will have
    // SPELL_EFFECT_GAMEOBJECT_DAMAGE.
    // Fire ranged attacks when within VEHICLE_RANGE_DIST, and both attacks when within VEHICLE_MELEE_DIST.
    // Advance only when the path blocker is destroyed.
    // TODO: Horde attackers use Broken Temple. Alliance attackers use Sunken Ring. Need more complex workshop capturing.
    if (isDriver && isAttacker)
    {
        // On vehicle entry, clear data for pathing to the workshop and release the bot's slot in s_WgBotsGoingToWorkshop.
        if (m_workshopWpIdx != 0xFF || m_workshopPathId != 0xFF || m_goingToWorkshop)
        {
            m_workshopWpIdx  = 0xFF;
            m_workshopPathId = 0xFF;
            if (m_goingToWorkshop)
            {
                m_goingToWorkshop = false;
                --s_WgBotsGoingToWorkshop;
            }
        }

        // Vehicle ranged and melee attacks are actually further than 30/3, but path blocking has issues with higher values.
        static constexpr float VEHICLE_RANGE_DIST = 30.0f;
        static constexpr float VEHICLE_MELEE_DIST = 3.0f;

        bool isHorde = (bot->GetTeamId() == TEAM_HORDE);
        WgVehicleWp const* vehicleWps   = isHorde ? WG_NW_VEHICLE_WAYPOINTS     : WG_NE_VEHICLE_WAYPOINTS;
        uint8              vehicleWpMax = isHorde ? WG_NW_VEHICLE_WAYPOINT_COUNT : WG_NE_VEHICLE_WAYPOINT_COUNT;

        Position const& wp = vehicleWps[m_vehicleWpIdx].pos;
        float d = bot->GetDistance(wp.GetPositionX(), wp.GetPositionY(), wp.GetPositionZ());
        if (d <= VEHICLE_RANGE_DIST)
        {
            uint32 blockEntry = vehicleWps[m_vehicleWpIdx].pathBlock;
            if (blockEntry != 0)
            {
                GameObject* go = bot->FindNearestGameObject(blockEntry, 50.0f);
                if (go && go->GetDestructibleState() != GO_DESTRUCTIBLE_DESTROYED)
                {
                    Vehicle* vehicle = bot->GetVehicle();
                    if (!vehicle)
                        return false;
                    Unit* vehicleBase = vehicle->GetBase();
                    Creature* creature = vehicleBase ? vehicleBase->ToCreature() : nullptr;
                    if (!creature)
                        return false;

                    PositionMap& posMap = AI_VALUE(PositionMap&, "position");
                    posMap["bg siege"].Set(go->GetPositionX(), go->GetPositionY(),
                                          go->GetPositionZ(), bot->GetMapId());

                    bool atWaypoint = (d <= VEHICLE_MELEE_DIST);
                    bool spellFired = false;
                    for (uint32 x = 0; x < MAX_CREATURE_SPELLS; ++x)
                    {
                        uint32 spellId = creature->m_spells[x];
                        if (!spellId || spellId == 2)
                            continue;
                        SpellInfo const* info = sSpellMgr->GetSpellInfo(spellId);
                        if (!info || info->IsPassive())
                            continue;

                        if (info->Targets & TARGET_FLAG_DEST_LOCATION)
                        {
                            // Ranged projectile (Hurl Boulder): fire at siege position.
                            spellFired |= botAI->CastVehicleSpell(spellId, vehicleBase);
                        }
                        else if (atWaypoint)
                        {
                            // Melee building attack (Ram): only when at the waypoint.
                            bool hasGoDamage = false;
                            for (uint8 eff = 0; eff < MAX_SPELL_EFFECTS; ++eff)
                                if (info->Effects[eff].Effect == SPELL_EFFECT_GAMEOBJECT_DAMAGE)
                                    { hasGoDamage = true; break; }
                            if (hasGoDamage)
                            {
                                vehicleBase->CastSpell(go->GetPositionX(), go->GetPositionY(),
                                                       go->GetPositionZ(), spellId, false);
                                spellFired = true;
                            }
                        }
                    }
                    // If still closing in, keep driving to reach Ram range.
                    if (!atWaypoint)
                        return MoveTo(bot->GetMapId(), wp.GetPositionX(), wp.GetPositionY(), wp.GetPositionZ());
                    return spellFired;
                }
            }
            // Path blocker destroyed. Clear siege position and advance.
            AI_VALUE(PositionMap&, "position")["bg siege"].Reset();
            if (m_vehicleWpIdx < vehicleWpMax - 1)
                ++m_vehicleWpIdx;
        }
        Position const& dest = vehicleWps[m_vehicleWpIdx].pos;
        return MoveTo(bot->GetMapId(), dest.GetPositionX(), dest.GetPositionY(), dest.GetPositionZ());
    }

    // Defender drivers, do nothing. Tactics for them will be added later.
    if (isDriver)
        return false;

    // On foot (vehicle got destroyed or bot dismounted).
    // Reset vehicle index so the next vehicle starts fresh, and clear any leftover siege position.
    // Data for pathing to the workshop was already reset when the bot got in the vehicle.
    m_vehicleWpIdx = 0;
    AI_VALUE(PositionMap&, "position")["bg siege"].Reset();

    // Defenders: Basic march to the fortress front court.
    if (!isAttacker)
    {
        float bestDist = FLT_MAX;
        uint8 nearest = 0;
        for (uint8 i = 0; i < WG_DEFENDER_WAYPOINT_COUNT; ++i)
        {
            float d = bot->GetDistance(WG_DEFENDER_WAYPOINTS[i].GetPositionX(),
                                       WG_DEFENDER_WAYPOINTS[i].GetPositionY(),
                                       WG_DEFENDER_WAYPOINTS[i].GetPositionZ());
            if (d < bestDist)
            {
                bestDist = d;
                nearest = i;
            }
        }
        m_defenderWpIdx = nearest;

        if (m_defenderWpIdx < WG_DEFENDER_WAYPOINT_COUNT - 1)
            ++m_defenderWpIdx;

        return MoveTo(bot->GetMapId(), WG_DEFENDER_WAYPOINTS[m_defenderWpIdx].GetPositionX(),
                      WG_DEFENDER_WAYPOINTS[m_defenderWpIdx].GetPositionY(),
                      WG_DEFENDER_WAYPOINTS[m_defenderWpIdx].GetPositionZ());
    }

    // Attackers to workshops:
    // When bots attains the rank First Lieutenant, a portion of them will be sent to the nearest friendly workshop to
    // summon a demolisher, if vehicle slots are available. If there are no slots, fall through to the fortress advance below.
    // Navigation uses the inverse of the vehicle path from fortress to workshop. Index 0 of each path is the engineer.
    // m_workshopWpIdx steps down one waypoint at a time, when the bot arrives within WG_WORKSHOP_WP_ARRIVE_DIST of the next
    // waypoint, preventing redundant MMAPS pathfinding calls to the same coordinates.
    // TODO: Consider using this method for all pathing.
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
            struct WgWorkshopPath { uint8 workshopId; WgVehicleWp const* wps; uint8 wpMax; };
            static WgWorkshopPath const WG_WORKSHOP_PATHS[] = {
                { 0, WG_NE_VEHICLE_WAYPOINTS, WG_NE_VEHICLE_WAYPOINT_COUNT },   // NE - Sunken Ring
                { 1, WG_NW_VEHICLE_WAYPOINTS, WG_NW_VEHICLE_WAYPOINT_COUNT },   // NW - Broken Temple
            };
            static constexpr uint8 WG_WORKSHOP_PATH_COUNT   = 2;
            static constexpr float WG_WORKSHOP_WP_ARRIVE_DIST = 10.0f;

            // Find the nearest friendly workshop to select the path.
            float bestDist   = FLT_MAX;
            uint8 bestPathId = 0xFF;
            for (uint8 i = 0; i < WG_WORKSHOP_PATH_COUNT; ++i)
            {
                if (wg->GetWorkshopTeam(WG_WORKSHOP_PATHS[i].workshopId) != team)
                    continue;

                float d = bot->GetDistance(WG_WORKSHOP_PATHS[i].wps[0].pos.GetPositionX(),
                                           WG_WORKSHOP_PATHS[i].wps[0].pos.GetPositionY(),
                                           WG_WORKSHOP_PATHS[i].wps[0].pos.GetPositionZ());
                if (d < bestDist) { bestDist = d; bestPathId = i; }
            }

            if (bestPathId != 0xFF)
            {
                // Reset persistent state when the target workshop changes. Needed when workshop capturing is implemented.
                if (m_workshopPathId != bestPathId)
                {
                    m_workshopPathId = bestPathId;
                    m_workshopWpIdx  = 0xFF;
                }

                WgVehicleWp const* wps   = WG_WORKSHOP_PATHS[bestPathId].wps;
                uint8              wpMax = WG_WORKSHOP_PATHS[bestPathId].wpMax;

                // On first entry, scan all waypoints to snap onto the nearest point of the path.
                if (m_workshopWpIdx == 0xFF)
                {
                    float wpBestDist = FLT_MAX;
                    uint8 nearest    = 0;
                    for (uint8 i = 0; i < wpMax; ++i)
                    {
                        float d = bot->GetDistance(wps[i].pos.GetPositionX(),
                                                   wps[i].pos.GetPositionY(),
                                                   wps[i].pos.GetPositionZ());
                        if (d < wpBestDist) { wpBestDist = d; nearest = i; }
                    }
                    m_workshopWpIdx = (nearest > 0) ? nearest - 1 : 0;
                }

                // Enforce the going-to-workshop cap before committing to this path.
                bool capOk = m_goingToWorkshop ||
                             (s_WgBotsGoingToWorkshop < static_cast<int32>(WORKSHOP_GO_MULTIPLIER * availSlots));
                if (!capOk) // At capacity. Don't send this bot to the workshop.
                {
                    // Release slot if held and fall through to fortress march. Also clear the workshop waypoint index.
                    if (m_goingToWorkshop)
                    {
                        m_goingToWorkshop = false;
                        --s_WgBotsGoingToWorkshop;
                    }
                    m_workshopWpIdx = 0xFF;
                }
                else
                {
                    if (!m_goingToWorkshop)
                    {
                        m_goingToWorkshop = true;
                        ++s_WgBotsGoingToWorkshop;
                    }

                    // Decrement m_workshopWpIdx once within WG_WORKSHOP_WP_ARRIVE_DIST of the current waypoint.
                    if (m_workshopWpIdx > 0)
                    {
                        float d = bot->GetDistance(wps[m_workshopWpIdx].pos.GetPositionX(),
                                                   wps[m_workshopWpIdx].pos.GetPositionY(),
                                                   wps[m_workshopWpIdx].pos.GetPositionZ());
                        if (d <= WG_WORKSHOP_WP_ARRIVE_DIST)
                            --m_workshopWpIdx;
                    }

                    Position const& dest = wps[m_workshopWpIdx].pos;
                    return MoveTo(bot->GetMapId(), dest.GetPositionX(), dest.GetPositionY(), dest.GetPositionZ());
                }
            }
        }
    }

    // Not heading to a workshop. Release slot if held.
    if (m_goingToWorkshop)
    {
        m_goingToWorkshop = false;
        --s_WgBotsGoingToWorkshop;
    }

    // No vehicle slot available, or below First Lieutenant: march toward the fortress on foot.
    bool isHorde = (bot->GetTeamId() == TEAM_HORDE);
    Position const* attackWps   = isHorde ? WG_HORDE_ATTACKER_WAYPOINTS     : WG_ALLIANCE_ATTACKER_WAYPOINTS;
    uint8           attackWpMax = isHorde ? WG_HORDE_ATTACKER_WAYPOINT_COUNT : WG_ALLIANCE_ATTACKER_WAYPOINTS_COUNT;

    float bestDist = FLT_MAX;
    uint8 nearest = 0;
    for (uint8 i = 0; i < attackWpMax; ++i)
    {
        float d = bot->GetDistance(attackWps[i].GetPositionX(),
                                   attackWps[i].GetPositionY(),
                                   attackWps[i].GetPositionZ());
        if (d < bestDist) { bestDist = d; nearest = i; }
    }
    m_attackerWpIdx = nearest;

    if (m_attackerWpIdx < attackWpMax - 1)
        ++m_attackerWpIdx;

    return MoveTo(bot->GetMapId(), attackWps[m_attackerWpIdx].GetPositionX(),
                  attackWps[m_attackerWpIdx].GetPositionY(),
                  attackWps[m_attackerWpIdx].GetPositionZ());
}

bool WgSummonVehicleAction::Execute(Event /*event*/)
{
    // Must have First Lieutenant rank to summon a Demolisher.
    if (!bot->HasAura(SPELL_LIEUTENANT))
        return false;

    // Already in a vehicle; EnterVehicleAction will handle entry.
    if (bot->GetVehicle())
        return false;

    // Check that WG is in wartime and bot is an attacker. Defenders fight only on foot (for now).
    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg || !wg->IsWarTime())
        return false;
    if (bot->GetTeamId() == wg->GetDefenderTeam())
        return false;

    TeamId team = bot->GetTeamId();
    uint32 dataVeh = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_VEHICLE_H : BATTLEFIELD_WG_DATA_VEHICLE_A;
    uint32 dataMax = (team == TEAM_HORDE) ? BATTLEFIELD_WG_DATA_MAX_VEHICLE_H : BATTLEFIELD_WG_DATA_MAX_VEHICLE_A;
    if (wg->GetData(dataVeh) >= wg->GetData(dataMax))
        return false;

    // Find the workshop engineer NPC. Search within ENGINEER_SCAN_RANGE yards in the workshop area.
    // WgCheckFlagAction navigates the bot to the workshop, but the exact engineer position may differ
    // from the approach waypoint, so 20 yards should be plenty
    static constexpr float ENGINEER_SCAN_RANGE = 20.0f;

    uint32 engineerEntry = (team == TEAM_HORDE) ? NPC_WG_GOBLIN_MECHANIC : NPC_WG_GNOMISH_ENGINEER;
    Creature* engineer = bot->FindNearestCreature(engineerEntry, ENGINEER_SCAN_RANGE, true);
    if (!engineer)
        return false;

    if (bot->GetDistance(engineer) > INTERACTION_DISTANCE)
        return MoveTo(engineer);

    // Dismount before interacting. Mount aura removal is synchronous, so the bot falls through to the
    // gossip in the same tick, rather than yielding and risking a remount/dismount cycle.
    if (bot->IsMounted())
    {
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
    // The demolisher is all around the best vehicle, but might use other vehicles too in the future
    if (!menu.GetItem(1))
        return false;

    WorldPacket select;
    std::string code;
    select << engineer->GetGUID();
    select << menu.GetMenuId() << uint32(1);
    select << code;
    bot->GetSession()->HandleGossipSelectOptionOpcode(select);

    return true;
}

bool WgMountTowerCannonAction::Execute(Event /*event*/)
{
    // Already got in a cannon.
    if (bot->GetVehicle())
        return false;

    BattlefieldWG* wg = GetBattlefieldWG();
    if (!wg || !wg->IsWarTime())
        return false;
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
            // normally on the lower level cannons, but on tower cannons they jump up in a cartoonish way, but they do mount.
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

    // Scan gate: only when the bot close enough to the central inner wall.
    if (bot->GetDistance(WG_INNER_WALL_POS.GetPositionX(),
                         WG_INNER_WALL_POS.GetPositionY(),
                         WG_INNER_WALL_POS.GetPositionZ()) > WG_CANNON_INNER_WALL_RANGE)
        return false;

    // Level-priority stagger: normalize bot levels into 6 groups (0–5). Higher level bots have higher initial delay to
    // scan for avaliable cannos, and also would make later scans less frequently. Why waste a level 80 on the cannon
    // when a level 75 can operate it exactly the same way?
    uint32 minLvl   = sWorld->getIntConfig(CONFIG_WINTERGRASP_PLR_MIN_LVL);
    uint32 lvlRange = (DEFAULT_MAX_LEVEL > minLvl) ? (DEFAULT_MAX_LEVEL - minLvl) : 1u;
    uint32 aboveMin = (bot->GetLevel() > minLvl) ? uint32(bot->GetLevel() - minLvl) : 0u;
    uint32 stagger  = (aboveMin * 5u) / lvlRange;   // 0–5

    uint32 now = getMSTime();
    if (m_cannonScanTime == 0)
        m_cannonScanTime = now + (WG_SCAN_INTERVAL * stagger);

    if (now < m_cannonScanTime)
        return false;
    m_cannonScanTime = now + (WG_SCAN_INTERVAL * (1u + stagger));

    // Collect all NPC_WINTERGRASP_TOWER_CANNON creatures that are within WG_CANNON_SCAN_RANGE yards from the bot.
    std::list<Creature*> nearby;
    bot->GetCreatureListWithEntryInGrid(nearby, NPC_WINTERGRASP_TOWER_CANNON, WG_CANNON_SCAN_RANGE);

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

Unit* WgFireCannonAction::GetTarget()
{
    // Scan for targets at slightly beyond the cannon's firing range (70 yards)
    static constexpr float CANNON_TARGET_SCAN = 80.0f;

    // WG assault vehicles that threaten the fortress: highest-priority targets.
    static uint32 const WG_ASSAULT_ENTRIES[] = {
        NPC_WINTERGRASP_DEMOLISHER,
        NPC_WINTERGRASP_CATAPULT,
        NPC_WINTERGRASP_SIEGE_ENGINE_ALLIANCE,
        NPC_WINTERGRASP_SIEGE_ENGINE_HORDE,
    };

    Creature* nearest     = nullptr;
    float     nearestDist = FLT_MAX;

    for (uint32 entry : WG_ASSAULT_ENTRIES)
    {
        std::list<Creature*> found;
        bot->GetCreatureListWithEntryInGrid(found, entry, CANNON_TARGET_SCAN);
        for (Creature* c : found)
        {
            if (!c->IsAlive() || !bot->IsHostileTo(c))
                continue;
            float d = bot->GetDistance(c);
            if (d < nearestDist)
            {
                nearestDist = d;
                nearest     = c;
            }
        }
    }

    if (nearest)
        return nearest;

    // No hostile vehicle in range. Fall back to normal hostile targets.
    return CastVehicleSpellAction::GetTarget();
}
