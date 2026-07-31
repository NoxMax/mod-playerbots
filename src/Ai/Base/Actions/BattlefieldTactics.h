/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef _PLAYERBOT_BATTLEFIELDTACTICS_H
#define _PLAYERBOT_BATTLEFIELDTACTICS_H

#include "GenericSpellActions.h"
#include "MovementActions.h"
#include <vector>

class BattlefieldWG;
class Creature;

// NPC and spell entries from zone_wintergrasp.cpp
static constexpr uint32 NPC_WG_GOBLIN_MECHANIC  = 30400;    // Horde workshop engineer
static constexpr uint32 NPC_WG_GNOMISH_ENGINEER = 30499;    // Alliance workshop engineer
static constexpr uint32 SPELL_VEHICLE_TELEPORT  = 49759;    // Aura applied by the fortress vehicle teleporter

struct WgWaypoint
{
    float  x, y, z;
    uint32 pathBlock = 0;       // WorldState ID of the building blocking this waypoint (0 = none)
    bool   noSkip    = false;   // true = bot must reach this node before advancing past it
};

using WgPath = std::vector<WgWaypoint>;

class WgCheckFlagAction : public MovementAction
{
public:
    WgCheckFlagAction(PlayerbotAI* botAI) : MovementAction(botAI, "wg check flag"),
        m_botGuidRaw(0), m_routeStep(0), m_atkVehiclePhase(0), m_defVehiclePhase(0),
        m_atkGoingToWorkshop(false), m_defGoingToWorkshop(false), m_workshopIdx(0xFF),
        m_defGuardFortress(0), m_defGuardFortressTime(0), m_warScanTime(0), m_targetTowerIdx(0xFF),
        m_isTowerAttacker(false), m_captureWsIdx(0xFF), m_arrivedAtCapture(false)
    {
        // Cached here so the destructor can remove this bot from s_WgCapturingWorkshop.
        // Bot may no longer be valid when the destructor runs.
        if (bot) m_botGuidRaw = bot->GetGUID().GetRawValue();
    }
    ~WgCheckFlagAction() override;
    bool Execute(Event event) override;

    // Public because WgMountTowerCannonAction stages its cannon approach through it.
    bool FollowWgRoute(Position const& objective, bool checkPathBlock);

    // Used by combat target value filters to suppress combat for workshop capture bots.
    static bool IsCapturingWorkshop(Player* bot);

    // Public because BfStrategyCheckAction must call it when it deactivates the strategy at end of battle.
    void ResetBattleState();

private:
    void ClearSharedTracking();
    bool TryCaptureWorkshop(BattlefieldWG* wg);
    void FallBackToGuardOrWar(bool whenTheWallsFell);
    Creature* AcquireWarTarget();

    uint64_t            m_botGuidRaw;           // Cached bot GUID for safe use in destructor
    std::vector<uint32> m_route;
    uint32              m_routeStep;
    uint8               m_atkVehiclePhase;
    uint8               m_defVehiclePhase;
    bool                m_atkGoingToWorkshop;
    bool                m_defGoingToWorkshop;
    uint8               m_workshopIdx;          // Index into WG_WORKSHOPS[]
    uint8               m_defGuardFortress;     // 0=unassigned, 1=hisGuardAtWall, 2=hisGuardAtGate, 3=hisGuardAtWar (hunt), 4=hisGuardAtCourt
    uint32              m_defGuardFortressTime; // Timestamp of last guard position assignment evaluation
    ObjectGuid          m_warTarget;            // hisGuardAtWar: the attacker vehicle this hunter is going after (empty = none)
    uint32              m_warScanTime;          // hisGuardAtWar: timestamp of last hunt target scan
    uint8               m_targetTowerIdx;       // Index into DEF_TOWERS[]
    bool                m_isTowerAttacker;      // True while this bot holds a tower squad slot
    uint8               m_captureWsIdx;         // Workshop this bot is assigned to capture (0xFF = none)
    bool                m_arrivedAtCapture;     // Latch: set on first arrival; suppression never re-activates
                                                //   until assignment changes
};

class WgSummonVehicleAction : public MovementAction
{
public:
    WgSummonVehicleAction(PlayerbotAI* botAI) : MovementAction(botAI, "wg summon vehicle") {}
    bool Execute(Event event) override;
};

class WgMountTowerCannonAction : public MovementAction
{
public:
    WgMountTowerCannonAction(PlayerbotAI* botAI) : MovementAction(botAI, "wg mount tower cannon") {}
    bool Execute(Event event) override;

private:
    void ResetCannonState();

    ObjectGuid m_targetCannon;
    uint32     m_cannonScanTime = 0;
};

class WgFireCannonAction : public CastVehicleSpellAction
{
public:
    WgFireCannonAction(PlayerbotAI* botAI) : CastVehicleSpellAction(botAI, "fire cannon") {}
    Unit* GetTarget() override;
};

class WgHurlBoulderAction : public CastVehicleSpellAction
{
public:
    WgHurlBoulderAction(PlayerbotAI* botAI) : CastVehicleSpellAction(botAI, "hurl boulder") {}
    Unit* GetTarget() override;
};

#endif
