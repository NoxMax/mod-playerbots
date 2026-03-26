/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#ifndef _PLAYERBOT_BATTLEFIELDTACTICS_H
#define _PLAYERBOT_BATTLEFIELDTACTICS_H

#include "GenericSpellActions.h"
#include "MovementActions.h"

class WgCheckFlagAction : public MovementAction
{
public:
    WgCheckFlagAction(PlayerbotAI* botAI) : MovementAction(botAI, "wg check flag"),
        m_attackerWpIdx(0), m_defenderWpIdx(0), m_vehicleWpIdx(0),
        m_workshopWpIdx(0xFF), m_workshopPathId(0xFF), m_goingToWorkshop(false) {}
    ~WgCheckFlagAction() override;
    bool Execute(Event event) override;
private:
    uint8 m_attackerWpIdx;      // On foot attacker path index to fortress, from spawn.
    uint8 m_defenderWpIdx;      // On foot defender path index to fortress, from graveyard behind fortress.
    uint8 m_vehicleWpIdx;       // In vehicle attacker path index to fortress, from friendly workshop.
    uint8 m_workshopWpIdx;      // On foot attacker path index to friendly workshop. Inverse of m_vehicleWpIdx.
    uint8 m_workshopPathId;     // Active path index in WG_WORKSHOP_PATHS being used.
    bool  m_goingToWorkshop;    // True when this bot is counted in s_WgBotsGoingToWorkshop.
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
    ObjectGuid m_targetCannon;          // GUID of the cannon the bot is heading to.
    uint32     m_cannonScanTime = 0;    // Timestamp of the next scan for free cannons. Staggered per bot level.
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
