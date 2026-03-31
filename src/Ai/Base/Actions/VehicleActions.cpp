/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#include "VehicleActions.h"

#include "BattlegroundIC.h"
#include "BattlefieldWG.h"
#include "ItemVisitors.h"
#include "ObjectDefines.h"
#include "Playerbots.h"
#include "QuestValues.h"
#include "ServerFacade.h"
#include "Unit.h"
#include "Vehicle.h"

// TODO methods to enter/exit vehicle should be added to BGTactics or MovementAction (so that we can better control
// whether bot is in vehicle, eg: get out of vehicle to cap flag, if we're down to final boss, etc),
// right now they will enter vehicle based only what's available here, then they're stuck in vehicle until they die
// (LeaveVehicleAction doesnt do much seeing as they, or another bot, will get in immediately after exit)
bool EnterVehicleAction::Execute(Event event)
{
    // do not switch vehicles yet
    if (bot->GetVehicle())
        return false;

    Player* master = botAI->GetMaster();
    // Triggered by a chat command
    if (event.getOwner() && master && master->GetTarget())
    {
        Unit* vehicleBase = botAI->GetUnit(master->GetTarget());
        if (!vehicleBase)
            return false;
        Vehicle* veh = vehicleBase->GetVehicleKit();
        if (vehicleBase->IsVehicle() && veh && veh->GetAvailableSeatCount())
        {
            return EnterVehicle(vehicleBase, false);
        }
        return false;
    }

    GuidVector npcs = AI_VALUE(GuidVector, "nearest vehicles");
    for (GuidVector::iterator i = npcs.begin(); i != npcs.end(); i++)
    {
        Unit* vehicleBase = botAI->GetUnit(*i);
        if (!vehicleBase)
            continue;

        if (vehicleBase->HasUnitFlag(UNIT_FLAG_NOT_SELECTABLE))
            continue;

        // Faction check before the more expensive lookups
        if (!vehicleBase->IsFriendlyTo(bot))
            continue;

        uint32 entry = vehicleBase->GetEntry();

        // IoC-specific: dont let bots get in the cannons as they'll stay forever and do nothing useful,
        // and the catapults can't be used by bots since they are non-attacking vehicles.
        if (entry == NPC_KEEP_CANNON || entry == NPC_CATAPULT)
            continue;

        // Wintergrasp vehicles are custom handled in BattlefieldTactics.
        // Do not add an exception to the siege engine turret though, so that a secondary driver can simply
        // hitch a ride with the main driver and use the cannon.
        if (entry == NPC_WINTERGRASP_TOWER_CANNON           ||
            entry == NPC_WINTERGRASP_CATAPULT               ||
            entry == NPC_WINTERGRASP_DEMOLISHER             ||
            entry == NPC_WINTERGRASP_SIEGE_ENGINE_ALLIANCE  ||
            entry == NPC_WINTERGRASP_SIEGE_ENGINE_HORDE)
            continue;

        Vehicle* vehKit = vehicleBase->GetVehicleKit();
        if (!vehKit || !vehKit->GetAvailableSeatCount())
            continue;

        // Avoid adding passengers; they do little for IoC or WG vehicles.
        if (vehKit->IsVehicleInUse())
            continue;

        if (EnterVehicle(vehicleBase, true))
            return true;
    }

    return false;
}

bool EnterVehicleAction::EnterVehicle(Unit* vehicleBase, bool moveIfFar)
{
    float dist = ServerFacade::instance().GetDistance2d(bot, vehicleBase);
    if (dist > 40.0f)
        return false;

    if (dist > INTERACTION_DISTANCE && !moveIfFar)
        return false;

    if (dist > INTERACTION_DISTANCE)
        return MoveTo(vehicleBase);
    // Use HandleSpellClick instead of Unit::EnterVehicle to handle special vehicle script (ulduar)
    vehicleBase->HandleSpellClick(bot);

    if (!bot->IsOnVehicle(vehicleBase))
        return false;

    // dismount because bots can enter vehicle on mount
    WorldPacket emptyPacket;
    bot->GetSession()->HandleCancelMountAuraOpcode(emptyPacket);
    return true;
}

bool LeaveVehicleAction::Execute(Event /*event*/)
{
    Vehicle* myVehicle = bot->GetVehicle();
    if (!myVehicle)
        return false;

    VehicleSeatEntry const* seat = myVehicle->GetSeatForPassenger(bot);
    if (!seat || !seat->CanEnterOrExit())
        return false;

    WorldPacket p;
    bot->GetSession()->HandleRequestVehicleExit(p);

    return true;
}
