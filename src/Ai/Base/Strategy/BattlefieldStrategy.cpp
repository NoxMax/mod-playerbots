/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#include "BattlefieldStrategy.h"

#include "Playerbots.h"

BfStrategy::BfStrategy(PlayerbotAI* botAI) : PassTroughStrategy(botAI) {}

void BfStrategy::InitTriggers(std::vector<TriggerNode*>& triggers)
{
    // Respond to Wintergrasp enrollment packets as they arrive
    triggers.push_back(new TriggerNode("wg queue invite", {NextAction("accept bf queue invite", ACTION_EMERGENCY)}));
    triggers.push_back(new TriggerNode("wg entry invite", {NextAction("accept bf entry invite", ACTION_EMERGENCY)}));

    // Every second: a cheap check on whether to activate/deactivate WintergraspStrategy.
    triggers.push_back(new TriggerNode("timer", {NextAction("bf strategy check", relevance)}));

    // Every 60 seconds: guaranteed re-check at ACTION_EMERGENCY, mirroring AV's "timer bg" -> "bg reset objective force".
    triggers.push_back(new TriggerNode("timer bg", {NextAction("bf strategy check", ACTION_EMERGENCY)}));
}

std::vector<NextAction> WintergraspStrategy::getDefaultActions()
{
    return { NextAction("wg check flag", ACTION_MOVE + 1.0f) };
}

void WintergraspStrategy::InitTriggers(std::vector<TriggerNode*>& triggers)
{
    // Re-engage WG objective after respawn.
    triggers.push_back(new TriggerNode("dead", {NextAction("bf strategy check", ACTION_EMERGENCY)}));

    // This is an active injection of the default navigation strategy. There may be an issue with getDefaultActions failing
    // to trigger as the fallback sometimes. This injection seems to help. TODO: Investigate the necessity of this.
    triggers.push_back(new TriggerNode("timer", {NextAction("wg check flag", ACTION_MOVE + 1.0f)}));

    // Mount/dismount for travel; re-registered above NonCombatStrategy's priority so WG timer triggers don't starve it.
    triggers.push_back(new TriggerNode("timer", {NextAction("check mount state", ACTION_MOVE + 2.0f)}));

    // Defenders: board a free fortress cannon.
    triggers.push_back(new TriggerNode("timer", {NextAction("wg mount tower cannon", ACTION_MOVE + 4.0f)}));

    // Gossip with workshop engineer to summon a vehicle.
    triggers.push_back(new TriggerNode("timer", {NextAction("wg summon vehicle", ACTION_MOVE + 5.0f)}));

    // Enter a nearby friendly vehicle (Demolisher/Catapult/Siege Engine).
    triggers.push_back(new TriggerNode("timer", {NextAction("enter vehicle", ACTION_MOVE + 8.0f)}));

    // Vehicle combat abilities; isPossible() limits each to applicable vehicle types.
    // Low priority here for moving vehicles against creatures. Attacks on buildings and vehicles are handled in BattlefieldTactics.
    triggers.push_back(new TriggerNode("in vehicle", {NextAction("hurl boulder", ACTION_MOVE + 1.0f)}));
    triggers.push_back(new TriggerNode("in vehicle", {NextAction("ram", ACTION_MOVE + 1.0f)}));
    triggers.push_back(new TriggerNode("in vehicle", {NextAction("wg fire cannon", ACTION_MOVE + 9.0f)}));
}
