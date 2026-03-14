/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#include "BattlefieldJoinAction.h"

#include "AreaDefines.h"
#include "Battlefield.h"
#include "BattlefieldMgr.h"
#include "Event.h"
#include "Opcodes.h"
#include "PlayerbotAI.h"
#include "Playerbots.h"
#include "WorldPacket.h"

static bool AcceptQueueInvite(Player* bot, uint32 battleId)
{
    if (!sBattlefieldMgr->GetBattlefieldByBattleId(battleId))
        return false;

    WorldPacket response(CMSG_BATTLEFIELD_MGR_QUEUE_INVITE_RESPONSE, 5);
    response << battleId << uint8(1);  // Accepted = 1
    bot->GetSession()->HandleBfQueueInviteResponse(response);
    return true;
}

static bool AcceptEntryInvite(Player* bot, PlayerbotAI* botAI, uint32 battleId)
{
    if (!sBattlefieldMgr->GetBattlefieldByBattleId(battleId))
        return false;

    if (sRandomPlayerbotMgr.IsRandomBot(bot))
        botAI->SetMaster(nullptr);

    WorldPacket response(CMSG_BATTLEFIELD_MGR_ENTRY_INVITE_RESPONSE, 5);
    response << battleId << uint8(1);  // Accepted = 1
    bot->GetSession()->HandleBfEntryInviteResponse(response);
    return true;
}

bool BfStrategyCheckAction::Execute(Event event)
{
    bool inWG = bot->GetZoneId() == AREA_WINTERGRASP;
    Battlefield* wg = sBattlefieldMgr->GetBattlefieldByBattleId(BATTLEFIELD_BATTLEID_WG);
    bool wartime = wg && wg->IsWarTime();

    // Process pending WG invites (stored when packets arrived, processed here to work in combat mode)
    if (botAI->pendingWgQueueInviteBattleId)
    {
        uint32 battleId = botAI->pendingWgQueueInviteBattleId;
        botAI->pendingWgQueueInviteBattleId = 0;
        return AcceptQueueInvite(bot, battleId);
    }

    if (botAI->pendingWgEntryInviteBattleId)
    {
        uint32 battleId = botAI->pendingWgEntryInviteBattleId;
        botAI->pendingWgEntryInviteBattleId = 0;
        return AcceptEntryInvite(bot, botAI, battleId);
    }

    // Check if strategy is active in either combat or non-combat state
    bool hasWGStratNonCombat = botAI->HasStrategy("wintergrasp", BOT_STATE_NON_COMBAT);
    bool hasWGStratCombat = botAI->HasStrategy("wintergrasp", BOT_STATE_COMBAT);
    bool hasWGStrat = hasWGStratNonCombat || hasWGStratCombat;

    if (inWG && wartime && !hasWGStrat)
    {
        botAI->ChangeStrategy("+wintergrasp", BOT_STATE_NON_COMBAT);
        botAI->ChangeStrategy("+wintergrasp", BOT_STATE_COMBAT);
        LOG_INFO("playerbots", "Bot {} <{}> activates Wintergrasp strategy", bot->GetGUID().ToString(), bot->GetName());
        return true;
    }
    if ((!inWG || !wartime) && hasWGStrat)
    {
        botAI->ChangeStrategy("-wintergrasp", BOT_STATE_NON_COMBAT);
        botAI->ChangeStrategy("-wintergrasp", BOT_STATE_COMBAT);
        LOG_INFO("playerbots", "Bot {} <{}> deactivates Wintergrasp strategy", bot->GetGUID().ToString(), bot->GetName());
        // Remove random bots from their group when the WG battle ends. Group::RemoveMember handles auto-disband
        // when the last member leaves. TODO: This group cleaning logic exists in core for leaving BGs, but not
        // for WG (MAR 2026), so this cleanup will become redundant if that's added later in core.
        if (sRandomPlayerbotMgr.IsRandomBot(bot))
            if (Group* group = bot->GetGroup())
                group->RemoveMember(bot->GetGUID());

        return true;
    }
    return false;
}

// SMSG_BATTLEFIELD_MGR_QUEUE_INVITE packet layout:
//   uint32 BattleId
//   uint8  warmup (unused by us)
bool AcceptBfQueueInviteAction::Execute(Event event)
{
    WorldPacket p(event.getPacket());
    uint32 battleId;
    p >> battleId;
    return AcceptQueueInvite(bot, battleId);
}

// SMSG_BATTLEFIELD_MGR_ENTRY_INVITE packet layout:
//   uint32 BattleId
//   uint32 ZoneId
//   uint32 expiry time (game time + accept window)
bool AcceptBfEntryInviteAction::Execute(Event event)
{
    WorldPacket p(event.getPacket());
    uint32 battleId;
    p >> battleId;
    return AcceptEntryInvite(bot, botAI, battleId);
}
