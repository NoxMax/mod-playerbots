/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#include "BattlefieldJoinAction.h"

#include "AreaDefines.h"
#include "Battlefield.h"
#include "BattlefieldMgr.h"
#include "BattlefieldTactics.h"
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

bool BfStrategyCheckAction::Execute(Event /*event*/)
{
    bool inActiveWG = bot->InBattlefield();

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

    if (inActiveWG && !hasWGStrat)
    {
        botAI->ChangeStrategy("+wintergrasp", BOT_STATE_NON_COMBAT);
        botAI->ChangeStrategy("+wintergrasp", BOT_STATE_COMBAT);
        LOG_INFO("playerbots", "Bot {} <{}> activates Wintergrasp strategy", bot->GetGUID().ToString(), bot->GetName());
        return true;
    }
    if (!inActiveWG && hasWGStrat)
    {
        // Primary battle reset is done here rather than in BattlefieldTactics, since BfStrategyCheckAction::Execute
        // outranks WgCheckFlagAction::Execute. The latter Execute is also removed by "-wintergrasp" below.
        if (Action* action = botAI->GetAiObjectContext()->GetAction("wg check flag"))
            static_cast<WgCheckFlagAction*>(action)->ResetBattleState();

        botAI->ChangeStrategy("-wintergrasp", BOT_STATE_NON_COMBAT);
        botAI->ChangeStrategy("-wintergrasp", BOT_STATE_COMBAT);
        LOG_INFO("playerbots", "Bot {} <{}> deactivates Wintergrasp strategy", bot->GetGUID().ToString(), bot->GetName());
        return true;
    }
    return false;
}
