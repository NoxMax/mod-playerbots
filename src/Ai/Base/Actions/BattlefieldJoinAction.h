/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef _PLAYERBOT_BATTLEFIELDJOINACTION_H
#define _PLAYERBOT_BATTLEFIELDJOINACTION_H

#include "Action.h"

// Periodically checks whether to activate/deactivate WintergraspStrategy based on zone + war state.
// Also accepts pending WG queue/entry invites (stored by PlayerbotAI::HandleBotOutgoingPacket) by
// responding with CMSG_BATTLEFIELD_MGR_QUEUE_INVITE_RESPONSE / CMSG_BATTLEFIELD_MGR_ENTRY_INVITE_RESPONSE.
class BfStrategyCheckAction : public Action
{
public:
    BfStrategyCheckAction(PlayerbotAI* botAI) : Action(botAI, "bf strategy check") {}
    bool Execute(Event event) override;
};

#endif
