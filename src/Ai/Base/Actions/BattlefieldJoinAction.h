/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
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
