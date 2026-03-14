/*
 * Copyright (C) 2016+ AzerothCore <www.azerothcore.org>, released under GNU AGPL v3 license, you may redistribute it
 * and/or modify it under version 3 of the License, or (at your option), any later version.
 */

#ifndef _PLAYERBOT_BATTLEFIELDJOINACTION_H
#define _PLAYERBOT_BATTLEFIELDJOINACTION_H

#include "Action.h"

// Periodically checks whether to activate/deactivate WintergraspStrategy based on zone + war state.
class BfStrategyCheckAction : public Action
{
public:
    BfStrategyCheckAction(PlayerbotAI* botAI) : Action(botAI, "bf strategy check") {}
    bool Execute(Event event) override;
};

// Responds to SMSG_BATTLEFIELD_MGR_QUEUE_INVITE (sent 15 min before war starts)
// by accepting the queue invite via CMSG_BATTLEFIELD_MGR_QUEUE_INVITE_RESPONSE
class AcceptBfQueueInviteAction : public Action
{
public:
    AcceptBfQueueInviteAction(PlayerbotAI* botAI) : Action(botAI, "accept bf queue invite") {}
    bool Execute(Event event) override;
};

// Responds to SMSG_BATTLEFIELD_MGR_ENTRY_INVITE (sent when war starts)
// by accepting the war entry via CMSG_BATTLEFIELD_MGR_ENTRY_INVITE_RESPONSE
class AcceptBfEntryInviteAction : public Action
{
public:
    AcceptBfEntryInviteAction(PlayerbotAI* botAI) : Action(botAI, "accept bf entry invite") {}
    bool Execute(Event event) override;
};

#endif
