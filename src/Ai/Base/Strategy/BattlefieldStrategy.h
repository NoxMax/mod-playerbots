/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef _PLAYERBOT_BATTLEFIELDSTRATEGY_H
#define _PLAYERBOT_BATTLEFIELDSTRATEGY_H

#include "PassThroughStrategy.h"
#include "Strategy.h"

// Always active WG lifecycle strategy. Accepts queue/entry invites and periodically fires BfStrategyCheckAction
// to toggle WintergraspStrategy based on war state.
class BfStrategy : public PassThroughStrategy
{
public:
    BfStrategy(PlayerbotAI* botAI);

    uint32 GetType() const override { return STRATEGY_TYPE_NONCOMBAT; }
    void InitTriggers(std::vector<TriggerNode*>& triggers) override;
    std::string const getName() override { return "bf"; }
};

// Active during WG war for enrolled bots. Responsible for foot and vehicle navigation, acquiring and using
// vehicles, and fires BfStrategyCheckAction on respawn after death.
class WintergraspStrategy : public Strategy
{
public:
    WintergraspStrategy(PlayerbotAI* botAI) : Strategy(botAI){};

    uint32 GetType() const override { return STRATEGY_TYPE_GENERIC; }
    std::vector<NextAction> getDefaultActions() override;
    void InitTriggers(std::vector<TriggerNode*>& triggers) override;
    std::string const getName() override { return "wintergrasp"; }
};

#endif
