/*
 * Copyright (c) 2012-2015 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <sstream>
#include <iostream>
#include "RoboCupGameControlData.h"
#include <alproxies/almemoryproxy.h>
#include <boost/thread.hpp>

int main(int argc, char *argv[])
{
  RoboCupGameControlData gameCtrlData;
  if(argc == 3)
  {
    int team_number = 0;
    int player_number = 0;
    team_number = atoi(argv[1]);
    player_number = atoi(argv[2]);
    AL::ALMemoryProxy memory("127.0.0.1",9559);
    memory.insertData("GameCtrl/teamNumber", team_number);
    memory.insertData("GameCtrl/playerNumber", player_number);
    int seq = 0;
    while(true)
    {
      AL::ALValue value = memory.getData("GameCtrl/RoboCupGameControlData");
      if(value.isBinary() && value.getSize() == sizeof(RoboCupGameControlData))
      {
        memcpy(&gameCtrlData, value, sizeof(RoboCupGameControlData));
        if(seq != (int)gameCtrlData.packetNumber)
        {
          memory.insertData("gc-bridge/packetNumber", (int)gameCtrlData.packetNumber);
          std::string header_str(gameCtrlData.header, 4);
          memory.insertData("gc-bridge/header", header_str);
          memory.insertData("gc-bridge/version", (int)gameCtrlData.version);
          memory.insertData("gc-bridge/playersPerTeam", (int)gameCtrlData.playersPerTeam);
          memory.insertData("gc-bridge/gameType", (int)gameCtrlData.gameType);
          memory.insertData("gc-bridge/state", (int)gameCtrlData.state);
          memory.insertData("gc-bridge/firstHalf", (int)gameCtrlData.firstHalf);
          memory.insertData("gc-bridge/kickOffTeam", (int)gameCtrlData.kickOffTeam);
          memory.insertData("gc-bridge/secondaryState", (int)gameCtrlData.secondaryState);
          memory.insertData("gc-bridge/dropInTeam", (int)gameCtrlData.dropInTeam);
          memory.insertData("gc-bridge/dropInTime", (int)gameCtrlData.dropInTime);
          memory.insertData("gc-bridge/secsRemaining", (int)gameCtrlData.secsRemaining);
          memory.insertData("gc-bridge/secondaryTime", (int)gameCtrlData.secondaryTime);

          //update team0 value
          memory.insertData("gc-bridge/team0_teamNumber", (int)gameCtrlData.teams[0].teamNumber);
          memory.insertData("gc-bridge/team0_teamColour", (int)gameCtrlData.teams[0].teamColour);
          memory.insertData("gc-bridge/team0_score", (int)gameCtrlData.teams[0].score);
          memory.insertData("gc-bridge/team0_penaltyShot", (int)gameCtrlData.teams[0].penaltyShot);
          memory.insertData("gc-bridge/team0_singleShots", (int)gameCtrlData.teams[0].singleShots);
          memory.insertData("gc-bridge/team0_coachSequence", (int)gameCtrlData.teams[0].coachSequence);
          int team0_coachMessage[SPL_COACH_MESSAGE_SIZE];
          for(int i = 0; i < SPL_COACH_MESSAGE_SIZE ;i++)
          {
            team0_coachMessage[i] = (int)gameCtrlData.teams[0].coachMessage[i];
          }
          memory.insertData("gc-bridge/team0_coachMessage", team0_coachMessage);
          memory.insertData("gc-bridge/team0_coach_penalty", (int)gameCtrlData.teams[0].coach.penalty);
          memory.insertData("gc-bridge/team0_coach_secsTillUnpenalised", (int)gameCtrlData.teams[0].coach.secsTillUnpenalised);
          for(int i = 0; i < MAX_NUM_PLAYERS ;i++)
          {
            std::stringstream ss;
            ss << i + 1;
            std::string key_penalty = "gc-bridge/team0_player" + ss.str() + "_penalty";
            memory.insertData(key_penalty,(int)gameCtrlData.teams[0].players[i].penalty);
            std::string key_until_penalized = "gc-bridge/team0_player" + ss.str() + "_secsTillUnpenalised";
            memory.insertData(key_until_penalized,(int)gameCtrlData.teams[0].players[i].secsTillUnpenalised);
          }

          //update team1 value
          memory.insertData("gc-bridge/team1_teamNumber", (int)gameCtrlData.teams[1].teamNumber);
          memory.insertData("gc-bridge/team1_teamColour", (int)gameCtrlData.teams[1].teamColour);
          memory.insertData("gc-bridge/team1_score", (int)gameCtrlData.teams[1].score);
          memory.insertData("gc-bridge/team1_penaltyShot", (int)gameCtrlData.teams[1].penaltyShot);
          memory.insertData("gc-bridge/team1_singleShots", (int)gameCtrlData.teams[1].singleShots);
          memory.insertData("gc-bridge/team1_coachSequence", (int)gameCtrlData.teams[1].coachSequence);
          int team1_coachMessage[SPL_COACH_MESSAGE_SIZE];
          for(int i = 0; i < SPL_COACH_MESSAGE_SIZE ;i++)
          {
            team1_coachMessage[i] = (int)gameCtrlData.teams[1].coachMessage[i];
          }
          memory.insertData("gc-bridge/team1_coachMessage", team1_coachMessage);
          memory.insertData("gc-bridge/team1_coachMessage", team0_coachMessage);
          memory.insertData("gc-bridge/team1_coach_penalty", (int)gameCtrlData.teams[0].coach.penalty);
          memory.insertData("gc-bridge/team1_coach_secsTillUnpenalised", (int)gameCtrlData.teams[0].coach.secsTillUnpenalised);

          for(int i = 0; i < MAX_NUM_PLAYERS ;i++)
          {
            std::stringstream ss;
            ss << i + 1;
            std::string key_penalty = "gc-bridge/team1_player" + ss.str() + "_penalty";
            memory.insertData(key_penalty,(int)gameCtrlData.teams[1].players[i].penalty);
            std::string key_until_penalized = "gc-bridge/team1_player" + ss.str() + "_secsTillUnpenalised";
            memory.insertData(key_until_penalized,(int)gameCtrlData.teams[1].players[i].secsTillUnpenalised);
          }

          seq = (int)gameCtrlData.packetNumber;
          std::cout << "update value " << seq << " sequence finished" << std::endl;
          memory.raiseEvent("gc-bridge/updatePacket",seq);
          //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
      }
    }
  }
  else
  {
    std::cout << "usage: ./gcbridge <TEAM_NUMBER> <PLAYER_NUMBER>" << std::endl;
  }
  return 0;
}
