uint8 playerNumber
uint8 friend_teamNumber
uint8 enemy_teamNumber
uint8 friend_teamColour                            #colour of the team
uint8 enemy_teamColour
uint8 TEAM_BLUE = 0
uint8 TEAM_RED = 1
uint8 TEAM_YELLOW = 2
uint8 TEAM_BLACK = 3
uint8 TEAM_WHITE = 4
uint8 TEAM_GREEN = 5
uint8 TEAM_ORANGE = 6
uint8 TEAM_PURPLE = 7
uint8 TEAM_BROWN = 8
uint8 TEAM_GRAY = 9
uint8 TEAM_CYAN = 10
uint8 TEAM_MAGENTA = 11
uint16 secsRemaining
uint8 friend_score
uint8 enemy_score
uint8 state               #state of the game (STATE_READY, STATE_PLAYING, etc)
uint8 STATE_INITIAL = 0
uint8 STATE_READY = 1
uint8 STATE_SET = 2
uint8 STATE_PLAYING = 3
uint8 STATE_FINISHED = 4
uint8 firstHalf           #1 = game in first half, 0 otherwise
uint8 FIRST_HALF = 1
uint8 SECOND_HALF = 0
