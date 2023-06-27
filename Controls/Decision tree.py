def ball_possession(ballPosition, teamPositions, enemyPositions):
    """
    determines which team posesses the ball
    if ball isn't visible -> return 0

    if not ->
    calculate the distance between the ball and each player to check which player might have the ball
    if the distance between one of our team players and the ball is small enough, it means the ball is ours -> return 1
    if the distance between one of our enemy players and the ball is small enough, it means the ball is enemy's -> return 2

    if none of the players have the ball -> return 3

    """
    return 0


ballPosition = [0, 0]  # x and y coord for the ball
teamPositions = [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]
enemyPositions = [[11, 11], [12, 12], [13, 13], [14, 14], [15, 15]]
ballPossession = ball_possession(ballPosition, teamPositions, enemyPositions)

if (ballPossession == 0):
    # use previous ballPosession
    print(0)
elif (ballPossession == 1):
    """
    we possess the ball -> attack formation (see notes)
    if this robot has the ball ->
    if there is no defender in front -> go forward
    else -> if the robot is close enough to the goals or no pass (described later) -> shoot
            else -> search for the best pass:
                    find an ally closest to the enemy goals, find the enemy guarding the ally.
                    search the area on the other side of the ball-ally line (where there isn't a guard) for a spot,
                    which the ally can reach before the guard and pass the ball there
                    if there is no such spot ot it's far, repeat for the next ally closest to the goals

    if an ally has the ball ->move towards the goal (need move algorithm to go around enemies)

    """
    print(1)

elif ballPossession == 2:
    """
    enemy possesses the ball -> defence formation (see notes)
    find an unguarded enemy starting from the one, which has the ball
    attach the robot to that enemy
    if that enemy has the ball -> stand between the ball and the goals close to the enemy 
        (if there is an enemy in this line -> move closer to the goals)
    else -> move to the line between the attached enemy and the ball and then move closer to the goals
    """

    print(2)
else:  # ballPosession == 3
    """
    noone has the ball -> 
    calculate what player will get to the ball first (somehow we need to identify the speed of the enemy's robots, 
        probably dynamically in the first iterations of the cycle)
    if it's our player -> move to the ball, intercept and start dribbling
    elif it's not this exact robot -> move into the starting attack formation (see notes)
    else -> assume the ball is possessed by the enemy closest to the ball and start defence formation 
    """

    print(3)