#!/usr/bin/env python3
# John Mann Project 3 Task 2

import sys
from wall_follow import WallFollower


if __name__ == "__main__":
	if len(sys.argv) == 2:
		wall_follower = WallFollower(sys.argv[1])
	if len(sys.argv) == 3:
		wall_follower = WallFollower(sys.argv[1], sys.argv[2])
	elif len(sys.argv) == 4:
		wall_follower = WallFollower(sys.argv[1], sys.argv[2], sys.argv[3])
	else:
		print("please specify mode:")
		print("learn *table* *folder*")
		print("teleop")
		print("test *table*")

