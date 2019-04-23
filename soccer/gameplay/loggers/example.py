import robocup
import main
import customLogger

class BallLogger(customLogger.CustomLogger):

	def __init__(self):
		pass

	
	def log_data(self):
		pos = main.ball().pos
		return {'Ballx':pos.x, 'Bally':pos.y}